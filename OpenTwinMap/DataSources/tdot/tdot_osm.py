import osmium
import time
import os
import math
import subprocess
import pyproj
import shapely
import networkx as nx
import joblib
import xml.etree.ElementTree as ET
from .tdot_metadata import TDOTMetadata


class WayNodeCollectorNodeBoundClipping(osmium.SimpleHandler):

    def __init__(self, bounds, buffer_size=0.001):
        self.proj = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:6576", always_xy=True
        )
        bounds_min = self.projectCoordinateToMeters(bounds[:2], convert_to_origin=False)
        bounds_max = self.projectCoordinateToMeters(bounds[2:], convert_to_origin=False)
        bounds_max[0] -= bounds_min[0]
        bounds_max[1] -= bounds_min[1]
        self.origin = bounds_min
        self.bounds = shapely.geometry.box(0, 0, bounds_max[0], bounds_max[1])
        self.inside_intersection = shapely.geometry.box(
            buffer_size,
            buffer_size,
            bounds_max[0] - buffer_size,
            bounds_max[1] - buffer_size,
        )
        self.nodes = {}
        self.ways = {}
        self.outside_points = {}
        self.node_graph = nx.Graph()

    def projectCoordinateToMeters(self, coordinate, convert_to_origin=True):
        result = list(self.proj.transform(coordinate[0], coordinate[1]))
        if convert_to_origin:
            result[0] -= self.origin[0]
            result[1] -= self.origin[1]
        return result

    def projectMetersToCoordinate(self, meters):
        return list(
            self.proj.transform(
                meters[0] + self.origin[0],
                meters[1] + self.origin[1],
                direction="INVERSE",
            )
        )

    def node(self, n):
        n_id = str(n.id)
        self.nodes[n_id] = {"coordinates": [n.location.lon, n.location.lat], "ways": []}

    def way(self, w):
        w_id = str(w.id)
        nodes = []
        for n in w.nodes:
            n_ref = str(n.ref)
            nodes.append(n_ref)
            self.nodes[n_ref]["ways"].append(w_id)
        for node1, node2 in zip(nodes[:-1], nodes[1:]):
            self.node_graph.add_edge(node1, node2)
        self.ways[w_id] = {"nodes": nodes}

    def containsPoint(self, coordinate, tolerance=0.0000):
        coordinates_transformed = self.projectCoordinateToMeters(coordinate)
        coordinate_point = shapely.geometry.Point(
            coordinates_transformed[0], coordinates_transformed[1]
        )
        return self.bounds.contains(coordinate_point) or (
            self.bounds.distance(coordinate_point) < tolerance
        )

    def getCorrectedSecondNodeCoordinate(self, node1_location, node2_location):
        node1_location = self.projectCoordinateToMeters(node1_location)
        node2_location = self.projectCoordinateToMeters(node2_location)
        pair_string = shapely.geometry.LineString([node2_location, node1_location])
        result = self.bounds.intersection(pair_string).coords
        print(
            node1_location,
            node2_location,
            list(result),
            self.bounds,
            self.bounds.boundary.distance(shapely.geometry.Point(node1_location)),
            self.bounds.boundary.distance(shapely.geometry.Point(node2_location)),
        )
        return self.projectMetersToCoordinate(list(result[0]))

    def clipToInside(self):
        for node1, neighbors in self.node_graph.adjacency():
            node1_location = self.nodes[node1]["coordinates"]
            if self.containsPoint(node1_location):
                neighbor_list = list(neighbors.keys())
                for node2 in neighbor_list:
                    node2_location = self.nodes[node2]["coordinates"]
                    if not self.containsPoint(node2_location):
                        node2_corrected_location = (
                            self.getCorrectedSecondNodeCoordinate(
                                node1_location, node2_location
                            )
                        )
                        self.nodes[node1][
                            "corrected_coordinates"
                        ] = node2_corrected_location
                        print(f"Node {node1} corrected to {node2_corrected_location}")

    def removeExternalPoints(self):
        for node in self.nodes:
            node_coordinate = self.nodes[node]["coordinates"]
            if not self.containsPoint(node_coordinate, tolerance=0.01):
                self.outside_points[node] = node

    def correctOSMFile(self, tree):
        root = tree.getroot()

        # 1. Remove outside nodes
        for node in root.findall("node"):
            n_id = str(node.get("id"))
            if n_id in self.outside_points:
                root.remove(node)

        # 2. Update coordinates of inside nodes
        for node in root.findall("node"):
            n_id = str(node.get("id"))
            lon, lat = (
                self.nodes[n_id]["corrected_coordinates"]
                if "corrected_coordinates" in self.nodes[n_id]
                else self.nodes[n_id]["coordinates"]
            )
            node.set("lon", str(lon))
            node.set("lat", str(lat))

        # 3. Remove <nd> references to deleted nodes in ways
        for way in root.findall("way"):
            nds = way.findall("nd")
            for nd in nds:
                ref = str(nd.get("ref"))
                if ref in self.outside_points:
                    way.remove(nd)

            # Optional: remove empty-low node count ways
            if len(way.findall("nd")) < 2:
                print(f"Removing way {way}")
                root.remove(way)

        return tree
    
class TDOTOSMWayInterpolate(osmium.SimpleHandler):
    MAX_DIST_M = 100.0   # <= 0.1 km target
    HIGHWAY_WHITELIST = {
        "motorway", "trunk", "primary", "secondary", "tertiary",
        "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
    }
    def __init__(self, bounds):
        self.proj = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:6576", always_xy=True
        )
        bounds_min = self.projectCoordinateToMeters(bounds[:2], convert_to_origin=False)
        self.origin = bounds_min
        self.nodes = {}
        self.new_nodes = {}
        self.ways = {}
        self.next_neg_node_id = -1

    def projectCoordinateToMeters(self, coordinate, convert_to_origin=True):
        result = list(self.proj.transform(coordinate[0], coordinate[1]))
        if convert_to_origin:
            result[0] -= self.origin[0]
            result[1] -= self.origin[1]
        return result

    def projectMetersToCoordinate(self, meters):
        return list(
            self.proj.transform(
                meters[0] + self.origin[0],
                meters[1] + self.origin[1],
                direction="INVERSE",
            )
        )

    def node(self, n):
        n_id = str(n.id)
        n_id_int = int(n_id)
        if n_id_int < self.next_neg_node_id:
            self.next_neg_node_id = n_id_int - 1
        self.nodes[n_id] = {"coordinates": [n.location.lon, n.location.lat], "ways": []}

    def way(self, w):
        highway_type = w.tags["highway"] if "highway" in w.tags else "None"
        if highway_type not in self.HIGHWAY_WHITELIST:
            return
        w_id = str(w.id)
        nodes = []
        for n in w.nodes:
            n_ref = str(n.ref)
            nodes.append(n_ref)
        self.ways[w_id] = {"nodes": nodes, "highway_type": highway_type}

    # We assume the same elevation here. Should be more than fine for our purposes.
    def calculateDistanceMeters(self, coordinate1, coordinate2):
        coordinate1_meters = self.projectCoordinateToMeters(coordinate1)
        coordinate2_meters = self.projectCoordinateToMeters(coordinate2)
        return math.sqrt(((coordinate1_meters[0] - coordinate2_meters[0])**2) + ((coordinate1_meters[1] - coordinate2_meters[1])**2))

    def densifyWays(self):
        inserted_nodes_total = 0
        densified_ways = 0
        for w_id in self.ways:
            nd_elems = self.ways[w_id]["nodes"]
            if len(nd_elems) < 2:
                continue

            missing = False
            for nd in nd_elems:
                if nd not in self.nodes:
                    missing = True
                    break
            if missing:
                continue

            new_nd_refs = []
            changed = False

            for i in range(len(nd_elems) - 1):
                ref1 = nd_elems[i]
                ref2 = nd_elems[i + 1]
                ref1_coordinates = self.nodes[ref1]["coordinates"]
                ref2_coordinates = self.nodes[ref2]["coordinates"]

                if (i == 0):
                    new_nd_refs.append(ref1)

                d = self.calculateDistanceMeters(ref1_coordinates, ref2_coordinates)

                if d > self.MAX_DIST_M:
                    # number of inserts so that all segments <= MAX_DIST_M
                    # segments = n+1; (d / (n+1)) <= MAX_DIST_M => n >= ceil(d/MAX_DIST_M) - 1
                    n_insert = max(1, math.ceil(d / self.MAX_DIST_M) - 1)
                    # Insert n points at fractions j/(n+1), j=1..n
                    for j in range(1, n_insert + 1):
                        f = j / float(n_insert + 1)
                        lon, lat = self.interpolateLongLat(ref1_coordinates, ref2_coordinates, f)

                        # Create new <node> with negative ID
                        new_node = ET.Element("node")
                        new_node.set("id", str(self.next_neg_node_id))
                        new_node.set("version", "1")
                        new_node.set("lon", f"{lon:.8f}")
                        new_node.set("lat", f"{lat:.8f}")
                        self.new_nodes[str(self.next_neg_node_id)] = new_node
                        
                        # Append to way's nd sequence
                        new_nd_refs.append(str(self.next_neg_node_id))

                        self.next_neg_node_id -= 1
                        inserted_nodes_total += 1
                        changed = True

                # Always append the second original node of the pair
                new_nd_refs.append(ref2)
            self.ways[w_id]["nodes"] = new_nd_refs
            if changed:
                densified_ways += 1
        
    @staticmethod
    def interpolateLongLat(p1, p2, f):
        """Linear interpolation in lon/lat space: p = (1-f)*p1 + f*p2"""
        (lon1, lat1), (lon2, lat2) = p1, p2
        return (lon1 + f*(lon2 - lon1), lat1 + f*(lat2 - lat1))
    
    def getLastNodeIndex(self, root):
        nodes = root.findall("node")
        last_node = nodes[-1]
        return list(root).index(last_node)
    
    def updateOSMFile(self, tree):
        root = tree.getroot()
        insert_index = self.getLastNodeIndex(root) + 1
        # Add new nodes
        for new_node in self.new_nodes:
            root.insert(insert_index, self.new_nodes[new_node])

        # Update Ways to have new node lists
        for way in root.iter("way"):
            w_id = way.get("id")
            if w_id not in self.ways:
                continue
            for nd in list(way.findall("nd")):
                way.remove(nd)

            children = list(way)
            first_tag_idx = next((i for i, c in enumerate(children) if c.tag == 'tag'), None)

            def make_nd(ref):
                e = ET.Element("nd")
                e.set("ref", str(ref))
                return e
        
            new_nd_elems = [make_nd(ref) for ref in self.ways[w_id]["nodes"]]

            if first_tag_idx is None:
                for e in new_nd_elems:
                    way.append(e)
            else:
                idx = first_tag_idx
                for e in new_nd_elems:
                    way.insert(idx, e)
                    idx += 1
        return tree

            


class TDOTOSMCreator:
    root_folder: str = None
    osm_folder: str = None

    def __init__(self, root_folder, bounds_bbox):
        self.root_folder = root_folder
        self.osm_folder = os.path.join(root_folder, "osm/")
        os.makedirs(self.osm_folder, exist_ok=True)

    @staticmethod
    def downloadOSM(osm_folder, metadata, tile):
        tile = str(tile)
        metadata_entry = metadata["tiles"][tile]
        bounding_box = TDOTMetadata.getBoundingBox(metadata_entry)
        osm_path = os.path.join(osm_folder, tile + ".osm")
        print("Saving path ", osm_path)
        print("Bounding box ", bounding_box)
        print("Invoking ", "https://api.openstreetmap.org/api/0.6/map?bbox=")
        api_path = "https://api.openstreetmap.org/api/0.6/map?bbox={},{},{},{}".format(
            bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]
        )
        subprocess.run(["wget", "-O", osm_path, api_path], shell=False)
        time.sleep(5.0)

    def compileOSMSubset(self, metadata, subset):

        merged_osm_original = os.path.join(
            self.root_folder, "osm_subset_merged_no_bounds.osm"
        )
        merged_osm_interp = os.path.join(
            self.root_folder, "osm_subset_merged_interp.osm"
        )
        merged_osm_final = os.path.join(self.root_folder, "osm_subset.osm")

        parallel_result = joblib.Parallel(n_jobs=2, backend="multiprocessing")(
            joblib.delayed(TDOTOSMCreator.downloadOSM)(self.osm_folder, metadata, tile)
            for tile in subset
        )

        commands = ["osmium", "merge"]
        for tile in subset:
            tile = str(tile)
            osm_path = os.path.join(self.osm_folder, tile + ".osm")
            commands.append(osm_path)
        commands.append("-o")
        commands.append(merged_osm_original)
        subprocess.run(commands, shell=False)

        min_long, min_lat, max_long, max_lat = TDOTMetadata.getFinalOSMBound(
            metadata, subset
        )
        tree = ET.parse(merged_osm_original)

        # Way interpolation
        node_interpolate = TDOTOSMWayInterpolate(
            [min_long, min_lat, max_long, max_lat]
        )
        node_interpolate.apply_file(merged_osm_original)
        node_interpolate.densifyWays()
        tree = node_interpolate.updateOSMFile(tree)
        tree.write(merged_osm_interp, encoding="utf-8", xml_declaration=True)

        # Bound clipping
        tree = ET.parse(merged_osm_interp)
        clipping_processor = WayNodeCollectorNodeBoundClipping(
            [min_long, min_lat, max_long, max_lat]
        )
        clipping_processor.apply_file(merged_osm_interp)
        clipping_processor.clipToInside()
        clipping_processor.removeExternalPoints()
        tree = clipping_processor.correctOSMFile(tree)
        root = tree.getroot()
        current_bounds_tag = root.findall("bounds")
        if len(current_bounds_tag) == 0:
            bounds = ET.Element(
                "bounds",
                {
                    "minlat": f"{min_lat:.8f}",
                    "minlon": f"{min_long:.8f}",
                    "maxlat": f"{max_lat:.8f}",
                    "maxlon": f"{max_long:.8f}",
                },
            )
            root.insert(0, bounds)
        else:
            current_bounds_tag.set("minlat", f"{min_lat:.8f}")
            current_bounds_tag.set("minlon", f"{min_long:.8f}")
            current_bounds_tag.set("maxlat", f"{max_lat:.8f}")
            current_bounds_tag.set("maxlon", f"{max_long:.8f}")


        tree.write(merged_osm_final, encoding="utf-8", xml_declaration=True)

