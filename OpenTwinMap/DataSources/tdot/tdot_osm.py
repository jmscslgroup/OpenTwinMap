import osmium
import time
import os
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


class TDOTOSMCreator:
    root_folder: str = None
    osm_folder: str = None

    def __init__(self, root_folder):
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

        clipping_processor = WayNodeCollectorNodeBoundClipping(
            [min_long, min_lat, max_long, max_lat]
        )
        clipping_processor.apply_file(merged_osm_original)
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
