import pyproj
import networkx as nx
import osmium

meters_to_feet = 3.28084
feet_to_meters = 1.0 / meters_to_feet

class WayNodeCollectorLidarCorrection(osmium.SimpleHandler):
    feet_to_meters = 0.3048
    meters_to_feet = 3.28084

    def __init__(self, minlon, minlat):
        super().__init__()
        self.nodes = {}  # id → (lat, lon)
        self.ways_original = {}
        self.ways = {}
        self.node_graph = nx.Graph()
        self.proj = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:6576", always_xy=True
        )
        self.osm_origin_x, self.osm_origin_y = self.proj.transform(minlon, minlat)
        self.osm_origin_x *= self.feet_to_meters
        self.osm_origin_y *= self.feet_to_meters

    def node(self, n):
        n_id = str(n.id)
        elevation = 0.0
        if "ele" in n.tags:
            elevation = float(n.tags["ele"])
            print(elevation)
        coordinates = [n.location.lon, n.location.lat, elevation]
        meters_coordinates = self.projectToMeters(coordinates)
        self.nodes[n_id] = {
            "coordinates": coordinates,
            "meters_coordinates": meters_coordinates,
            "total_ways": [],
            "corrected_coordinates": np.array([0, 0, 0]),
            "lane_widths": [],
            "lane_counts": [],
            "bridge": False,
        }

    def way(self, w):
        w_id = str(w.id)
        if "highway" in w.tags:
            bridge = ("bridge" in w.tags) and (w.tags["bridge"] == "yes")
            lane_width_by_highway = {
                "motorway": 3.75,
                "motorway_link": 3.5,
                "primary": 3.5,
                "secondary": 3.0,
                "tertiary": 2.8,
                "residential": 2.7,
                "service": 2.5,
            }
            lane_type = w.tags["highway"]
            lane_width = (
                float(lane_width_by_highway[lane_type])
                if lane_type in lane_width_by_highway
                else 3.65
            )
            self.ways_original[w_id] = {
                "nodes": [],
                "corrected_node_positions": [],
                "lane_count": [],
                "lane_width": [lane_width for n in w.nodes],
            }
            self.ways_original[w_id]["bridge"] = [bridge for n in w.nodes]
            nodes = []
            lanes = w.tags.get("lanes")
            if lanes is not None:
                try:
                    lane_count = int(lanes)
                    self.ways_original[w_id]["lane_count"] = [
                        lane_count for n in w.nodes
                    ]
                except ValueError:
                    self.ways_original[w_id]["lane_count"] = [1 for n in w.nodes]
            else:
                self.ways_original[w_id]["lane_count"] = [1 for n in w.nodes]
            for n in w.nodes:
                n_ref = str(n.ref)
                self.ways_original[w_id]["nodes"].append(n_ref)
                self.ways_original[w_id]["corrected_node_positions"].append(
                    np.array([0, 0, 0])
                )  # Default of zero coordinate
                self.nodes[n_ref]["total_ways"].append(w_id)
                self.nodes[n_ref]["lane_counts"].append(
                    self.ways_original[w_id]["lane_count"][0]
                )
                self.nodes[n_ref]["lane_widths"].append(lane_width)
                self.nodes[n_ref]["bridge"] = self.nodes[n_ref]["bridge"] or bridge
                nodes.append(n_ref)
            for node1, node2 in zip(nodes[:-1], nodes[1:]):
                self.node_graph.add_edge(node1, node2)
            self.ways[w_id] = self.ways_original[w_id].copy()

    def generateImplicitWays(self, node_count=15, distance_bound=200, cores=48):
        all_segments = []

        def findPathsFromSource(node_graph, source, node_count):
            segments = []
            for target in node_graph.nodes:
                if source == target:
                    continue
                for path in nx.all_simple_paths(
                    node_graph, source=source, target=target, cutoff=node_count - 1
                ):
                    if len(path) == node_count:
                        # To avoid duplicates: enforce an ordering
                        if path[0] < path[-1]:
                            segments.append(path)
            return segments

        def getMinMaxMeterDistanceOfWay(segment):
            x_points = []
            y_points = []
            for node in segment:
                meters_coordinates = self.nodes[node]["meters_coordinates"]
                x_points.append(meters_coordinates[0])
                y_points.append(meters_coordinates[1])
            x_min, x_max = min(x_points), max(x_points)
            y_min, y_max = min(y_points), max(y_points)
            return math.sqrt(math.pow(x_max - x_min, 2) + math.pow(y_max - y_min, 2))

        sources = self.node_graph.nodes
        all_segments_lists = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=cores)(
                    joblib.delayed(findPathsFromSource)(
                        self.node_graph, source, node_count
                    )
                    for source in sources
                ),
                total=len(sources),
            )
        )
        all_segments = [item for sublist in all_segments_lists for item in sublist]
        accepted_count = 0
        for segment in all_segments:
            if getMinMaxMeterDistanceOfWay(segment) > distance_bound:
                continue
            accepted_count += 1
            segment_hashes = [
                hashlib.sha256(node_id.encode()).hexdigest() for node_id in segment
            ]
            segment_way_id = hashlib.sha256(
                "".join(segment_hashes).encode()
            ).hexdigest()
            self.ways[segment_way_id] = {
                "nodes": [],
                "corrected_node_positions": [],
                "lane_count": [
                    np.mean(self.nodes[node]["lane_counts"]) for node in segment
                ],
                "lane_width": [
                    np.mean(self.nodes[node]["lane_widths"]) for node in segment
                ],
                "bridge": [self.nodes[node]["bridge"] for node in segment],
            }
            for node in segment:
                self.ways[segment_way_id]["nodes"].append(node)
                self.ways[segment_way_id]["corrected_node_positions"].append(
                    np.array([0, 0, 0])
                )  # Default of zero coordinate
                self.nodes[node]["total_ways"].append(segment_way_id)

        print(f"Found {accepted_count} unique segments of {node_count} nodes.")

    def getWayNodes(self, wid):
        return self.ways[wid]["nodes"]

    def projectToMeters(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x, y = self.proj.transform(x, y)
        x = x * self.feet_to_meters
        y = y * self.feet_to_meters
        return np.array([x, y, z])

    def projectToOSM(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x = x * self.meters_to_feet
        y = y * self.meters_to_feet
        x, y = self.proj.transform(x, y, direction="INVERSE")
        return np.array([x, y, z])

    def getWayCoordinates(self, wid, project_to_meters=True):
        result = []
        for node_entry in self.getWayNodes(wid):
            coordinates = self.nodes[node_entry]["coordinates"]
            x, y, z = coordinates[0], coordinates[1], coordinates[2]
            if project_to_meters:
                meters = self.projectToMeters(coordinates)
                x, y = meters[0], meters[1]
            result.append([x, y, z])
        return np.array(result)

    def getWayBoundingBox(self, wid, project_to_meters=True):
        way_coordinates = self.getWayCoordinates(
            wid, project_to_meters=project_to_meters
        )
        bottom_left = way_coordinates[0][:2].tolist()
        top_right = way_coordinates[0][:2].tolist()
        for i in range(1, len(way_coordinates)):
            x, y, z = way_coordinates[i]
            if bottom_left[0] > x:
                bottom_left[0] = x
            if bottom_left[1] > y:
                bottom_left[1] = y
            if top_right[0] < x:
                top_right[0] = x
            if top_right[1] < y:
                top_right[1] = y
        return np.array([bottom_left[0], bottom_left[1], top_right[0], top_right[1]])

    def annotateWayWithCorrectedPoints(self, wid, way_coordinates):
        for i in range(len(way_coordinates)):
            self.ways[wid]["corrected_node_positions"][i] = way_coordinates[i]

    def correctNodePoints(self, dem):
        for wid in self.ways:
            way = self.ways[wid]
            for i in range(len(way["nodes"])):
                nid = way["nodes"][i]
                corrected_nid = way["corrected_node_positions"][i]
                self.nodes[nid]["corrected_coordinates"] = (
                    self.nodes[nid]["corrected_coordinates"] + corrected_nid
                )
                # print(corrected_nid, len(self.nodes[nid]["corrected_coordinates"]))
        for nid in self.nodes:
            node = self.nodes[nid]
            total_ways = len(node["total_ways"])
            # total_ways = 0
            if total_ways > 0:
                node["corrected_coordinates"] = self.projectToOSM(
                    node["corrected_coordinates"] / total_ways
                )
            else:
                node["corrected_coordinates"] = node["coordinates"]
            corrected_coordinates_meters = self.projectToMeters(
                node["corrected_coordinates"]
            )

    def createCorrectedOSMFile(self, original_osm_file, target_osm_file):
        # Parse XML using ElementTree to directly update lat/lon
        tree = ET.parse(original_osm_file)
        root = tree.getroot()

        for elem in root.findall("node"):
            nid = str(elem.attrib["id"])
            if nid in self.nodes:
                lon, lat, height = (
                    self.nodes[nid]["corrected_coordinates"][0],
                    self.nodes[nid]["corrected_coordinates"][1],
                    self.nodes[nid]["corrected_coordinates"][2],
                )
                elem.set("lat", f"{lat:.8f}")
                elem.set("lon", f"{lon:.8f}")
                ele_tag = ET.SubElement(elem, "tag")
                ele_tag.set("k", "ele")
                ele_tag.set("v", f"{height:.8f}")

        # Output file
        tree.write(target_osm_file, encoding="utf-8", xml_declaration=True)