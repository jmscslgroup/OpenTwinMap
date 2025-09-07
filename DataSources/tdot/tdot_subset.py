import json
import os
import subprocess
import numpy as np
import joblib
import shutil
import rtree
import pyproj
from tqdm import tqdm
import open3d
import osmium
import shapely.geometry
import math
import networkx as nx
import hashlib
import time
from .tdot_metadata import TDOTMetadata
from .tdot_utils import TDOTUtils
from .tdot_osm import TDOTOSMCreator
from .tdot_lidar import TDOTLidarCreator
from .tdot_dem import TDOTDEMCreator
from DEM_python import DEM

import xml.etree.ElementTree as ET


class WayNodeCollectorLidarCorrection(osmium.SimpleHandler):
    feet_to_meters = 0.3048
    meters_to_feet = 3.28084
    subset_parent = None

    def __init__(self, minlon, minlat, subset_parent):
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
        self.subset_parent = subset_parent

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
            # node["corrected_coordinates"][2] = self.subset_parent.MinHeightAtXYMeters(dem, (corrected_coordinates_meters[0], corrected_coordinates_meters[1]))

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


class TDOTSubset:
    root_folder = None
    proj = None
    feet_to_meters = 0.3048
    meters_to_feet = 3.28084
    metadata_json = None
    meters_index = None
    coords_index = None
    osm_path = None

    def __getstate__(self):
        state = self.__dict__.copy()
        # Remove unpicklable entries
        del state["proj"]
        del state["meters_index"]
        del state["coords_index"]
        if "osm_handler" in state:
            del state["osm_handler"]
        return state

    def __setstate__(self, state):
        # print("Unpickling....")
        self.__dict__.update(state)
        # Recreate the attributes
        self.proj = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:6576", always_xy=True
        )
        self.meters_index = rtree.Index()
        self.coords_index = rtree.Index()
        for k in self.metadata_json["tiles"]:
            k_int = int(k)
            meters_bounds = self.metadata_json["tiles"][k]["LAZ"]["bounds"]
            coords_bounds = self.metadata_json["tiles"][k]["GEOJSON"]["bounds"]
            meters_bbox = [
                meters_bounds["x"]["min"],
                meters_bounds["y"]["min"],
                meters_bounds["x"]["max"],
                meters_bounds["y"]["max"],
            ]
            coords_bbox = [
                coords_bounds["min"][0],
                coords_bounds["min"][1],
                coords_bounds["max"][0],
                coords_bounds["max"][1],
            ]
            self.meters_index.insert(k_int, meters_bbox)
            self.coords_index.insert(k_int, coords_bbox)

    def __init__(self, root_folder, original_data_path, osm_path="osm_subset.osm"):
        self.osm_path = osm_path
        self.root_folder = root_folder
        self.original_data_path = original_data_path
        self.metadata_json = TDOTUtils.loadJson(self.getMetadataPath())
        self.proj = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:6576", always_xy=True
        )
        self.meters_index = rtree.Index()
        self.coords_index = rtree.Index()
        for k in self.metadata_json["tiles"]:
            k_int = int(k)
            meters_bounds = self.metadata_json["tiles"][k]["LAZ"]["bounds"]
            coords_bounds = self.metadata_json["tiles"][k]["GEOJSON"]["bounds"]
            print(meters_bounds)
            print(coords_bounds)
            meters_bbox = [
                meters_bounds["x"]["min"],
                meters_bounds["y"]["min"],
                meters_bounds["x"]["max"],
                meters_bounds["y"]["max"],
            ]
            coords_bbox = [
                coords_bounds["min"][0],
                coords_bounds["min"][1],
                coords_bounds["max"][0],
                coords_bounds["max"][1],
            ]
            self.meters_index.insert(k_int, meters_bbox)
            self.coords_index.insert(k_int, coords_bbox)

    def processOSM(self):
        min_long, min_lat, max_long, max_lat = (
            self.metadata_json["bounds"]["min_long"],
            self.metadata_json["bounds"]["min_lat"],
            self.metadata_json["bounds"]["max_long"],
            self.metadata_json["bounds"]["max_lat"],
        )
        self.osm_handler = WayNodeCollectorLidarCorrection(min_long, min_lat, self)
        osm_file_path = self.getOSMPath()
        self.osm_handler.apply_file(osm_file_path)

    def getMetadataPath(self):
        return os.path.join(self.root_folder, "metadata.json")

    def getOSMPath(self):
        return os.path.join(self.root_folder, self.osm_path)

    def getCorrectedOSMPath(self):
        return os.path.join(
            self.root_folder, "osm_subset_corrected_all_lidar_for_elevation.osm"
        )
        # return os.path.join(self.root_folder, "osm_subset_corrected.osm")

    def getDEMPath(self, tile):
        return os.path.join(
            self.root_folder, self.metadata_json["tiles"][tile]["DEM"]["path"]
        )

    def getLAZPath(self, tile):
        return os.path.join(
            self.root_folder, self.metadata_json["tiles"][tile]["LAZ"]["path"]
        )

    def getXODRPath(self):
        return os.path.join(self.root_folder, "map.xodr")

    def getBoundsInCoords(self):
        all_tiles = self.getAllTiles()
        southwest_coordinates = [
            self.metadata_json["tiles"][tile]["GEOJSON"]["bounds"]["min"]
            for tile in all_tiles
        ]
        southwest_bound = getSouthWestCoordinate(southwest_coordinates)
        northeast_coordinates = [
            self.metadata_json["tiles"][tile]["GEOJSON"]["bounds"]["max"]
            for tile in all_tiles
        ]
        northeast_bound = getNorthEastCoordinate(northeast_coordinates)
        return np.array(
            [
                southwest_bound[0],
                southwest_bound[1],
                northeast_bound[0],
                northeast_bound[1],
            ]
        )

    def getBoundsInMeters(self):
        bounds_coords = self.getBoundsInCoords()
        result = [0, 0, 0, 0]
        result[:2] = self.convertToMeters(bounds_coords[:2])
        result[2:] = self.convertToMeters(bounds_coords[2:])
        return np.array(result)

    def getAllTiles(self):
        return [k for k in self.metadata_json["tiles"]]

    def loadDEM(self, tile):
        dem_path = self.getDEMPath(tile)
        return DEM.from_csv(dem_path, 2.0, -999999)

    def processLAZ(self, pcd_points):
        # x, y = pcd_points.get_min_bound()[0], pcd_points.get_min_bound()[1]
        # pcd_points.translate((-x, -y, 0))
        # pcd_points_points = np.asarray(pcd_points.points)
        # pcd_points_points *= feet_to_meters
        # pcd_points.points = open3d.utility.Vector3dVector(pcd_points_points)
        # pcd_points.voxel_down_sample(voxel_size=1.0)
        pcd_points.estimate_normals(
            search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
        )
        pcd_points.normalize_normals()
        pcd_points.paint_uniform_color([0.3, 0.3, 0.3])
        return pcd_points

    def loadLAZ(self, tile, process=False):
        pcd_path = self.getLAZPath(tile)
        pcd_points = open3d.io.read_point_cloud(pcd_path)
        if process:
            pcd_points = self.processLAZ(pcd_points)
        return pcd_points

    def loadDEMs(self, tiles):
        dems = [self.loadDEM(tile) for tile in tiles]
        return DEM.from_dems(dems, 2.0, -999999)

    def loadLAZs(self, tiles, process=False, voxel_size=None, every_k_points=None):
        pcds = [self.loadLAZ(tile, process) for tile in tiles]
        if voxel_size is not None:
            for i in range(len(pcds)):
                pcds[i] = pcds[i].voxel_down_sample(voxel_size=voxel_size)
        elif every_k_points is not None:
            for i in range(len(pcds)):
                pcds[i] = pcds[i].uniform_down_sample(every_k_points=every_k_points)
        return pcds

    def convertToMeters(self, coord):
        x, y = self.proj.transform(coord[0], coord[1])
        x *= self.feet_to_meters
        y *= self.feet_to_meters
        return np.array([x, y])

    def convertToCoords(self, meters):
        x, y = self.proj.transform(
            meters[0] * self.meters_to_feet,
            meters[1] * self.meters_to_feet,
            direction="INVERSE",
        )
        return np.array([x, y])

    def _augmentBBoxWithMargins(self, bbox, margins):
        # print(bbox)
        return np.array(
            [bbox[0] - margins, bbox[1] - margins, bbox[2] + margins, bbox[3] + margins]
        )

    def loadDEMsFromBoundingBoxMeters(self, bbox, margins=10):
        # print(bbox)
        bbox = self._augmentBBoxWithMargins(bbox, margins)
        tiles = self.getTilesFromBoundingBoxMeters(bbox)
        dems = self.loadDEMs(tiles)
        cropped_dems = DEM.clip_dem(
            dems,
            [bbox[0] * self.meters_to_feet, bbox[1] * self.meters_to_feet],
            [bbox[2] * self.meters_to_feet, bbox[3] * self.meters_to_feet],
            margins=margins * self.meters_to_feet,
        )
        return cropped_dems

    def loadLAZsFromBoundingBoxMeters(self, bbox, margins=10):
        bbox = self._augmentBBoxWithMargins(bbox, margins)
        tiles = self.getTilesFromBoundingBoxMeters(bbox)
        pcds = self.loadLAZs(tiles)
        bottom_left = bbox[:2]
        top_right = bbox[2:]
        aabb = open3d.geometry.AxisAlignedBoundingBox(
            np.array([bottom_left[0], bottom_left[1], float("-inf")]),
            np.array([top_right[0], top_right[1], float("inf")]),
        )
        cropped_pcds = [pcd.crop(aabb) for pcd in pcds]
        merged_pcd = open3d.geometry.PointCloud()
        for pcd in cropped_pcds:
            merged_pcd += pcd
        return self.processLAZ(merged_pcd)

    def getTilesFromBoundingBoxMeters(self, bbox):
        return [str(k) for k in self.meters_index.intersection(bbox)]

    def loadDEMsFromBoundingBoxCoords(self, bbox):
        bbox_meters = self.convertToMeters(bbox)
        return self.loadDEMsFromBoundingBoxMeters(bbox_meters)

    def loadLAZsFromBoundingBoxCoords(self, bbox):
        bbox_meters = self.convertToMeters(bbox)
        return self.loadLAZsFromBoundingBoxMeters(bbox_meters)

    def getTilesFromBoundingBoxCoords(self, bbox):
        bbox_meters = self.convertToMeters(bbox)
        return self.getTilesFromBoundingBoxMeters(bbox_meters)

    def minHeightAtXYMeters(self, dem, xy_coord):
        # Extract X, Y, Z
        x, y = xy_coord
        result = (
            dem.altitude((x * self.meters_to_feet), (y * self.meters_to_feet))
            * self.feet_to_meters
        )
        return result

    def lidarMedianHeightAtXYMeters(self, pcd_points, xy_coord, bbox_size=10.0):
        delta = bbox_size / 2.0
        bbox = open3d.geometry.AxisAlignedBoundingBox(
            min_bound=[xy_coord[0] - delta, xy_coord[1] - delta, float("-inf")],
            max_bound=[xy_coord[0] + delta, xy_coord[1] + delta, float("inf")],
        )

        # Crop point cloud to bounding box
        cropped = pcd_points.crop(bbox)

        # Get numpy array of cropped points
        points = np.asarray(cropped.points)

        # Get median point by Z
        median_height = None
        if len(points) == 0:
            print("No points found in bounding box.")
        else:
            median_height = np.median(points[:, 2])
            # print("Median point:", median_height)
        return median_height

    def lidarMaxHeightAtXYMeters(self, pcd_points, xy_coord, bbox_size=1.0):
        delta = bbox_size / 2.0
        bbox = open3d.geometry.AxisAlignedBoundingBox(
            min_bound=[xy_coord[0] - delta, xy_coord[1] - delta, float("-inf")],
            max_bound=[xy_coord[0] + delta, xy_coord[1] + delta, float("inf")],
        )

        # Crop point cloud to bounding box
        cropped = pcd_points.crop(bbox)

        # Get numpy array of cropped points
        points = np.asarray(cropped.points)

        # Get highest point by Z
        highest_point = None
        if len(points) == 0:
            print("No points found in bounding box.")
            return
        else:
            max_idx = np.argmax(points[:, 2])  # Z-axis
            highest_point = points[max_idx]
            print("Highest point:", highest_point)
        return highest_point[-1]


class TDOTSubsetCreator:
    original_data_path = None
    subset_folder = None
    metadata_obj = None
    metadata_json = None
    osm_obj = None
    lidar_obj = None
    dem_obj = None
    metadata_path = None

    def __init__(self, original_data_path, subset_folder):
        self.original_data_path = original_data_path
        self.subset_folder = subset_folder
        self.metadata_path = os.path.join(self.subset_folder, "metadata.json")
        os.makedirs(self.subset_folder, exist_ok=True)

    def compileDEMSubset(self):
        if self.metadata_obj is None:
            return
        self.dem_obj = TDOTDEMCreator(self.subset_folder)
        self.dem_obj.compileDEMSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileLidarSubset(self):
        if self.metadata_obj is None:
            return
        self.lidar_obj = TDOTLidarCreator(self.subset_folder)
        self.lidar_obj.compileLidarSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileOSMSubset(self):
        if self.metadata_obj is None:
            return
        self.osm_obj = TDOTOSMCreator(self.subset_folder)
        self.osm_obj.compileOSMSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileMetadata(self):
        self.metadata_obj = TDOTMetadata(self.original_data_path)
        self.metadata_json = self.metadata_obj.compileMetadata()
        TDOTUtils.writeJson(self.metadata_path, self.metadata_obj)
