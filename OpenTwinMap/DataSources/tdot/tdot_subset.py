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
import hashlib
import time

from DEM_python import DEM
from .tdot_utils import TDOTUtils

import xml.etree.ElementTree as ET


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

    def getMetadataPath(self):
        return os.path.join(self.root_folder, "metadata.json")

    def getOSMPath(self):
        return os.path.join(self.root_folder, self.osm_path)

    def getCorrectedOSMPath(self):
        return os.path.join(
            self.root_folder, "osm_subset_corrected.osm"
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
        return np.array(
            [bbox[0] - margins, bbox[1] - margins, bbox[2] + margins, bbox[3] + margins]
        )

    def loadDEMsFromBoundingBoxMeters(self, bbox, margins=10):
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
