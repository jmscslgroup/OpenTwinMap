import osmium
import open3d
import pyproj
import numpy as np
import math
import xml.etree.ElementTree as ET
import joblib
import time
import hashlib
import sys
from ..DataSources.tdot.tdot_subset import TDOTSubset
from scipy.signal import savgol_filter
from copy import deepcopy
from tqdm import tqdm
from ..DEM_python import DEM
from .utils import WayNodeCollectorLidarCorrection, meters_to_feet, feet_to_meters

class OSMLidarCorrection:

    @staticmethod
    def LidarGetAverageMeterSpacing(pcd):
        points = np.asarray(pcd.points)

        # Use built-in KDTree to estimate spacing
        pcd_tree = open3d.geometry.KDTreeFlann(pcd)

        nn_dists = []
        for i in range(len(points)):
            [_, idx, dist2] = pcd_tree.search_knn_vector_3d(
                points[i], 2
            )  # 2 nearest:  itself + 1 neighbor
            if len(dist2) > 1:
                nn_dists.append(np.sqrt(dist2[1]))  # exclude distance to self

        # Estimate average spacing
        avg_spacing = np.mean(nn_dists)
        print(f"Estimated average point spacing: {avg_spacing:.2f} meters")

        return avg_spacing

    @staticmethod
    def UpsampleLine(points, target_spacing):
        """
        Upsample a polyline to match a desired point spacing.
        Input: points = (N, 2) or (N, 3) array
        Output: (M, 3) array with interpolated points
        """
        points = np.array(points)
        upsampled = []

        for i in range(len(points) - 1):
            p0, p1 = points[i], points[i + 1]
            # print(p0, p1)
            segment_vec = p1 - p0
            segment_len = np.linalg.norm(segment_vec)

            # Determine how many points to insert
            num_insert = max(int(np.floor(segment_len / target_spacing)), 1)
            for j in range(num_insert):
                alpha = j / num_insert
                interp_point = p0 + alpha * segment_vec
                upsampled.append(interp_point)

        upsampled.append(points[-1])  # Always include last point
        return np.array(upsampled)

    @staticmethod
    def DownsampleLine(points, target_number):
        indices = np.linspace(0, len(points) - 1, target_number, dtype=int)
        return points[indices]

    @staticmethod
    def ExpandCenterlineToDenseRibbon(
        centerline_xyz,
        lanes,
        oneway,
        lanes_backward,
        lanes_forward,
        lane_width,
        shoulder_width=1.8,
        width_resolution=0.21,
        length_resolution=0.21,
    ):
        points = []

        for i in range(len(centerline_xyz) - 1):
            p0 = centerline_xyz[i]
            p1 = centerline_xyz[i + 1]

            # Segment direction
            direction = p1[:2] - p0[:2]
            length = np.linalg.norm(direction)
            if length == 0:
                continue
            direction /= length

            # Perpendicular (2D) vector
            perp = np.array([direction[1], -direction[0]])

            # Number of samples
            n_length = max(2, int(length / length_resolution))
            '''
            n_width = max(
                2,
                int(((lane_width[i] * lanes[i]) + (2 * shoulder_width)) / width_resolution),
            )
            '''
            n_width_backward = int((lane_width[i] * lanes_backward[i] + shoulder_width) / width_resolution)
            n_width_forward = int((lane_width[i] * lanes_forward[i] + shoulder_width)  / width_resolution)
            # We kludge here and split into half backward, half forward. Othertimes we assume the centerline is split between forwards and backwards
            '''
            if oneway:
                lanes_backward_i = lanes[i] // 2
                lanes_forward_i = lanes[i] - lanes_backward_i
                n_width_backward = int((lane_width[i] * lanes_backward_i + shoulder_width) / width_resolution)
                n_width_forward = int((lane_width[i] * lanes_forward_i + shoulder_width)  / width_resolution)
            '''
            for j in range(n_length + 1):
                alpha = j / n_length
                point_center = (1 - alpha) * p0 + alpha * p1
                z = point_center[2]

                for k in range(-n_width_backward, 0):
                    offset = (k * width_resolution) * perp
                    x, y = point_center[:2] + offset
                    points.append([x, y, z])
                for k in range(0, n_width_forward):
                    offset = (k * width_resolution) * perp
                    x, y = point_center[:2] + offset
                    points.append([x, y, z])
                '''
                for k in range(-n_width // 2, n_width // 2 + 1):
                    offset = (k * width_resolution) * perp
                    x, y = point_center[:2] + offset
                    points.append([x, y, z])
                '''

        return np.array(points)

    @staticmethod
    def SmoothSavgol(centerline_xyz, window_length=21, polyorder=3):
        x = centerline_xyz[:, 0]
        y = centerline_xyz[:, 1]
        z = centerline_xyz[:, 2]
        if (len(z) < window_length) or (np.isnan(z).any()):
            return centerline_xyz  # not enough points to smooth

        x_smooth = savgol_filter(x, window_length, polyorder)
        y_smooth = savgol_filter(y, window_length, polyorder)
        z_smooth = savgol_filter(z, window_length, polyorder)
        smoothed = centerline_xyz.copy()
        smoothed[:, 0] = x_smooth
        smoothed[:, 1] = y_smooth
        smoothed[:, 2] = z_smooth
        return smoothed


    @staticmethod
    def visualizeWaysWithLidar(dataset, osm_handler, cores=48):
        pcd_points_list = dataset.loadLAZs(dataset.getAllTiles())
        pcd_points = open3d.geometry.PointCloud()
        for pcd in pcd_points_list:
            pcd_points += pcd
        pcd_points.paint_uniform_color([0.3, 0.3, 0.3])

        ways_ribbon_arguments = []
        for wid, way_nodes in tqdm(osm_handler.ways.items()):
            way_nodes = osm_handler.getWayNodes(wid)
            way_coordinates = osm_handler.getWayCoordinates(
                wid, project_to_meters=True
            )
            way_bounding_box_meters = osm_handler.getWayBoundingBox(
                wid, project_to_meters=True
            )
            lane_count = osm_handler.ways[wid]["lane_count"]
            lane_width = osm_handler.ways[wid]["lane_width"]
            bridge = osm_handler.ways[wid]["bridge"]
            arguments = [
                way_coordinates,
                way_bounding_box_meters,
                lane_count,
                lane_width,
                bridge,
            ]
            ways_ribbon_arguments.append(arguments)

        ways_ribbons = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=cores)(
                    joblib.delayed(OSMLidarCorrection.convertWaysToWaysRibbonsElevation)(
                        ways_ribbon_arguments[i][0],
                        ways_ribbon_arguments[i][1],
                        ways_ribbon_arguments[i][2],
                        ways_ribbon_arguments[i][3],
                        ways_ribbon_arguments[i][4],
                    )
                    for i in range(len(ways_ribbon_arguments))
                ),
                total=len(ways_ribbon_arguments),
            )
        )

        ways_lidar_points = open3d.geometry.PointCloud()
        for i in range(len(ways_ribbons)):
            new_point_cloud = open3d.geometry.PointCloud()
            new_point_cloud.points = open3d.utility.Vector3dVector(ways_ribbons[i])
            ways_lidar_points = ways_lidar_points + new_point_cloud
        ways_lidar_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
        open3d.visualization.draw_geometries([pcd_points, ways_lidar_points])


    @staticmethod
    def getNearestNonBridgeHeight(way_coordinates, bridge):
        current_height = None
        for i in range(len(bridge)):
            if bridge[i] and (current_height is not None):
                way_coordinates[i][2] = current_height
            elif not bridge[i]:
                current_height = way_coordinates[i][2]
        current_height = None
        for i in range(len(bridge) - 1, -1, -1):
            if bridge[i] and (current_height is not None):
                way_coordinates[i][2] = current_height
            elif not bridge[i]:
                current_height = way_coordinates[i][2]
        return way_coordinates

    @staticmethod
    def computeLidarHeight(dataset, pcd_points, way_coordinates, bridge):
        for i in range(len(bridge)):
            if bridge[i]:
                lidar_median_height = dataset.lidarMedianHeightAtXYMeters(
                    pcd_points, [way_coordinates[i][0], way_coordinates[i][1]]
                )
                if lidar_median_height is not None:
                    way_coordinates[i][2] = lidar_median_height
        return way_coordinates

    @staticmethod
    def convertWaysToWaysRibbons(
        dataset,
        pcd_points,
        way_coordinates,
        way_bounding_box_meters,
        lane_count,
        oneway,
        lanes_backward,
        lanes_forward,
        lane_width,
        bridge,
    ):
        lidar_spacing = 0.21

        dem_data = dataset.loadDEMsFromBoundingBoxMeters(way_bounding_box_meters)
        # Create OSM cloud points
        for i in range(len(way_coordinates)):
            way_current_height = dataset.minHeightAtXYMeters(
                dem_data, [way_coordinates[i][0], way_coordinates[i][1]]
            )
            way_coordinates[i][2] = way_current_height
        # Correcting for bridges
        # way_coordinates = getNearestNonBridgeHeight(way_coordinates, bridge)
        way_coordinates = OSMLidarCorrection.computeLidarHeight(
            dataset, pcd_points, way_coordinates, bridge
        )
        # print("Expanding.....")
        expanded = OSMLidarCorrection.ExpandCenterlineToDenseRibbon(
            way_coordinates,
            lanes=lane_count,
            oneway=oneway,
            lanes_backward=lanes_backward,
            lanes_forward=lanes_forward,
            lane_width=lane_width,
            width_resolution=lidar_spacing,
            length_resolution=lidar_spacing,
        )
        if len(expanded) == 0:
            return expanded, way_coordinates
        smoothed = OSMLidarCorrection.SmoothSavgol(expanded)
        return smoothed, way_coordinates

    @staticmethod
    def convertWaysToWaysRibbonsElevation(
        way_coordinates,
        way_bounding_box_meters,
        lane_count,
        lane_width,
        bridge,
    ):
        lidar_spacing = 0.21

        # print("Expanding.....")
        expanded = OSMLidarCorrection.ExpandCenterlineToDenseRibbon(
            way_coordinates,
            lanes=lane_count,
            lane_width=lane_width,
            width_resolution=lidar_spacing,
            length_resolution=lidar_spacing,
        )
        smoothed = OSMLidarCorrection.SmoothSavgol(expanded)
        return smoothed

    @staticmethod
    def conformToDEMs(
        dataset, way_coordinates, way_bounding_box_meters, lane_height=0.25
    ):
        dem_data = dataset.loadDEMsFromBoundingBoxMeters(way_bounding_box_meters)

        for i in range(len(way_coordinates)):
            way_current_height = dataset.minHeightAtXYMeters(
                dem_data, [way_coordinates[i][0], way_coordinates[i][1]]
            )
            way_coordinates[i][2] = min(
                way_coordinates[i][2], way_current_height + lane_height
            )

        return way_coordinates

    @staticmethod
    def processLidarCorrection(
        child_seed,
        dataset,
        wid,
        way_coordinates,
        way_bounding_box_meters,
        lane_count,
        oneway,
        lanes_backward,
        lanes_forward,
        lane_width,
        bridge,
        visualize=[]
        #visualize=["108162374", "19447013", "970086456"]
    ):
        rng = np.random.default_rng(child_seed)
        # rng_generated = rng.integers(0, 1000)
        rng_generated = 1

        pcd_points = dataset.loadLAZsFromBoundingBoxMeters(way_bounding_box_meters)
        if len(pcd_points.points) == 0:
            print("No point cloud, returning identity", way_bounding_box_meters)
            sys.stdout.flush()
            return (np.eye(4), 0.0, way_coordinates, way_coordinates)

        smoothed, way_coordinates = OSMLidarCorrection.convertWaysToWaysRibbons(
            dataset,
            pcd_points,
            way_coordinates,
            way_bounding_box_meters,
            lane_count,
            oneway,
            lanes_backward,
            lanes_forward,
            lane_width,
            bridge,
        )
        if len(smoothed) == 0:
            print("Way not long enough, returning identity")
            sys.stdout.flush()
            return (np.eye(4), 0.0, way_coordinates, way_coordinates)

        osm_process_points = open3d.geometry.PointCloud()
        osm_process_points.points = open3d.utility.Vector3dVector(smoothed)
        thresholds = [0.35]
        fitness_desire = 0.99
        transforms, fitnesses, mse = [], [], []
        # print(wid, " ready!")
        for i in thresholds:
            reg_p2p = open3d.pipelines.registration.registration_icp(
                osm_process_points,
                pcd_points,
                i,
                np.eye(4),  # Initial guess
                open3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=open3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-5, relative_rmse=1e-5, max_iteration=5000
                ),
            )
            transform = reg_p2p.transformation
            fitness = reg_p2p.fitness
            rmse = reg_p2p.inlier_rmse
            if reg_p2p.fitness > fitness_desire:
                transforms.append(transform)
                fitnesses.append(fitness)
                mse.append(rmse)
            print(wid, transform, fitness)
            sys.stdout.flush()
            if wid in visualize:
                pcd_points.paint_uniform_color([0.1, 0.1, 0.8]) # Blueish ribbon
                osm_process_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
                osm_adjusted_full_points = open3d.geometry.PointCloud(osm_process_points)
                osm_adjusted_full_points.transform(transform)
                osm_adjusted_full_points.paint_uniform_color([0.1, 0.8, 0.1]) # Greenish ribbon
                open3d.visualization.draw_geometries([pcd_points, osm_process_points, osm_adjusted_full_points], window_name=f"WID: {wid}")
        if len(mse) > 0:
            best_index = np.argmin(mse)
            adjusted_osm_points = open3d.geometry.PointCloud()
            adjusted_osm_points.points = open3d.utility.Vector3dVector(way_coordinates)
            adjusted_osm_points.transform(transforms[best_index])
            osm_corrected_points = OSMLidarCorrection.conformToDEMs(
                dataset, np.asarray(adjusted_osm_points.points), way_bounding_box_meters
            )
            return (
                transforms[best_index],
                fitnesses[best_index],
                way_coordinates,
                osm_corrected_points,
            )
        return (np.eye(4), 0.0, way_coordinates, way_coordinates)

    @staticmethod
    def correctWaysWithLidar(dataset, osm_handler, cores=48):
        way_correction_arguments = []
        for wid, way_nodes in tqdm(osm_handler.ways.items()):
            way_nodes = osm_handler.getWayNodes(wid)
            way_coordinates = osm_handler.getWayCoordinates(
                wid, project_to_meters=True
            )
            way_bounding_box_meters = osm_handler.getWayBoundingBox(
                wid, project_to_meters=True
            )
            lane_count = osm_handler.ways[wid]["lane_count"]
            oneway = osm_handler.ways[wid]["oneway"]
            lanes_backward = osm_handler.ways[wid]["lanes_backward"]
            lanes_forward = osm_handler.ways[wid]["lanes_forward"]
            lane_width = osm_handler.ways[wid]["lane_width"]
            bridge = osm_handler.ways[wid]["bridge"]
            arguments = [
                wid,
                way_coordinates,
                way_bounding_box_meters,
                lane_count,
                oneway,
                lanes_backward,
                lanes_forward,
                lane_width,
                bridge
            ]
            # print(arguments)
            way_correction_arguments.append(arguments)

        ss = np.random.SeedSequence(2024)
        child_seeds = ss.spawn(len(way_correction_arguments))

        way_correction_results = list(tqdm(
                joblib.Parallel(return_as="generator", n_jobs=cores)(
                    joblib.delayed(OSMLidarCorrection.processLidarCorrection)(
                        child_seed,
                        dataset,
                        way_correction_arguments[i][0],
                        way_correction_arguments[i][1],
                        way_correction_arguments[i][2],
                        way_correction_arguments[i][3],
                        way_correction_arguments[i][4],
                        way_correction_arguments[i][5],
                        way_correction_arguments[i][6],
                        way_correction_arguments[i][7],
                        way_correction_arguments[i][8],
                    )
                    for child_seed, i in zip(
                        child_seeds, range(len(way_correction_arguments))
                    )
                ),
                total=len(way_correction_arguments),
        )
        )

        for i in range(len(way_correction_results)):
            wid = way_correction_arguments[i][0]
            way_coordinates = way_correction_arguments[i][1]
            original_transformation_matrix = way_correction_results[i][0]
            original_transformation_matrix_fitness = way_correction_results[i][1]
            way_coordinates_with_elevation = way_correction_results[i][2]
            osm_corrected_points = way_correction_results[i][3]
            print(wid, way_coordinates, original_transformation_matrix, original_transformation_matrix_fitness, way_coordinates_with_elevation)
            osm_handler.annotateWayWithCorrectedPoints(
                wid, osm_corrected_points
            )
        print("Correcting points.....")
        full_dem = dataset.loadDEMs(dataset.getAllTiles())
        osm_handler.correctNodePoints(full_dem)

    @staticmethod
    def loadOSM(dataset):
        min_long, min_lat, max_long, max_lat = (
            dataset.metadata_json["bounds"]["min_long"],
            dataset.metadata_json["bounds"]["min_lat"],
            dataset.metadata_json["bounds"]["max_long"],
            dataset.metadata_json["bounds"]["max_lat"],
        )
        osm_handler = WayNodeCollectorLidarCorrection(min_long, min_lat)
        osm_file_path = dataset.getOSMPath()
        osm_handler.apply_file(osm_file_path)
        return osm_handler

    def performOSMLidarTuning(original_dataset_path, dataset_path, dataset_selection):
        dataset = None
        if dataset_selection == "tdot":
            dataset = TDOTSubset(dataset_path, recenter_global_origin_meters=True)
        else:
            raise Exception("Invalid dataset selection!")
        osm_handler = OSMLidarCorrection.loadOSM(dataset)
        osm_handler.generateImplicitWays()

        print("Lidar correction....")
        OSMLidarCorrection.correctWaysWithLidar(dataset, osm_handler, cores=40)

        print("Writing new OSM")
        osm_handler.createCorrectedOSMFile(
            dataset.getOSMPath(),
            dataset.getCorrectedOSMPath()
        )


if __name__ == "__main__":
    OSMLidarCorrection.performOSMLidarTuning("/home/richarwa/Documents/openstreetmap/TDOT_Davidson/", "SubsetSelection/", "tdot")