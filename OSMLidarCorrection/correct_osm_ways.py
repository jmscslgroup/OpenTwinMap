modules_path = "/home/richarwa/CarlaIngestion"
import sys
sys.path.append(modules_path)
import osmium
import open3d
import pyproj
import numpy as np
import math
import xml.etree.ElementTree as ET
import joblib
import time
import networkx as nx
import hashlib
from tdot_subset import TDOTSubset
from scipy.signal import savgol_filter
from copy import deepcopy
from tqdm import tqdm
from DEM_python import DEM

meters_to_feet = 3.28084
feet_to_meters = 1.0/meters_to_feet

def LidarGetAverageMeterSpacing(pcd):
    points = np.asarray(pcd.points)
    
    # Use built-in KDTree to estimate spacing
    pcd_tree = open3d.geometry.KDTreeFlann(pcd)
    
    nn_dists = []
    for i in range(len(points)):
        [_, idx, dist2] = pcd_tree.search_knn_vector_3d(points[i], 2)  # 2 nearest:  itself + 1 neighbor
        if len(dist2) > 1:
            nn_dists.append(np.sqrt(dist2[1]))  # exclude distance to self
    
    # Estimate average spacing
    avg_spacing = np.mean(nn_dists)
    print(f"Estimated average point spacing: {avg_spacing:.2f} meters")
    
    return avg_spacing

def UpsampleLine(points, target_spacing):
    """
    Upsample a polyline to match a desired point spacing.
    Input: points = (N, 2) or (N, 3) array
    Output: (M, 3) array with interpolated points
    """
    points = np.array(points)
    upsampled = []

    for i in range(len(points) - 1):
        p0, p1 = points[i], points[i+1]
        #print(p0, p1)
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

def DownsampleLine(points, target_number):
    indices = np.linspace(0, len(points) - 1, target_number, dtype=int)
    return points[indices]

def ExpandCenterlineToDenseRibbon(centerline_xyz, lanes, lane_width, shoulder_width=1.8, width_resolution=0.21, length_resolution=0.21):
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
        perp = np.array([-direction[1], direction[0]])

        # Number of samples
        n_length = max(2, int(length / length_resolution))
        n_width = max(2, int(((lane_width[i] * lanes[i]) + (2 * shoulder_width)) / width_resolution))

        for j in range(n_length + 1):
            alpha = j / n_length
            point_center = (1 - alpha) * p0 + alpha * p1
            z = point_center[2]

            for k in range(-n_width // 2, n_width // 2 + 1):
                offset = (k * width_resolution) * perp
                x, y = point_center[:2] + offset
                points.append([x, y, z])

    return np.array(points)

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

def visualizeWaysWithLidar(tdot_subset, cores=48):
    pcd_points_list = tdot_subset.loadLAZs(tdot_subset.getAllTiles())
    pcd_points = open3d.geometry.PointCloud()
    for pcd in pcd_points_list:
        pcd_points += pcd
    pcd_points.paint_uniform_color([0.3, 0.3, 0.3])

    ways_ribbon_arguments = []
    for wid, way_nodes in tqdm(tdot_subset.osm_handler.ways.items()):
        way_nodes = tdot_subset.osm_handler.getWayNodes(wid)
        way_coordinates = tdot_subset.osm_handler.getWayCoordinates(wid, project_to_meters=True)
        way_bounding_box_meters = tdot_subset.osm_handler.getWayBoundingBox(wid, project_to_meters=True)
        lane_count = tdot_subset.osm_handler.ways[wid]["lane_count"]
        lane_width = tdot_subset.osm_handler.ways[wid]["lane_width"]
        bridge = tdot_subset.osm_handler.ways[wid]["bridge"]
        arguments = [way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge]
        ways_ribbon_arguments.append(arguments)

    ways_ribbons = list(tqdm(joblib.Parallel(return_as="generator", n_jobs=cores)(
        joblib.delayed(convertWaysToWaysRibbonsElevation)(tdot_subset, ways_ribbon_arguments[i][0], ways_ribbon_arguments[i][1], ways_ribbon_arguments[i][2], ways_ribbon_arguments[i][3], ways_ribbon_arguments[i][4]) for i in range(len(ways_ribbon_arguments))
    ), total=len(ways_ribbon_arguments)))
    
    ways_lidar_points = open3d.geometry.PointCloud()
    for i in range(len(ways_ribbons)):
        new_point_cloud = open3d.geometry.PointCloud()
        new_point_cloud.points = open3d.utility.Vector3dVector(ways_ribbons[i])
        ways_lidar_points = ways_lidar_points + new_point_cloud
    ways_lidar_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
    open3d.visualization.draw_geometries([pcd_points, ways_lidar_points])

def getNearestNonBridgeHeight(tdot_subset, way_coordinates, bridge):
    current_height = None
    for i in range(len(bridge)):
        if bridge[i] and (current_height is not None):
            way_coordinates[i][2] = current_height
        elif (not bridge[i]):
            current_height = way_coordinates[i][2]
    current_height = None
    for i in range(len(bridge) - 1, -1, -1):
        if bridge[i] and (current_height is not None):
            way_coordinates[i][2] = current_height
        elif (not bridge[i]):
            current_height = way_coordinates[i][2]
    return way_coordinates

def computeLidarHeight(tdot_subset, pcd_points, way_coordinates, bridge):
    for i in range(len(bridge)):
        if bridge[i]:
            lidar_median_height = tdot_subset.lidarMedianHeightAtXYMeters(pcd_points, [way_coordinates[i][0], way_coordinates[i][1]])
            if lidar_median_height is not None:
                way_coordinates[i][2] = lidar_median_height
    return way_coordinates

def convertWaysToWaysRibbons(tdot_subset, pcd_points, way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge):
    lidar_spacing = 0.21
    
    dem_data = tdot_subset.loadDEMsFromBoundingBoxMeters(way_bounding_box_meters)

    # Create OSM cloud points
    for i in range(len(way_coordinates)):
        way_current_height = tdot_subset.minHeightAtXYMeters(dem_data, [way_coordinates[i][0], way_coordinates[i][1]])
        way_coordinates[i][2] = way_current_height
    # Correcting for bridges
    #way_coordinates = getNearestNonBridgeHeight(way_coordinates, bridge)
    way_coordinates = computeLidarHeight(tdot_subset, pcd_points, way_coordinates, bridge)
    #print("Expanding.....")
    expanded = ExpandCenterlineToDenseRibbon(way_coordinates, lanes=lane_count, lane_width=lane_width, width_resolution=lidar_spacing, length_resolution=lidar_spacing)
    if (len(expanded) == 0):
        return expanded, way_coordinates
    smoothed = SmoothSavgol(expanded)
    return smoothed, way_coordinates

def convertWaysToWaysRibbonsElevation(tdot_subset, way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge):
    lidar_spacing = 0.21
    
    #print("Expanding.....")
    expanded = ExpandCenterlineToDenseRibbon(way_coordinates, lanes=lane_count, lane_width=lane_width, width_resolution=lidar_spacing, length_resolution=lidar_spacing)
    smoothed = SmoothSavgol(expanded)
    return smoothed

def processLidarCorrection(child_seed, tdot_subset, wid, way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge):
    rng = np.random.default_rng(child_seed)
    #rng_generated = rng.integers(0, 1000)
    rng_generated = 1

    pcd_points = tdot_subset.loadLAZsFromBoundingBoxMeters(way_bounding_box_meters)
    if (len(pcd_points.points) == 0):
        print("No point cloud, returning identity")
        return (np.eye(4), 0.0, way_coordinates)

    smoothed, way_coordinates = convertWaysToWaysRibbons(tdot_subset, pcd_points, way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge)
    if (len(smoothed) == 0):
        print("Way not long enough, returning identity")
        return (np.eye(4), 0.0, way_coordinates)
    '''
    if (rng_generated == 0) or (str(wid) == "108162489") or (str(wid) == "635078708"):
        print(bridge)
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(smoothed)
        adjusted_osm_points.paint_uniform_color([0.8, 0.1, 0.1])  # greenish ribbon
        open3d.visualization.draw_geometries([pcd_points, adjusted_osm_points])
    '''

    osm_process_points = open3d.geometry.PointCloud()
    osm_process_points.points = open3d.utility.Vector3dVector(smoothed)
    thresholds = [0.35]
    fitness_desire = 0.99
    transforms, fitnesses, mse = [], [], []
    #print(wid, " ready!")
    for i in thresholds:
        reg_p2p = open3d.pipelines.registration.registration_icp(
            osm_process_points, pcd_points, i,
            np.eye(4),  # Initial guess
            open3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-10, relative_rmse=1e-10, max_iteration=200)
        )
        transform = reg_p2p.transformation
        fitness = reg_p2p.fitness
        rmse = reg_p2p.inlier_rmse
        
        '''
        if (rng_generated == 0) or (str(wid) == "108162489") or (str(wid) == "635078708"):
            print(wid, i, transform, fitness,  rmse, getattr(reg_p2p, 'converged', 'unknown'))
            adjusted_osm_points = open3d.geometry.PointCloud()
            adjusted_osm_points.points = open3d.utility.Vector3dVector(np.asarray(osm_process_points.points))
            adjusted_osm_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
            open3d.visualization.draw_geometries([pcd_points, adjusted_osm_points])
        '''
        if (reg_p2p.fitness > fitness_desire):
            transforms.append(transform)
            fitnesses.append(fitness)
            mse.append(rmse)
    if (len(mse) > 0):
        best_index = np.argmin(mse)
        return (transforms[best_index], fitnesses[best_index], way_coordinates)
    return (np.eye(4), 0.0, way_coordinates)

def correctWaysWithLidar(tdot_subset, cores=32):
    way_correction_arguments = []
    for wid, way_nodes in tqdm(tdot_subset.osm_handler.ways.items()):
        way_nodes = tdot_subset.osm_handler.getWayNodes(wid)
        way_coordinates = tdot_subset.osm_handler.getWayCoordinates(wid, project_to_meters=True)
        way_bounding_box_meters = tdot_subset.osm_handler.getWayBoundingBox(wid, project_to_meters=True)
        lane_count = tdot_subset.osm_handler.ways[wid]["lane_count"]
        lane_width = tdot_subset.osm_handler.ways[wid]["lane_width"]
        bridge = tdot_subset.osm_handler.ways[wid]["bridge"]
        arguments = [wid, way_coordinates, way_bounding_box_meters, lane_count, lane_width, bridge]
        #print(arguments)
        way_correction_arguments.append(arguments)

    ss = np.random.SeedSequence(2024)
    child_seeds = ss.spawn(len(way_correction_arguments))

    way_correction_results = list(tqdm(joblib.Parallel(return_as="generator", n_jobs=cores)(
        joblib.delayed(processLidarCorrection)(child_seed, tdot_subset, way_correction_arguments[i][0], way_correction_arguments[i][1], way_correction_arguments[i][2], way_correction_arguments[i][3], way_correction_arguments[i][4], way_correction_arguments[i][5]) for child_seed, i in zip(child_seeds, range(len(way_correction_arguments)))
    ), total=len(way_correction_arguments)))

    for i in range(len(way_correction_results)):
        wid = way_correction_arguments[i][0]
        way_coordinates = way_correction_arguments[i][1]
        original_transformation_matrix = way_correction_results[i][0]
        original_transformation_matrix_fitness = way_correction_results[i][1]
        way_coordinates_with_elevation = way_correction_results[i][2]
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(way_coordinates_with_elevation)
        adjusted_osm_points.transform(original_transformation_matrix)

        osm_corrected_points = np.asarray(adjusted_osm_points.points)
        tdot_subset.osm_handler.annotateWayWithCorrectedPoints(wid, osm_corrected_points)
    print("Correcting points.....")
    full_dem = tdot_subset.loadDEMs(tdot_subset.getAllTiles())
    tdot_subset.osm_handler.correctNodePoints(full_dem)

if __name__ == "__main__":
    tdot_subset = TDOTSubset("SubsetSelection/")
    tdot_subset.processOSM()
    tdot_subset.osm_handler.generateImplicitWays()

    print("Lidar correction....")
    correctWaysWithLidar(tdot_subset, cores=48)

    print("Writing new OSM")
    tdot_subset.osm_handler.createCorrectedOSMFile("SubsetSelection/osm_subset.osm", "SubsetSelection/osm_subset_corrected_all_lidar_for_elevation.osm")