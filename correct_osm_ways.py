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
from scipy.signal import savgol_filter
from copy import deepcopy
from tqdm import tqdm
from DEM_python import DEM

meters_to_feet = 3.28084
feet_to_meters = 1.0/meters_to_feet

def MinHeightAtXY(dem, xy_coord, radius=0.0):
    # Extract X, Y, Z
    x, y = xy_coord
    heights = []
    if (radius > 0):
        for x_current, y_current in zip(np.arange(x - radius, x + radius + 1.0, 1.0), np.arange(y - radius, y + radius + 1.0, 1.0)):
            heights.append(dem.altitude((x_current * meters_to_feet) + dem.original_x_bottom_left,  (y_current * meters_to_feet) + dem.original_y_bottom_left) * feet_to_meters)
        return min(heights)
    return dem.altitude((x * meters_to_feet) + dem.original_x_bottom_left,  (y * meters_to_feet) + dem.original_y_bottom_left) * feet_to_meters
    #return float(np.min(nearby_z))

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

'''
def ExpandCenterlineToDenseRibbon(centerline_xyz, lanes=1, lane_width=3.65, shoulder_width=0.0, width_resolution=0.21, length_resolution=0.21):
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
        n_width = max(2, int(((lane_width * lanes) + (2 * shoulder_width)) / width_resolution))

        for j in range(n_length + 1):
            alpha = j / n_length
            point_center = (1 - alpha) * p0 + alpha * p1
            z = point_center[2]

            for k in range(-n_width // 2, n_width // 2 + 1):
                offset = (k * width_resolution) * perp
                x, y = point_center[:2] + offset
                points.append([x, y, z])

    return np.array(points)
'''

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

def processWayToLidar(child_seed, dem_data, pcd_points_points, pcd_points_normals, wid, lane_count, lane_width, way_coordinates, lidar_spacing=0.21):
    pcd_points = open3d.geometry.PointCloud()
    pcd_points.points = open3d.utility.Vector3dVector(pcd_points_points.copy())
    way_x_mean, way_y_mean, way_z_mean = np.mean(way_coordinates, axis=0)
    for i in range(len(way_coordinates)):
        way_current_height = MinHeightAtXY(dem_data, [way_coordinates[i][0], way_coordinates[i][1]])
        way_coordinates[i][2] = way_current_height
        #if (str(wid) == "19447013"):
        #    MinHeightAtXY(dem_data, [way_coordinates[i][0], way_coordinates[i][1]], doprint=True)
        #    print(wid, i, [way_coordinates[i][0], way_coordinates[i][1]], way_current_height, way_bounding_box_feet, dem_data.height, dem_data.width, dem_data.x_origin, dem_data.y_origin, dem_data.x_origin - dem_data.original_x_bottom_left, dem_data.y_origin - dem_data.original_y_bottom_left)
    #print("Expanding.....")
    expanded = ExpandCenterlineToDenseRibbon(way_coordinates, lanes=lane_count, lane_width=lane_width, width_resolution=lidar_spacing, length_resolution=lidar_spacing)
    smoothed = SmoothSavgol(expanded)
    #print(wid)
    rng = np.random.default_rng(child_seed)
    #if (str(wid) == "19477386") or (str(wid) == "27925532") or (str(wid) == "108162489") or (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838") or (str(wid) == "19442996") or (str(wid) == "108162489") or (str(wid) == "975552234"):
    if (rng.integers(0,500) == 0):
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(expanded)
        adjusted_osm_points.paint_uniform_color([0.8, 0.1, 0.1])  # greenish ribbon
        #print("PCD ", pcd_points.get_min_bound(), pcd_points.get_max_bound())
        #print("OSM ", adjusted_osm_points.get_min_bound(), adjusted_osm_points.get_max_bound())
        open3d.visualization.draw_geometries([pcd_points, adjusted_osm_points])
    

    del expanded
    return smoothed 

def processLidarCorrection(child_seed, wid, osm_points, pcd_points_points, pcd_points_normals):
    if (len(pcd_points_points) == 0):
        print("No point cloud, returning identity")
        return (np.eye(4), 0.0)
    pcd_points = open3d.geometry.PointCloud()
    pcd_points.points = open3d.utility.Vector3dVector(pcd_points_points.copy())
    pcd_points.normals = open3d.utility.Vector3dVector(pcd_points_normals.copy())
    osm_process_points = open3d.geometry.PointCloud()
    osm_process_points.points = open3d.utility.Vector3dVector(osm_points.copy())
    thresholds = [0.35]
    fitness_desire = 0.99
    transforms, fitnesses, mse = [], [], []
    #print(wid, " ready!")
    rng = np.random.default_rng(child_seed)
    for i in thresholds:
        reg_p2p = open3d.pipelines.registration.registration_icp(
            osm_process_points, pcd_points, i,
            np.eye(4),  # Initial guess
            open3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-10, relative_rmse=1e-10, max_iteration=200)
        )
        #print(wid, " corrected!")
        transform = reg_p2p.transformation
        fitness = reg_p2p.fitness
        rmse = reg_p2p.inlier_rmse
        
        #if (str(wid) == "19477386") or (str(wid) == "27925532") or (str(wid) == "108162489") or (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838") or (str(wid) == "19442996") or (str(wid) == "108162489") or (str(wid) == "975552234"):
        if (rng.integers(0,500) == 0):
            print(wid, i, transform, fitness,  rmse, getattr(reg_p2p, 'converged', 'unknown'))
            adjusted_osm_points = open3d.geometry.PointCloud()
            adjusted_osm_points.points = open3d.utility.Vector3dVector(np.asarray(osm_process_points.points))
            adjusted_osm_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
            open3d.visualization.draw_geometries([pcd_points, adjusted_osm_points])
        if (reg_p2p.fitness > fitness_desire):
            transforms.append(transform)
            fitnesses.append(fitness)
            mse.append(rmse)
    if (len(mse) > 0):
        best_index = np.argmin(mse)
        return (transforms[best_index], fitnesses[best_index])
    return (np.eye(4), 0.0)

def ClipPCD(pcd, bottom_left, top_right, margins=0.0, downscale=True):
    bottom_left[0] -= margins
    bottom_left[1] -= margins
    top_right[0] += margins
    top_right[1] += margins
    aabb = open3d.geometry.AxisAlignedBoundingBox(bottom_left, top_right)
    cropped_pcd = pcd.crop(aabb)
    if downscale:
        cropped_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.5)
        cropped_pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30))
        cropped_pcd.normalize_normals()
    return cropped_pcd

def convertWaysToLidar(handler, dem_data, pcd_points):
    pcd_points_points = np.asarray(pcd_points.points)
    pcd_points_min_bound = pcd_points.get_min_bound()
    pcd_points_max_bound = pcd_points.get_max_bound()

    way_lidar_results_info = []
    way_corrections = []
    way_correction_arguments = []
    print(len(handler.ways.items()))
    for wid, way_nodes in tqdm(handler.ways.items()):
        way_nodes = handler.getWayNodes(wid)
        way_coordinates = handler.getWayCoordinates(wid, project_to_meters=True)
        way_bounding_box_meters = handler.getWayBoundingBox(wid, project_to_meters=True)
        way_bounding_box_meters = [[way_bounding_box_meters[0][0], way_bounding_box_meters[0][1], pcd_points_min_bound[2]], [way_bounding_box_meters[1][0], way_bounding_box_meters[1][1], pcd_points_max_bound[2]]]
        way_bounding_box_feet = (np.array(way_bounding_box_meters) * meters_to_feet).tolist()
        dem_x_origin = dem_data.x_origin * feet_to_meters
        dem_y_origin = dem_data.y_origin * feet_to_meters
        #print("BOX ", way_bounding_box_feet[0][0], way_bounding_box_feet[0][1], way_bounding_box_feet[1][0], way_bounding_box_feet[1][1])
        #print("DEM ORIGIN ", dem_x_origin, dem_y_origin, pcd_points_min_bound)
        pcd_origin_x = pcd_points_min_bound[0]
        pcd_origin_y = pcd_points_min_bound[1]
        way_bounding_box_feet[0][0] += ((dem_x_origin - pcd_origin_x) * meters_to_feet)
        way_bounding_box_feet[0][1] += ((dem_y_origin - pcd_origin_y) * meters_to_feet)
        way_bounding_box_feet[1][0] += ((dem_x_origin - pcd_origin_x) * meters_to_feet)
        way_bounding_box_feet[1][1] += ((dem_y_origin - pcd_origin_y) * meters_to_feet)
        lane_count = handler.ways[wid]["lane_count"]
        lane_width = handler.ways[wid]["lane_width"]
        margins = (np.max(lane_count) * np.max(lane_width)) + 3.0
        clipped_dem_data = DEM.clip_dem(dem_data, way_bounding_box_feet[0], way_bounding_box_feet[1], margins=margins*meters_to_feet)
        clipped_pcd = ClipPCD(pcd_points, way_bounding_box_meters[0], way_bounding_box_meters[1], margins=margins)
        clipped_pcd_points = np.asarray(clipped_pcd.points)
        clipped_pcd_normals = np.asarray(clipped_pcd.normals)
        way_lidar_results_info.append([clipped_dem_data, clipped_pcd_points, clipped_pcd_normals, wid, lane_count, lane_width, way_coordinates])

    ss = np.random.SeedSequence(2024)
    child_seeds = ss.spawn(len(way_lidar_results_info))
    way_lidar_results = list(tqdm(joblib.Parallel(return_as="generator", n_jobs=32)(
        joblib.delayed(processWayToLidar)(child_seed, info[0], info[1], info[2], info[3], info[4], info[5], info[6]) for child_seed, info in zip(child_seeds, way_lidar_results_info)
    ), total=len(way_lidar_results_info)))

    print("Ways processed!")

    for i in range(len(way_lidar_results)):
        clipped_dem_data, clipped_pcd_points, clipped_pcd_normals, wid, lane_count, lane_width, way_coordinates = way_lidar_results_info[i]
        way_lidar_points = way_lidar_results[i]
        way_corrections.append({"clipped_dem_data": clipped_dem_data, "clipped_pcd_points": clipped_pcd_points, "clipped_pcd_normals": clipped_pcd_normals, "wid": wid, "way_coordinates": way_coordinates, "lane_count": lane_count, "lane_width": lane_width})
        way_correction_arguments.append([way_lidar_points, clipped_pcd_points, clipped_pcd_normals])

    return way_corrections, way_correction_arguments

def correctWaysWithLidar(handler, dem, way_corrections, way_correction_arguments):
    ss = np.random.SeedSequence(2024)
    child_seeds = ss.spawn(len(way_correction_arguments))

    correction_results = list(tqdm(joblib.Parallel(return_as="generator", n_jobs=32)(
        joblib.delayed(processLidarCorrection)(child_seed, way_corrections[i]["wid"], way_correction_arguments[i][0], way_correction_arguments[i][1], way_correction_arguments[i][2]) for child_seed, i in zip(child_seeds, range(len(way_correction_arguments)))
    ), total=len(way_correction_arguments)))
    #correction_results = [processLidarCorrection(pcd_points, way_corrections[i]["wid"], way_correction_arguments[i][0]) for i in range(len(way_correction_arguments))]

    for i in range(len(way_corrections)):
        wid = way_corrections[i]["wid"]
        way_coordinates = way_corrections[i]["way_coordinates"]
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(way_coordinates)

        original_transformation_matrix = correction_results[i][0]
        original_transformation_matrix_fitness = correction_results[i][1]
        adjusted_osm_points.transform(original_transformation_matrix)
        #default_transformation_matrix = np.eye(4)
        #if (original_transformation_matrix_fitness > 0.5):
        #    adjusted_osm_points.transform(original_transformation_matrix)
        #else:
        #    adjusted_osm_points.transform(default_transformation_matrix)
        osm_corrected_points = np.asarray(adjusted_osm_points.points)
        handler.annotateWayWithCorrectedPoints(wid, osm_corrected_points)
    print("Correcting points.....")
    handler.correctNodePoints(dem)

def visualizeWaysWithLidar(ways, ways_info, lidar):
    ways_lidar_points = open3d.geometry.PointCloud()
    for i in range(len(ways)):
        new_point_cloud = open3d.geometry.PointCloud()
        new_point_cloud.points = open3d.utility.Vector3dVector(ways_info[i][0])
        ways_lidar_points = ways_lidar_points + new_point_cloud
    ways_lidar_points.paint_uniform_color([0.8, 0.1, 0.1])  # redish ribbon
    open3d.visualization.draw_geometries([ways_lidar_points, lidar])

def preprocessInputLidar(lidar_path, process=True):
    pcd_points = open3d.io.read_point_cloud(lidar_path)
    if process:
        x, y = pcd_points.get_min_bound()[0], pcd_points.get_min_bound()[1]
        pcd_points.translate((-x, -y, 0))
        pcd_points_points = np.asarray(pcd_points.points)
        pcd_points_points *= feet_to_meters
        pcd_points.points = open3d.utility.Vector3dVector(pcd_points_points)
        pcd_points.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30))
        pcd_points.normalize_normals()
        pcd_points.paint_uniform_color([0.1, 0.1, 0.1])
    return pcd_points

def createCorrectedOSMFile(handler, original_osm_file, target_osm_file):
    # Parse XML using ElementTree to directly update lat/lon
    tree = ET.parse(original_osm_file)
    root = tree.getroot()

    for elem in root.findall("node"):
        nid = str(elem.attrib['id'])
        if nid in handler.nodes:
            lon, lat, height = handler.nodes[nid]["corrected_coordinates"][0], handler.nodes[nid]["corrected_coordinates"][1], handler.nodes[nid]["corrected_coordinates"][2]
            elem.set('lat', f"{lat:.8f}")
            elem.set('lon', f"{lon:.8f}")
            ele_tag = ET.SubElement(elem, "tag")
            ele_tag.set("k", "ele")
            ele_tag.set("v", f"{height:.8f}")

    # Output file
    tree.write(target_osm_file, encoding="utf-8", xml_declaration=True)

class WayNodeCollector(osmium.SimpleHandler):
    def __init__(self, minlon, minlat):
        super().__init__()
        self.nodes = {}  # id → (lat, lon)
        self.ways_original = {}
        self.ways = {}
        self.node_graph = nx.Graph()
        #self.proj = pyproj.Proj(proj="utm", zone=16, ellps="WGS84")
        self.proj = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:6576", always_xy=True)
        self.osm_origin_x, self.osm_origin_y = self.proj.transform(minlon, minlat)
        self.osm_origin_x *= feet_to_meters
        self.osm_origin_y *= feet_to_meters

    def node(self, n):
        n_id = str(n.id)
        coordinates = [n.location.lon, n.location.lat, 0]
        meters_coordinates = self.projectToMeters(coordinates)
        #coordinates[2] = AverageHeightAtXY(coordinates[:2])
        #coordinates[2] = dem_data.altitude(meters_coordinates[0] * meters_to_feet, meters_coordinates[1] * meters_to_feet) * feet_to_meters * altitude_importance_factor
        self.nodes[n_id] = {"coordinates": coordinates, "meters_coordinates": meters_coordinates, "total_ways": [], "corrected_coordinates": np.array([0, 0, 0]), "lane_widths": [], "lane_counts": []}
        
    def way(self, w):
        #if (len(w.nodes) < 3):
        ##    return
        w_id = str(w.id)
        if 'highway' in w.tags:
            lane_width_by_highway = {
                'motorway': 3.75,
                'motorway_link': 3.5,
                'primary': 3.5,
                'secondary': 3.0,
                'tertiary': 2.8,
                'residential': 2.7,
                'service': 2.5,
            }
            lane_type = w.tags['highway']
            lane_width = float(lane_width_by_highway[lane_type]) if lane_type in lane_width_by_highway else 3.65
            self.ways_original[w_id] = {"nodes": [], "corrected_node_positions": [], "lane_count": [], "lane_width": [lane_width for n in w.nodes]}
            nodes = []
            lanes = w.tags.get('lanes')
            if lanes is not None:
                try:
                    lane_count = int(lanes)
                    self.ways_original[w_id]["lane_count"] = [lane_count for n in w.nodes]
                except ValueError:
                    self.ways_original[w_id]["lane_count"] = [1 for n in w.nodes]
            else:
                self.ways_original[w_id]["lane_count"] = [1 for n in w.nodes]
            for n in w.nodes:
                n_ref = str(n.ref)
                self.ways_original[w_id]["nodes"].append(n_ref)
                self.ways_original[w_id]["corrected_node_positions"].append(np.array([0, 0, 0])) # Default of zero coordinate
                self.nodes[n_ref]["total_ways"].append(w_id)
                self.nodes[n_ref]["lane_counts"].append(self.ways_original[w_id]["lane_count"][0])
                self.nodes[n_ref]["lane_widths"].append(lane_width)
                nodes.append(n_ref)
            for node1, node2 in zip(nodes[:-1], nodes[1:]):
                self.node_graph.add_edge(node1, node2)
            self.ways[w_id] = self.ways_original[w_id].copy()
    
    def generateImplicitWays(self, node_count=15, distance_bound=125):
        all_segments = []

        def findPathsFromSource(node_graph, source, node_count):
            segments = []
            for target in node_graph.nodes:
                if (source == target):
                    continue
                for path in nx.all_simple_paths(node_graph, source=source, target=target, cutoff=node_count - 1):
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
        all_segments_lists = list(tqdm(joblib.Parallel(return_as="generator", n_jobs=48)(
            joblib.delayed(findPathsFromSource)(self.node_graph, source, node_count) for source in sources
        ), total=len(sources)))
        all_segments = [item for sublist in all_segments_lists for item in sublist]
        accepted_count = 0
        for segment in all_segments:
            if (getMinMaxMeterDistanceOfWay(segment) > distance_bound):
                continue
            accepted_count += 1
            segment_hashes = [hashlib.sha256(node_id.encode()).hexdigest() for node_id in segment]
            segment_way_id = hashlib.sha256("".join(segment_hashes).encode()).hexdigest()
            self.ways[segment_way_id] = {"nodes": [], "corrected_node_positions": [], "lane_count": [np.mean(self.nodes[node]["lane_counts"]) for node in segment], "lane_width": [np.mean(self.nodes[node]["lane_widths"]) for node in segment]}
            for node in segment:
                self.ways[segment_way_id]["nodes"].append(node)
                self.ways[segment_way_id]["corrected_node_positions"].append(np.array([0, 0, 0])) # Default of zero coordinate
                self.nodes[node]["total_ways"].append(segment_way_id)

        print(f"Found {accepted_count} unique segments of {node_count} nodes.")

    def getWayNodes(self, wid):
        return self.ways[wid]["nodes"]

    def projectToMeters(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x, y = self.proj.transform(x,y)
        x = (x * feet_to_meters) - self.osm_origin_x
        y = (y * feet_to_meters) - self.osm_origin_y
        return np.array([x, y, z])
    
    def projectToOSM(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        #print("Starting projection ", x, y, z, osm_origin_x, osm_origin_y)
        x = (x + self.osm_origin_x) * meters_to_feet
        y = (y + self.osm_origin_y) * meters_to_feet
        #x, y = self.proj(x, y, inverse=True)
        x, y = self.proj.transform(x, y, direction="INVERSE")
        #print("Final projection ", x, y, z)
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
        way_coordinates = self.getWayCoordinates(wid, project_to_meters=project_to_meters)
        bottom_left = way_coordinates[0][:2].tolist()
        top_right = way_coordinates[0][:2].tolist()
        #print(len(way_coordinates))
        for i in range(1, len(way_coordinates)):
            x, y, z = way_coordinates[i]
            #print(bottom_left)
            #print(top_right)
            #print(x, y, z)
            if (bottom_left[0] > x):
                bottom_left[0] = x
            if (bottom_left[1] > y):
                bottom_left[1] = y
            if (top_right[0] < x):
                top_right[0] = x
            if (top_right[1] < y):
                top_right[1] = y
        #print(way_coordinates)
        #print(bottom_left)
        #print(top_right)
        return [bottom_left, top_right]

    def annotateWayWithCorrectedPoints(self, wid, way_coordinates):
        for i in range(len(way_coordinates)):
            self.ways[wid]["corrected_node_positions"][i] = way_coordinates[i]

    def correctNodePoints(self, dem):
        for wid in self.ways:
            way = self.ways[wid]
            for i in range(len(way["nodes"])):
                nid = way["nodes"][i]
                corrected_nid = way["corrected_node_positions"][i]
                self.nodes[nid]["corrected_coordinates"] = self.nodes[nid]["corrected_coordinates"] + corrected_nid
                #print(corrected_nid, len(self.nodes[nid]["corrected_coordinates"]))
        for nid in self.nodes:
            node = self.nodes[nid]
            total_ways = len(node["total_ways"])
            #total_ways = 0
            if (total_ways > 0):
                node["corrected_coordinates"] = self.projectToOSM(node["corrected_coordinates"] / total_ways)
                #print("Corrected: ", self.nodes[nid]["corrected_coordinates"])
            else:
                node["corrected_coordinates"] = node["coordinates"]
            corrected_coordinates_meters = self.projectToMeters(node["corrected_coordinates"])
            node["corrected_coordinates"][2] = MinHeightAtXY(dem, (corrected_coordinates_meters[0], corrected_coordinates_meters[1]))
                #node["corrected_coordinates"] = self.projectToOSM(self.projectToMeters(node["coordinates"]))

if __name__ == "__main__":

    # minlat="36.04929259" minlon="-86.71072242" maxlat="36.08809402" maxlon="-86.64948975"
    #minlon=-86.69027
    #minlat=36.06589 
    #maxlon=-86.68008
    #maxlat=36.07144
    #minlon=-86.71072242
    #minlat=36.04929259
    #maxlon=-86.64948975
    #maxlat=36.08809402

    #Medium size
    minlat=36.06033995
    minlon=-86.70032724
    maxlat=36.07699495
    maxlon=-86.67001764

    #148110 Single tile
    #minlat=36.0658924703269
    #minlon=-86.690225450505
    #maxlat=36.0714441392433
    #maxlon=-86.68001222532651
    
    handler = WayNodeCollector(minlon, minlat)

    print("Reading PCD")
    #pcd = open3d.io.read_point_cloud("SubsetSelection/lidar_subset.pcd")
    print("Postprocessing PCD")
    #pcd_points = np.asarray(pcd.points).astype(np.float32)
    #pcd_normals = np.asarray(pcd.normals).astype(np.float32)
    #pcd_points_shape = pcd_points.shape
    #pcd_normals_shape = pcd_normals.shape
    #pcd_points_shape = (449707604, 3)
    #pcd_normals_shape = (449707604, 3)

    # Load OSM
    print("Loading OSM")
    #osm_file = "148110_full_corrected_2022.osm"
    #target_osm_file = "148110_full_corrected_2022_lidar.osm"
    osm_file = "SubsetSelection/osm_subset.osm"
    target_osm_file = "SubsetSelection/osm_subset_corrected_lidar.osm"
    handler.apply_file(osm_file)
    print("Creating implicit paths")
    handler.generateImplicitWays()

    print("Reading DEM")
    dem_data = DEM.from_csv("SubsetSelection/dem_subset.csv", 2.0, -999999)
    #dem_data = DEM("148110.asc", 2.0, -999999)

    print("Loading lidar....")
    #lidar_path = "148110.pcd"
    lidar_path = "SubsetSelection/lidar_subset.pcd"
    pcd_points = preprocessInputLidar(lidar_path, False)

    print("Lidar correction....")
    way_corrections, way_correction_arguments = convertWaysToLidar(handler, dem_data, pcd_points)
    correctWaysWithLidar(handler, dem_data, way_corrections, way_correction_arguments)

    print("Writing new OSM")
    createCorrectedOSMFile(handler, osm_file, target_osm_file)