import osmium
import open3d
import pyproj
import numpy as np
import xml.etree.ElementTree as ET
import joblib
import time
from scipy.signal import savgol_filter
from copy import deepcopy
from DEM_python import DEM

meters_to_feet = 3.28084
feet_to_meters = 1.0/meters_to_feet

def MinHeightAtXY(dem, xy_coord, radius=1.0):
    # Extract X, Y, Z
    x, y = xy_coord
    return dem.altitude(x * meters_to_feet, y * meters_to_feet) * feet_to_meters
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

def ExpandCenterlineToDenseRibbon(centerline_xyz, lanes=1, lane_width=3.65, width_resolution=0.21, length_resolution=0.21):
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
        n_width = max(2, int((lane_width * lanes) / width_resolution))

        for j in range(n_length + 1):
            alpha = j / n_length
            point_center = (1 - alpha) * p0 + alpha * p1
            z = point_center[2]

            for k in range(-n_width // 2, n_width // 2 + 1):
                offset = (k * width_resolution) * perp
                x, y = point_center[:2] + offset
                points.append([x, y, z])

    return np.array(points)

def SmoothSavgol(centerline_xyz, window_length=21, polyorder=2):
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

def processWayToLidar(wid, lane_count, lane_width, way_coordinates, lidar_spacing=0.21):
    print("Reading DEM")
    dem_data = DEM("SubsetSelection/dem_subset.csv", 2.0, -999999)
    way_x_mean, way_y_mean, way_z_mean = np.mean(way_coordinates, axis=0)
    for i in range(len(way_coordinates)):
        way_current_height = MinHeightAtXY(dem_data, [way_coordinates[i][0], way_coordinates[i][1]])
        way_coordinates[i][2] = way_current_height
    print("Expanding.....")
    expanded = ExpandCenterlineToDenseRibbon(way_coordinates, lanes=lane_count, lane_width=lane_width, width_resolution=lidar_spacing, length_resolution=lidar_spacing)
    smoothed = SmoothSavgol(expanded)
    print(wid)
    if (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838"):
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(expanded)
        adjusted_osm_points.paint_uniform_color([0.1, 0.8, 0.1])  # greenish ribbon
        open3d.visualization.draw_geometries([adjusted_osm_points])
    del expanded
    return smoothed 

def processLidarCorrection(wid, osm_points):
    pcd_points = open3d.io.read_point_cloud("SubsetSelection/lidar_subset_1m.pcd")
    osm_process_points = open3d.geometry.PointCloud()
    print(wid, " Point clouds created!")
    #pcd_process_points.points = open3d.utility.Vector3dVector(pcd_points_numpy)
    print(wid, " Point clouds points loaded!")
    osm_process_points.points = open3d.utility.Vector3dVector(osm_points.copy())
    print(wid, " OSM Point clouds points loaded!")
    #pcd_process_points.normals = open3d.utility.Vector3dVector(pcd_normals_numpy)
    print(wid, " Point clouds normals loaded!")
    max_threshold = 5
    fitness_desire = 0.8
    transforms, fitnesses = [], []
    print(wid, " ready!")
    for i in range(max_threshold):
        current_i = float(i + 1)
        threshold = current_i  # distance threshold
        reg_p2p = open3d.pipelines.registration.registration_icp(
            osm_process_points, pcd_points, current_i,
            np.eye(4),  # Initial guess
            open3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        print(wid, " corrected!")
        transform = reg_p2p.transformation
        transforms.append(transform)
        fitnesses.append(reg_p2p.fitness)
        if (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838"):
            print(wid, current_i, transform, reg_p2p.fitness,  reg_p2p.inlier_rmse, getattr(reg_p2p, 'converged', 'unknown'))
        if (reg_p2p.fitness > fitness_desire):
            if (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838"):
                print("ACCEPTING MATCH: ", wid, current_i, transform, reg_p2p.fitness,  reg_p2p.inlier_rmse, getattr(reg_p2p, 'converged', 'unknown'))
            return (transform, reg_p2p.fitness)
        elif (current_i == (max_threshold - 1)):
            if (str(wid) == "19447013") or (str(wid) == "108162374") or (str(wid) == "19468810") or (str(wid) == "611434838"):
                print("Defaulting here: ", wid, current_i, transform, reg_p2p.fitness,  reg_p2p.inlier_rmse, getattr(reg_p2p, 'converged', 'unknown'))
            return transforms[0], fitnesses[0]

def convertWaysToLidar(handler):
    way_lidar_results_info = []
    way_corrections = []
    way_correction_arguments = []
    for wid, way_nodes in handler.ways.items():
        way_nodes = handler.getWayNodes(wid)
        way_coordinates = handler.getWayCoordinates(wid, project_to_meters=True)
        lane_count = handler.ways[wid]["lane_count"]
        lane_width = handler.ways[wid]["lane_width"]
        way_lidar_results_info.append([wid, lane_count, lane_width, way_coordinates])

    way_lidar_results = joblib.Parallel(n_jobs=16)(
        joblib.delayed(processWayToLidar)(info[0], info[1], info[2], info[3]) for info in way_lidar_results_info
    )

    print("Ways processed!")

    for i in range(len(way_lidar_results)):
        wid, lane_count, lane_width, way_coordinates = way_lidar_results_info[i]
        way_lidar_points = way_lidar_results[i]
        way_corrections.append({"wid": wid, "way_coordinates": way_coordinates, "lane_count": lane_count, "lane_width": lane_width})
        way_correction_arguments.append([way_lidar_points])

    return way_corrections, way_correction_arguments

def correctWaysWithLidar(handler, way_corrections, way_correction_arguments):
    #pcd_points = open3d.io.read_point_cloud("SubsetSelection/lidar_subset_1m.pcd")
    pcd_points = open3d.io.read_point_cloud("1481110.pcd")
    pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamKNN(knn=30))

    correction_results = joblib.Parallel(n_jobs=16)(
        joblib.delayed(processLidarCorrection)(way_corrections[i]["wid"], way_correction_arguments[i][0]) for i in range(len(way_correction_arguments))
    )
    #correction_results = [processLidarCorrection(pcd_points, way_corrections[i]["wid"], way_correction_arguments[i][0]) for i in range(len(way_correction_arguments))]

    for i in range(len(way_corrections)):
        wid = way_corrections[i]["wid"]
        way_coordinates = way_corrections[i]["way_coordinates"]
        adjusted_osm_points = open3d.geometry.PointCloud()
        adjusted_osm_points.points = open3d.utility.Vector3dVector(way_coordinates)

        original_transformation_matrix = correction_results[i][0]
        original_transformation_matrix_fitness = correction_results[i][1]
        default_transformation_matrix = np.eye(4)
        if (original_transformation_matrix_fitness > 0.5):
            adjusted_osm_points.transform(original_transformation_matrix)
        else:
            adjusted_osm_points.transform(default_transformation_matrix)
        osm_corrected_points = np.asarray(adjusted_osm_points.points)
        handler.annotateWayWithCorrectedPoints(wid, osm_corrected_points)
    print("Correcting points.....")
    handler.correctNodePoints()

def createCorrectedOSMFile(handler, original_osm_file, target_osm_file):
    # Parse XML using ElementTree to directly update lat/lon
    tree = ET.parse(original_osm_file)
    root = tree.getroot()

    for elem in root.findall("node"):
        nid = int(elem.attrib['id'])
        if nid in handler.nodes:
            lon, lat = handler.nodes[nid]["corrected_coordinates"][0], handler.nodes[nid]["corrected_coordinates"][1]
            elem.set('lat', f"{lat:.8f}")
            elem.set('lon', f"{lon:.8f}")

    # Output file
    tree.write(target_osm_file, encoding="utf-8", xml_declaration=True)


class WayNodeCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.nodes = {}  # id → (lat, lon)
        self.ways = {}
        self.proj = pyproj.Proj(proj="utm", zone=16, ellps="WGS84")

    def node(self, n):
        coordinates = [n.location.lon, n.location.lat, 0]
        meters_coordinates = self.projectToMeters(coordinates)
        #coordinates[2] = AverageHeightAtXY(coordinates[:2])
        #coordinates[2] = dem_data.altitude(meters_coordinates[0] * meters_to_feet, meters_coordinates[1] * meters_to_feet) * feet_to_meters * altitude_importance_factor
        self.nodes[n.id] = {"coordinates": coordinates, "total_ways": [], "corrected_coordinates": np.array([0, 0, 0])}
        
    def way(self, w):
        if 'highway' in w.tags:
            self.ways[w.id] = {"nodes": [], "corrected_node_positions": [], "lane_count": 0, "lane_width": 3.65}
            for n in w.nodes:
                self.ways[w.id]["nodes"].append(n.ref)
                self.ways[w.id]["corrected_node_positions"].append(np.array([0, 0, 0])) # Default of zero coordinate
                self.nodes[n.ref]["total_ways"].append(w.id)
            lanes = w.tags.get('lanes')
            if lanes is not None:
                try:
                    lane_count = int(lanes)
                    self.ways[w.id]["lane_count"] = lane_count
                except ValueError:
                    self.ways[w.id]["lane_count"] = 1
            else:
                self.ways[w.id]["lane_count"] = 1

    def getWayNodes(self, wid):
        return self.ways[wid]["nodes"]

    def projectToMeters(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x, y = self.proj(x,y)
        x = x - osm_origin_x
        y = y - osm_origin_y
        return np.array([x, y, z])
    
    def projectToOSM(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        #print("Starting projection ", x, y, z, osm_origin_x, osm_origin_y)
        x = x + osm_origin_x
        y = y + osm_origin_y
        x, y = self.proj(x, y, inverse=True)
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

    def annotateWayWithCorrectedPoints(self, wid, way_coordinates):
        for i in range(len(way_coordinates)):
            self.ways[wid]["corrected_node_positions"][i] = way_coordinates[i]

    def correctNodePoints(self):
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
                #node["corrected_coordinates"] = self.projectToOSM(self.projectToMeters(node["coordinates"]))

if __name__ == "__main__":

    # minlat="36.04929259" minlon="-86.71072242" maxlat="36.08809402" maxlon="-86.64948975"
    minlon=-86.69027
    minlat=36.06589 
    maxlon=-86.68008
    maxlat=36.07144
    #minlon=-86.71072242
    #minlat=36.04929259
    #maxlon=-86.64948975
    #maxlat=36.08809402
    
    handler = WayNodeCollector()
    osm_origin_x, osm_origin_y = handler.proj(minlon, minlat)

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
    osm_file = "148110_full_corrected_2022.osm"
    target_osm_file = "148110_full_corrected_2022_lidar.osm"
    handler.apply_file(osm_file)

    print("Lidar correction....")
    way_corrections, way_correction_arguments = convertWaysToLidar(handler)
    correctWaysWithLidar(handler, way_corrections, way_correction_arguments)

    print("Writing new OSM")
    createCorrectedOSMFile(handler, osm_file, target_osm_file)