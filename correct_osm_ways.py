import osmium
import open3d
import pyproj
import numpy as np
import xml.etree.ElementTree as ET
from copy import deepcopy
from DEM_python import DEM

meters_to_feet = 3.28084
feet_to_meters = 1.0/meters_to_feet
altitude_importance_factor = 1.0

minlon=-86.69027
minlat=36.06589 
maxlon=-86.68008
maxlat=36.07144
proj = pyproj.Proj(proj="utm", zone=16, ellps="WGS84")
inv_proj = pyproj.Proj(proj="utm", zone=16, ellps="WGS84", inverse=True)
osm_origin_x, osm_origin_y = proj(minlon, minlat)

print("Reading DEM")
dem_data = DEM("148110.asc", 2.0, -999999)
print("Reading PCD")
pcd = open3d.io.read_point_cloud("148110.pcd")
print("Postprocessing PCD")
points = np.asarray(pcd.points)
points *= feet_to_meters
pcd.points = open3d.utility.Vector3dVector(points)
pcd_origin_x, pcd_origin_y = pcd.get_min_bound()[0], pcd.get_min_bound()[1]
pcd.translate((-pcd_origin_x, -pcd_origin_y, 0))
points = np.asarray(pcd.points)
points[:, 2] *= altitude_importance_factor
pcd.points = open3d.utility.Vector3dVector(points)

def LidarGetAverageMeterSpacing(pcd):
    points = np.asarray(pcd.points)
    
    # Use built-in KDTree to estimate spacing
    pcd_tree = open3d.geometry.KDTreeFlann(pcd)
    
    nn_dists = []
    for i in range(len(points)):
        [_, idx, dist2] = pcd_tree.search_knn_vector_3d(points[i], 2)  # 2 nearest: itself + 1 neighbor
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
'''
def DownsampleLine(points, target_number):
    """
    Downsample the polyline to the original node number.
    Input: points = (N, 2) or (N, 3) array
    Output: (M, 3) array with interpolated points
    """
    points = np.array(points)
    downsampled = []
    step_amount = int(len(points) / target_number)
    for i in range(target_number):
        index = min(step_amount * i, len(points) - 1)
        downsampled.append(points[index])
    return np.array(downsampled)
'''
def DownsampleLine(points, target_number):
    indices = np.linspace(0, len(points) - 1, target_number, dtype=int)
    return points[indices]

#lidar_spacing = LidarGetAverageMeterSpacing(pcd)
lidar_spacing = 0.21

class WayNodeCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.nodes = {}  # id → (lat, lon)
        self.ways = {}

    def node(self, n):
        coordinates = [n.location.lon, n.location.lat, 0]
        meters_coordinates = self.projectToMeters(coordinates)
        coordinates[2] = dem_data.altitude(meters_coordinates[0] * meters_to_feet, meters_coordinates[1] * meters_to_feet) * feet_to_meters * altitude_importance_factor
        self.nodes[n.id] = {"coordinates": coordinates, "total_ways": [], "corrected_coordinates": np.array([0, 0, 0])}
        
    def way(self, w):
        self.ways[w.id] = {"nodes": [], "corrected_node_positions": []}
        for n in w.nodes:
            self.ways[w.id]["nodes"].append(n.ref)
            self.ways[w.id]["corrected_node_positions"].append(np.array([0, 0, 0])) # Default of zero coordinate
            self.nodes[n.ref]["total_ways"].append(w.id)

    def getWayNodes(self, wid):
        return self.ways[wid]["nodes"]

    def projectToMeters(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x, y = proj(x,y)
        x = x - osm_origin_x
        y = y - osm_origin_y
        return np.array([x, y, z])
    
    def projectToOSM(self, coordinates):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        #print("Starting projection ", x, y, z, osm_origin_x, osm_origin_y)
        x = x + osm_origin_x
        y = y + osm_origin_y
        x, y = proj(x, y, inverse=True)
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

    def transformWayCoordinatesIntoLidarPoints(self, way_coordinates):
        return UpsampleLine(way_coordinates, lidar_spacing)

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

# Load OSM
print("Loading OSM")
osm_file = "148110_full_corrected_2022.osm"
handler = WayNodeCollector()
handler.apply_file(osm_file)

# === Step 2: Convert lat/lon to ENU or UTM ===
# Use pyproj to project into meters (UTM or local projection)
 # change zone as needed
xyz = []
ids = []

print("Lidar correction....")
for wid, way_nodes in handler.ways.items():
    way_nodes = handler.getWayNodes(wid)
    print(wid, len(way_nodes))
    way_coordinates = handler.getWayCoordinates(wid, project_to_meters=True)
    way_lidar_points = handler.transformWayCoordinatesIntoLidarPoints(way_coordinates)
    osm_points = open3d.geometry.PointCloud()
    osm_points.points = open3d.utility.Vector3dVector(way_lidar_points)  # OSM centerline
    threshold = 10.0  # distance threshold
    reg_p2p = open3d.pipelines.registration.registration_icp(
        osm_points, pcd, threshold,
        np.eye(4),  # Initial guess
        open3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    
    #T = reg_p2p.transformation
    #T_translation_only = np.eye(4)
    #T_translation_only[:3, 3] = T[:3, 3]
    adjusted_osm_points = open3d.geometry.PointCloud()
    adjusted_osm_points.points = open3d.utility.Vector3dVector(way_coordinates)
    adjusted_osm_points.transform(reg_p2p.transformation)
    #osm_points.transform(T_translation_only)
    osm_corrected_points = np.asarray(adjusted_osm_points.points)
    print(reg_p2p.transformation)
    handler.annotateWayWithCorrectedPoints(wid, osm_corrected_points)

print("Correcting points.....")
handler.correctNodePoints()

print("Writing new OSM")
# Parse XML using ElementTree to directly update lat/lon
tree = ET.parse(osm_file)
root = tree.getroot()

for elem in root.findall("node"):
    nid = int(elem.attrib['id'])
    if nid in handler.nodes:
        lon, lat = handler.nodes[nid]["corrected_coordinates"][0], handler.nodes[nid]["corrected_coordinates"][1]
        elem.set('lat', f"{lat:.8f}")
        elem.set('lon', f"{lon:.8f}")

# Output file
tree.write("148110_full_corrected_2022_lidar.osm", encoding="utf-8", xml_declaration=True)
