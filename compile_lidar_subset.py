import json
import subprocess
import os
import laspy
import numpy
import open3d

lidar_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/PointCloud/Davidson_County_TN_2022_QL1_laz/Davidson_County_TN_2022_QL1_laz"
subset_folder = "./SubsetSelection"
os.makedirs(subset_folder, exist_ok=True)
target_json = "./metadata.json"
subset_json = "./compile_subset.json"
target_las = os.path.join(subset_folder, "lidar_subset.las")
target_pcd = os.path.join(subset_folder, "lidar_subset.pcd")

meters_to_feet = 3.28084
feet_to_meters = 1.0/meters_to_feet

def get_metadata():
	with open(target_json, "r") as f:
		data = json.load(f)
	return data

def get_subset():
    with open(subset_json, "r") as f:
        data = json.load(f)
    return data

metadata = get_metadata()
subset = get_subset()

def compile_subset():
    tiles = subset["tiles"]
    #gdal_merge.py -o merged.asc -of AAIGrid -n -9999 -a_nodata -9999 tile1.asc tile2.asc tile3.asc tile4.asc
    print("Compiling subset ", tiles)
    commands = ["pdal", "merge"]
    for tile in tiles:
        commands.append(os.path.join(lidar_path, str(tile)+".laz"))
    commands.append(target_las)
    subprocess.run(commands, shell=False)

    print("Converting las into pcd....")
    las = laspy.read(target_las)
    points = numpy.vstack((las.x, las.y, las.z)).transpose()
    points *= feet_to_meters

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    pcd_origin_x, pcd_origin_y = pcd.get_min_bound()[0], pcd.get_min_bound()[1]
    pcd.translate((-pcd_origin_x, -pcd_origin_y, 0))

    print("Computing normals....")
    pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamKNN(knn=30))

    print("Writing to disk....")
    open3d.io.write_point_cloud(target_pcd, pcd)
    
compile_subset()