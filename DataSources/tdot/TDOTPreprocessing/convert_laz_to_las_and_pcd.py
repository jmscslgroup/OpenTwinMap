import json
import subprocess
import laspy
import numpy
import joblib
from tqdm import tqdm
from pathlib import Path
import open3d

target_json = "./metadata.json"

meters_to_feet = 3.28084
feet_to_meters = 1.0 / meters_to_feet


def get_metadata():
    with open(target_json, "r") as f:
        data = json.load(f)
    return data


metadata = get_metadata()


def convert_laz_to_las_and_pcd(laz_file):
    if not Path(laz_file).exists():
        return
    las_file = Path(laz_file).with_suffix(".las")
    pcd_file = Path(laz_file).with_suffix(".pcd")
    print(laz_file, las_file, pcd_file)
    commands = ["pdal", "translate", laz_file, las_file]
    subprocess.run(commands, shell=False)

    print("Converting las into pcd....")
    las = laspy.read(las_file)
    points = numpy.vstack((las.x, las.y, las.z)).transpose()
    points *= feet_to_meters

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)

    print("Writing to disk....")
    open3d.io.write_point_cloud(pcd_file, pcd)


laz_files = [
    metadata[entry]["LAZ"]["path"] for entry in metadata if entry != "selected_origin"
]


result = list(
    tqdm(
        joblib.Parallel(return_as="generator", n_jobs=48)(
            joblib.delayed(convert_laz_to_las_and_pcd)(laz_file)
            for laz_file in laz_files
        ),
        total=len(laz_files),
    )
)
# for laz_file in laz_files:
#    convert_laz_to_las_and_pcd(laz_file)
