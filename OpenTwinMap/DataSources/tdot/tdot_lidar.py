import os
import joblib
from pathlib import Path
import shutil
from tqdm import tqdm
import open3d
import numpy
import laspy
import subprocess
from .tdot_utils import TDOTUtils


class TDOTLidarCreator:
    root_folder: str = ""
    pcd_folder: str = ""

    def __init__(self, root_folder):
        self.root_folder = root_folder
        self.pcd_folder = os.path.join(self.root_folder, "pcd")
        os.makedirs(self.pcd_folder, exist_ok=True)

    @staticmethod
    def convertLAZ(laz_file):
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
        points *= TDOTUtils.feet_to_meters

        print("Building cloud and computing normals....")
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(points)
        pcd.estimate_normals(
            search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=15)
        )
        pcd.normalize_normals()

        print("Writing to disk....")
        open3d.io.write_point_cloud(pcd_file, pcd)

    @staticmethod
    def copyPCD(pcd_original_path, root_folder, new_pcd_path):
        new_pcd_path = os.path.join(root_folder, new_pcd_path)
        # dem_pcd_subset_path = Path(self.pcd_folder) / dem_pcd_original_path.name
        shutil.copy(pcd_original_path, new_pcd_path)

    def compileLidarSubset(self, metadata, subset):
        subset_laz_files = [
            metadata["tiles"][str(entry)]["LAZ"]["original_laz_path"]
            for entry in subset
        ]
        print("Converting LAZ files...")
        result = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=32)(
                    joblib.delayed(TDOTLidarCreator.convertLAZ)(laz_original_path)
                    for laz_original_path in subset_laz_files
                ),
                total=len(subset_laz_files),
            )
        )

        subset_pcd_files = [
            (
                metadata["tiles"][str(entry)]["LAZ"]["original_pcd_path"],
                metadata["tiles"][str(entry)]["LAZ"]["path"],
            )
            for entry in subset
        ]
        print("Copying PCD files...")
        result = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=32)(
                    joblib.delayed(TDOTLidarCreator.copyPCD)(
                        pcd_original_path, self.root_folder, new_pcd_path
                    )
                    for pcd_original_path, new_pcd_path in subset_pcd_files
                ),
                total=len(subset_pcd_files),
            )
        )
