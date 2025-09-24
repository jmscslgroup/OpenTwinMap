import os
from pathlib import Path
import subprocess
import shutil
import joblib
from tqdm import tqdm


class TDOTDEMCreator:
    root_folder: str = ""
    dem_folder: str = ""

    def __init__(self, root_folder):
        self.root_folder = root_folder
        self.dem_folder = os.path.join(root_folder, "dem")
        os.makedirs(self.dem_folder, exist_ok=True)

    @staticmethod
    def convertTIF(tif_file):
        if not Path(tif_file).exists():
            return
        asc_file = Path(tif_file).with_suffix(".asc")
        csv_file = Path(tif_file).with_suffix(".csv")
        commands = ["gdal_translate", "-of", "AAIGrid", tif_file, asc_file]
        # print("Invoking command ", commands)
        subprocess.run(commands, shell=False)

        commands = ["gdal2xyz.py", asc_file, csv_file]
        # print("Invoking command ", commands)
        subprocess.run(commands, shell=False)

    @staticmethod
    def convertTIFs(dem_folder, subset):
        subset_dem_tif_files = [
            os.path.join(dem_folder, entry+".tif")
            for entry in subset
        ]
        print("Convert tif files...")
        result = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=32)(
                    joblib.delayed(TDOTDEMCreator.convertTIF)(original_tif_path)
                    for original_tif_path in subset_dem_tif_files
                ),
                total=len(subset_dem_tif_files),
            )
        )

    @staticmethod
    def copyDEM(dem_original_path, root_folder, new_dem_path):
        new_pcd_path = os.path.join(root_folder, new_dem_path)
        # dem_pcd_subset_path = Path(self.pcd_folder) / dem_pcd_original_path.name
        shutil.copy(dem_original_path, new_pcd_path)

    def copyDEMs(self, metadata, subset):
        subset_dem_csv_files = [
            (
                metadata["tiles"][str(entry)]["DEM"]["original_csv_path"],
                metadata["tiles"][str(entry)]["DEM"]["path"],
            )
            for entry in subset
        ]
        print("Copying dem files...")
        result = list(
            tqdm(
                joblib.Parallel(return_as="generator", n_jobs=32)(
                    joblib.delayed(TDOTDEMCreator.copyDEM)(
                        original_csv_path, self.root_folder, new_csv_path
                    )
                    for original_csv_path, new_csv_path in subset_dem_csv_files
                ),
                total=len(subset_dem_csv_files),
            )
        )

    def compileDEMSubset(self, metadata, subset):
        self.copyDEMs(metadata, subset)