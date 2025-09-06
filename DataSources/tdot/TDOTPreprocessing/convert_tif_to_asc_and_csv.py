import os
import subprocess
import joblib
from tqdm import tqdm
from pathlib import Path

DEM_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"


def get_all_tiles():
    return [
        os.path.splitext(entry)[0]
        for entry in os.listdir(DEM_path)
        if (".tif" == os.path.splitext(entry)[1])
        and ("." not in os.path.splitext(entry)[0])
    ]


tile_list = get_all_tiles()


def convert_tif_to_asc_and_csv(tif_file):
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


tif_files = [os.path.join(DEM_path, entry + ".tif") for entry in tile_list]


result = list(
    tqdm(
        joblib.Parallel(return_as="generator", n_jobs=48)(
            joblib.delayed(convert_tif_to_asc_and_csv)(tif_file)
            for tif_file in tif_files
        ),
        total=len(tif_files),
    )
)
