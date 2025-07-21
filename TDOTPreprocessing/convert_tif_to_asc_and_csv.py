import json
import subprocess
import joblib
from tqdm import tqdm
from pathlib import Path

target_json = "./metadata.json"

def get_metadata():
	with open(target_json, "r") as f:
		data = json.load(f)
	return data

metadata = get_metadata()

def convert_tif_to_asc_and_csv(tif_file):
    if not Path(tif_file).exists():
        return
    asc_file = Path(tif_file).with_suffix(".asc")
    csv_file = Path(tif_file).with_suffix(".csv")
    commands = ["gdal_translate", "-of", "AAIGrid", tif_file, asc_file]
    #print("Invoking command ", commands)
    subprocess.run(commands, shell=False)
    
    commands = ["gdal2xyz.py", asc_file, csv_file]
    #print("Invoking command ", commands)
    subprocess.run(commands, shell=False)

tif_files = [metadata[entry]["DEM"]["tif_path"] for entry in metadata if entry != "selected_origin"]


result = list(tqdm(
    joblib.Parallel(return_as="generator", n_jobs=48)
    (joblib.delayed(convert_tif_to_asc_and_csv)(tif_file) for tif_file in tif_files), 
    total=len(tif_files)
))
