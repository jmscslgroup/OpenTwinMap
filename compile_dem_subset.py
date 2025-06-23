import json
import subprocess
import os

DEM_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
subset_folder = "./SubsetSelection"
os.makedirs(subset_folder, exist_ok=True)
target_json = "./metadata.json"
subset_json = "./compile_subset.json"
target_dem_tif = os.path.join(subset_folder, "dem_subset.tif")
target_dem_asc = os.path.join(subset_folder, "dem_subset.asc")
target_dem_csv = os.path.join(subset_folder, "dem_subset.csv")
nodata = str("-999999")

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
    commands = ["gdal_merge.py", "-o", target_dem_tif, "-of", "GTiff", "-n", nodata, "-a_nodata", nodata]
    for tile in tiles:
        commands.append(os.path.join(DEM_path, str(tile)+".tif"))
    #print("Invoking command ", commands)
    subprocess.run(commands, shell=False)

    commands = ["gdal_translate", "-of", "AAIGrid", target_dem_tif, target_dem_asc]
    #print("Invoking command ", commands)
    subprocess.run(commands, shell=False)
    
    commands = ["gdal2xyz.py", target_dem_asc, target_dem_csv]
    #print("Invoking command ", commands)
    subprocess.run(commands, shell=False)

compile_subset()