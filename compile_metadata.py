import os
import json
import pandas
from joblib import Parallel, delayed

DEM_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
LAZ_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/PointCloud/Davidson_County_2022_QL1_laz/Davidson_County_2022_QL1_laz"
target_json = "./metadata.json"

def get_all_tiles():
	return [os.path.splitext(entry)[0] for entry in os.listdir(DEM_path)]

#tile_list = ["148110"]
tile_list = get_all_tiles()

def fetchDEMData(tile):
	asc_path = os.path.join(DEM_path, tile + ".asc")
	bin_path = os.path.join(DEM_path, tile + ".bin")
	asc_data_loaded = pandas.read_csv(asc_path, names=["x", "y", "z"], sep=" ")
	dem_metadata = {}
	dem_metadata["asc_path"] = asc_path
	dem_metadata["bin_path"] = bin_path
	dem_metadata["x"] = {}
	dem_metadata["y"] = {}
	dem_metadata["z"] = {}
	dem_metadata["x"]["min"] = asc_data_loaded["x"].min()
	dem_metadata["x"]["max"] = asc_data_loaded["x"].max()
	dem_metadata["y"]["min"] = asc_data_loaded["y"].min()
	dem_metadata["y"]["max"] = asc_data_loaded["y"].max()
	dem_metadata["z"]["min"] = asc_data_loaded["z"].min()
	dem_metadata["z"]["max"] = asc_data_loaded["z"].max()
	
	return dem_metadata

def fetchLAZData(tile):
	laz_path = os.path.join(LAZ_path, tile + ".laz")
	laz_metadata = {}
	laz_metadata["path"] = laz_path
	
	return laz_metadata
	
def fetchMetaData(tile):
	result = {}
	result["DEM"] = fetchDEMData(tile)
	result["LAZ"] = fetchLAZData(tile)
	
	return result
	

if __name__ == "__main__":
	json_result = {}
	parallel_result = Parallel(n_jobs=32)(delayed(fetchMetaData)(tile) for tile in tile_list)
	for i in range(len(tile_list)):
		json_result[tile_list[i]] = parallel_result[i]
	with open(target_json, "w+") as f:
		json.dump(json_result, f, indent=8)
	


