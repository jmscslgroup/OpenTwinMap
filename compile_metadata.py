import os
import json
import pandas
from joblib import Parallel, delayed

GEOJSON_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/2022_Davidson_County_QL1_Tile_Index.geojson"
DEM_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
LAZ_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/PointCloud/Davidson_County_TN_2022_QL1_laz/Davidson_County_TN_2022_QL1_laz"
OSM_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/OSM"
target_json = "./metadata.json"

def get_geojson_data():
	with open(GEOJSON_path, "r") as f:
		data = json.load(f)
	result = {}
	for entry in data["features"]:
		name = entry["properties"]["File_name"]
		result[name] = entry["geometry"]
	return result

def get_all_tiles():
	return [os.path.splitext(entry)[0] for entry in os.listdir(DEM_path)]

#tile_list = ["148110"]
GEOJSON_data = get_geojson_data()
tile_list = get_all_tiles()

def fetchDEMData(tile):
	tif_path = os.path.join(DEM_path, tile + ".tif")
	asc_path = os.path.join(DEM_path, tile + ".asc")
	bin_path = os.path.join(DEM_path, tile + ".bin")
	csv_path = os.path.join(DEM_path, tile + ".csv")
	asc_data_loaded = pandas.read_csv(asc_path, names=["x", "y", "z"], sep=" ")
	dem_metadata = {}
	dem_metadata["path"] = tif_path
	dem_metadata["tif_path"] = tif_path
	dem_metadata["asc_path"] = asc_path
	dem_metadata["bin_path"] = bin_path
	dem_metadata["csv_path"] = csv_path
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
	las_path = os.path.join(LAZ_path, tile + ".las")
	pcd_path = os.path.join(LAZ_path, tile + ".pcd")
	laz_metadata = {}
	laz_metadata["path"] = laz_path
	laz_metadata["laz_path"] = laz_path
	laz_metadata["las_path"] = las_path
	laz_metadata["pcd_path"] = pcd_path
	
	return laz_metadata
	
def fetchGEOJSONData(tile):
	if tile in GEOJSON_data:
		print("found tile!")
		return GEOJSON_data[tile]
	print("Failed to find tile!")
	return None
	
def fetchOSMData(tile):
	xml_path = os.path.join(OSM_path, tile + "_xml.osm")
	json_path = os.path.join(OSM_path, tile + "_json.osm")
	return {"xml_path": xml_path, "json_path": json_path}

def fetchMetaData(tile):
	result = {}
	result["DEM"] = fetchDEMData(tile)
	result["LAZ"] = fetchLAZData(tile)
	result["GEOJSON"] = fetchGEOJSONData(tile)
	result["OpenStreetMap"] = fetchOSMData(tile)
	
	return result
	

if __name__ == "__main__":
	json_result = {}
	parallel_result = Parallel(n_jobs=64, backend="multiprocessing")(delayed(fetchMetaData)(tile) for tile in tile_list)
	for i in range(len(tile_list)):
		json_result[tile_list[i]] = parallel_result[i]
	json_result["selected_origin"] = "148110"
	with open(target_json, "w+") as f:
		json.dump(json_result, f, indent=8)
	


