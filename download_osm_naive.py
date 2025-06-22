import os
import subprocess
import json
from joblib import Parallel, delayed

target_json = "./metadata.json"

def get_metadata():
	with open(target_json, "r") as f:
		data = json.load(f)
	return data

metadata = get_metadata()

def getCoordinatePairs(coordinates):
	total_entries = len(coordinates)
	if (total_entries <= 0):
		return []
	if (type(coordinates[0]) is float):
		if (total_entries != 2):
			raise Exception("Weird pairs - {}".format(str(coordinates)))
		return [coordinates]
	if (type(coordinates[0]) is list):
		result = []
		for i in range(total_entries):
			result = result + getCoordinatePairs(coordinates[i])
		return result

def getBoundingBox(metadata_entry):
	try:
		#geojson_coordinates = numpy.array(metadata_entry["GEOJSON"]["coordinates"]).reshape(-1, 2).tolist()
		geojson_coordinates = getCoordinatePairs(metadata_entry["GEOJSON"]["coordinates"])
		longitudes = [coordinate[0] for coordinate in geojson_coordinates]
		latitudes = [coordinate[1] for coordinate in geojson_coordinates]
		min_long = min(longitudes)
		min_lat = min(latitudes)
		max_long = max(longitudes)
		max_lat = max(latitudes)
		return [min_long, min_lat, max_long, max_lat]
	except:
		print(metadata_entry["GEOJSON"]["coordinates"])
		raise Exception(str(metadata_entry["GEOJSON"]["coordinates"]))

def download_osm(tile):
	metadata_entry = metadata[tile]
	osm_path = metadata_entry["OpenStreetMap"]["path"]
	bounding_box = getBoundingBox(metadata_entry)
	print("Saving path ", osm_path)
	print("Bounding box ", bounding_box)
	print("Invoking ", "https://api.openstreetmap.org/api/0.6/map?bbox=")
	api_path = "https://api.openstreetmap.org/api/0.6/map?bbox={},{},{},{}".format(bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3])
	subprocess.run(["wget", "-O", osm_path, api_path], shell=False)

if __name__ == "__main__":
	tile_list = list(metadata.keys())
	print(tile_list)
	os.makedirs("/home/richarwa/Documents/openstreetmap/TDOT_Davidson/OSM/", exist_ok=True)
	parallel_result = Parallel(n_jobs=2, backend="multiprocessing")(delayed(download_osm)(tile) for tile in tile_list)
	


