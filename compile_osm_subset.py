import os
import subprocess
import json
import xml.etree.ElementTree as ET
from joblib import Parallel, delayed

target_json = "./metadata.json"
subset_json = "./compile_subset.json"
subset_folder = "./SubsetSelection"

def get_metadata():
	with open(target_json, "r") as f:
		data = json.load(f)
	return data

def get_subset():
    with open(subset_json, "r") as f:
        data = json.load(f)
    return data

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

def getFinalOSMBound(metadata, tiles):
    result = None
    for tile in tiles:
        tile = str(tile)
        bounding_box = getBoundingBox(metadata[tile])
        if result is None:
            result = bounding_box
        else:
            if (result[0] > bounding_box[0]):
                result[0] = bounding_box[0]
            if (result[1] > bounding_box[1]):
                result[1] = bounding_box[1]
            if (result[2] < bounding_box[2]):
                result[2] = bounding_box[2]
            if (result[3] < bounding_box[3]):
                result[3] = bounding_box[3]
    return result

def download_osm(metadata, tile):
    tile = str(tile)
    metadata_entry = metadata[tile]
    bounding_box = getBoundingBox(metadata_entry)
    osm_path = os.path.join(osm_folder, tile+".osm")
    print("Saving path ", osm_path)
    print("Bounding box ", bounding_box)
    print("Invoking ", "https://api.openstreetmap.org/api/0.6/map?bbox=")
    api_path = "https://api.openstreetmap.org/api/0.6/map?bbox={},{},{},{}".format(bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3])
    subprocess.run(["wget", "-O", osm_path, api_path], shell=False)

if __name__ == "__main__":
    merged_osm_original = os.path.join(subset_folder, "osm_subset_merged.osm")
    merged_osm_final = os.path.join(subset_folder, "osm_subset.osm")
    osm_folder = os.path.join(subset_folder, "OSM/")
    os.makedirs(osm_folder, exist_ok=True)
    compiled_osm_target = os.path.join(subset_folder, "subset.osm")
    metadata = get_metadata()
    subset = get_subset()

    tile_list = subset["tiles"]
    print(tile_list)
    parallel_result = Parallel(n_jobs=2, backend="multiprocessing")(delayed(download_osm)(metadata, tile) for tile in tile_list)

    commands = ["osmium", "merge"]
    for tile in tile_list:
        tile = str(tile)
        osm_path = os.path.join(osm_folder, tile+".osm")
        commands.append(osm_path)
    commands.append("-o")
    commands.append(merged_osm_original)
    subprocess.run(commands, shell=False)

    min_long, min_lat, max_long, max_lat = getFinalOSMBound(metadata, tile_list)
    tree = ET.parse(merged_osm_original)
    root = tree.getroot()
    current_bounds_tag = root.findall("bounds")
    if (len(current_bounds_tag) == 0):
        bounds = ET.Element("bounds", {
            "minlat": f"{min_lat:.8f}",
            "minlon": f"{min_long:.8f}",
            "maxlat": f"{max_lat:.8f}",
            "maxlon": f"{max_long:.8f}",
        })
        root.insert(0, bounds)
    else:
        current_bounds_tag.set("minlat", f"{min_lat:.8f}")
        current_bounds_tag.set("minlon", f"{min_long:.8f}")
        current_bounds_tag.set("maxlat", f"{max_lat:.8f}")
        current_bounds_tag.set("maxlon", f"{max_long:.8f}")

    tree.write(merged_osm_final, encoding="utf-8", xml_declaration=True)
