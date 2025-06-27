import os
import subprocess
import json
import pyproj
import xml.etree.ElementTree as ET
from joblib import Parallel, delayed

target_json = "./metadata.json"
subset_json = "./compile_subset.json"
subset_folder = "./SubsetSelection"

feet_to_meters = 0.3048
tiles_x = 3
tiles_y = 3
tile_width = 2000 * feet_to_meters
tile_height = 3000 * feet_to_meters

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

def getSouthWestCoordinate(coords):
    min_lat = min(coord[1] for coord in coords)
    candidates = [coord for coord in coords if coord[1] == min_lat]
    southwest = min(candidates, key=lambda x: x[0])
    return southwest

def getNorthEastCoordinate(coords):
    max_lat = max(coord[1] for coord in coords)
    candidates = [coord for coord in coords if coord[1] == max_lat]
    northeast = max(candidates, key=lambda x: x[0])
    return northeast

def getBoundingBox(metadata_entry):
    try:
        geojson_coordinates = getCoordinatePairs(metadata_entry["GEOJSON"]["coordinates"])
        southwest = getSouthWestCoordinate(geojson_coordinates)
        northeast = getNorthEastCoordinate(geojson_coordinates)
        return [southwest[0], southwest[1], northeast[0], northeast[1]]
    except:
        print(metadata_entry["GEOJSON"]["coordinates"])
        raise Exception(str(metadata_entry["GEOJSON"]["coordinates"]))

def getEntryCoords(metadata_entry):
    try:o
        #geojson_coordinates = numpy.array(metadata_entry["GEOJSON"]["coordinates"]).reshape(-1, 2).tolist()
        geojson_coordinates = getCoordinatePairs(metadata_entry["GEOJSON"]["coordinates"])
        return geojson_coordinates
    except:
        print(metadata_entry["GEOJSON"]["coordinates"])
        raise Exception(str(metadata_entry["GEOJSON"]["coordinates"]))

def getFinalOSMBound(metadata, tiles):
    coords = []
    for tile in tiles:
        tile = str(tile)
        coords += getEntryCoords(metadata[tile])
    southwest = getSouthWestCoordinate(coords)
    northeast = getNorthEastCoordinate(coords)
    print("SW ", southwest)
    print("NE ", northeast)
    return [southwest[0], southwest[1], northeast[0], northeast[1]]

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

def 

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
