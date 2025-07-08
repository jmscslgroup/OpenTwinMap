import json
import os
import subprocess
import laspy
import numpy
import joblib
import shutil
from tqdm import tqdm
from pathlib import Path
import open3d
import xml.etree.ElementTree as ET

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
    try:
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

def download_osm(osm_folder, metadata, tile):
    tile = str(tile)
    metadata_entry = metadata[tile]
    bounding_box = getBoundingBox(metadata_entry)
    osm_path = os.path.join(osm_folder, tile+".osm")
    print("Saving path ", osm_path)
    print("Bounding box ", bounding_box)
    print("Invoking ", "https://api.openstreetmap.org/api/0.6/map?bbox=")
    api_path = "https://api.openstreetmap.org/api/0.6/map?bbox={},{},{},{}".format(bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3])
    subprocess.run(["wget", "-O", osm_path, api_path], shell=False)


class TDOTSubset:

    @classmethod
    def load_json(cls, json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
        return data

    @classmethod
    def write_json(cls, json_path, json_data):
        with open(json_path, "w+") as f:
            data = json.dump(json_data, f, indent=8)

    @classmethod
    def compile_dem_subset(cls, metadata_path, subset_path, root_folder):
        metadata = cls.load_json(metadata_path)
        subset = cls.load_json(subset_path)
        dem_data_path = os.path.join(root_folder, "dem")
        os.makedirs(dem_data_path, exist_ok=True)

        def copy_dem(dem_csv_original_path):
            dem_csv_original_path = Path(dem_csv_original_path)
            dem_csv_subset_path = Path(dem_data_path) / dem_csv_original_path.name
            shutil.copy(dem_csv_original_path, dem_csv_subset_path)
        subset_dem_files = [metadata[str(entry)]["DEM"]["csv_path"] for entry in subset["tiles"]]
        print("Copying dem files...")
        result = list(tqdm(
            joblib.Parallel(return_as="generator", n_jobs=32)
            (joblib.delayed(copy_dem)(dem_csv_original_path) for dem_csv_original_path in subset_dem_files), 
            total=len(subset_dem_files)
        ))

    @classmethod
    def compile_lidar_subset(cls, metadata_path, subset_path, root_folder):
        metadata = cls.load_json(metadata_path)
        subset = cls.load_json(subset_path)
        pcd_data_path = os.path.join(root_folder, "pcd")
        os.makedirs(pcd_data_path, exist_ok=True)

        def copy_pcd(dem_pcd_original_path):
            dem_pcd_original_path = Path(dem_pcd_original_path)
            dem_pcd_subset_path = Path(pcd_data_path) / dem_pcd_original_path.name
            shutil.copy(dem_pcd_original_path, dem_pcd_subset_path)
        subset_pcd_files = [metadata[str(entry)]["LAZ"]["pcd_path"] for entry in subset["tiles"]]
        print("Copying PCD files...")
        result = list(tqdm(
            joblib.Parallel(return_as="generator", n_jobs=32)
            (joblib.delayed(copy_pcd)(dem_pcd_original_path) for dem_pcd_original_path in subset_pcd_files), 
            total=len(subset_pcd_files)
        ))

    @classmethod
    def compile_osm_subset(cls, metadata_path, subset_path, root_folder):

        merged_osm_original = os.path.join(root_folder, "osm_subset_merged_no_bounds.osm")
        merged_osm_final = os.path.join(root_folder, "osm_subset.osm")
        osm_folder = os.path.join(root_folder, "osm/")
        os.makedirs(osm_folder, exist_ok=True)

        metadata = cls.load_json(metadata_path)
        subset = cls.load_json(subset_path)

        tile_list = subset["tiles"]
        print(tile_list)
        parallel_result = joblib.Parallel(n_jobs=2, backend="multiprocessing")(joblib.delayed(download_osm)(osm_folder, metadata, tile) for tile in tile_list)

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

    @classmethod
    def compile_metadata(cls, metadata_path, subset_path, root_folder):
        feet_to_meters = 0.3048
        cell_size = 2.0
        cell_delta = cell_size / 2.0
        metadata = cls.load_json(metadata_path)
        subset = cls.load_json(subset_path)
        metadata_new_path = os.path.join(root_folder, "metadata.json")
        subset_tiles = subset["tiles"]
        min_long, min_lat, max_long, max_lat = getFinalOSMBound(metadata, subset_tiles)

        result_metadata = {}
        result_metadata["tiles"] = {}
        for tile in subset_tiles:
            tile = str(tile)
            tile_data = {}
            tile_data["DEM"] = {}
            tile_data["DEM"]["path"] = os.path.join("dem", tile+".csv")
            tile_data["DEM"]["x"] = metadata[tile]["DEM"]["x"]
            tile_data["DEM"]["y"] = metadata[tile]["DEM"]["y"]
            tile_data["DEM"]["z"] = metadata[tile]["DEM"]["z"]
            tile_data["LAZ"] = {}
            tile_data["LAZ"]["path"] = os.path.join("pcd", tile+".pcd")
            tile_data["LAZ"]["bounds"] = {}
            tile_data["LAZ"]["bounds"]["x"] = {"min": (metadata[tile]["DEM"]["x"]["min"] - cell_delta) * feet_to_meters, "max": (metadata[tile]["DEM"]["x"]["max"] + cell_delta) * feet_to_meters}
            tile_data["LAZ"]["bounds"]["y"] = {"min": (metadata[tile]["DEM"]["y"]["min"] - cell_delta) * feet_to_meters, "max": (metadata[tile]["DEM"]["y"]["max"] + cell_delta) * feet_to_meters}
            tile_data["GEOJSON"] = metadata[tile]["GEOJSON"]
            tile_data["GEOJSON"]["coordinates"] = getCoordinatePairs(tile_data["GEOJSON"]["coordinates"])
            result_metadata["tiles"][tile] = tile_data
        result_metadata["bounds"] = {}
        result_metadata["bounds"]["min_long"] = min_long
        result_metadata["bounds"]["min_lat"] = min_lat
        result_metadata["bounds"]["max_long"] = max_long
        result_metadata["bounds"]["max_lat"] = max_lat
        
        min_meters_x = [result_metadata["tiles"][entry]["DEM"]["x"]["min"] for entry in result_metadata["tiles"]]
        max_meters_x = [result_metadata["tiles"][entry]["DEM"]["x"]["max"] for entry in result_metadata["tiles"]]
        min_meters_y = [result_metadata["tiles"][entry]["DEM"]["y"]["min"] for entry in result_metadata["tiles"]]
        max_meters_y = [result_metadata["tiles"][entry]["DEM"]["y"]["max"] for entry in result_metadata["tiles"]]
        min_meters_x, min_meters_y, max_meters_x, max_meters_y = min(min_meters_x), min(min_meters_y), max(max_meters_x), max(max_meters_y)
        
        result_metadata["bounds"]["min_meters_x"] = min_meters_x
        result_metadata["bounds"]["max_meters_x"] = max_meters_x
        result_metadata["bounds"]["min_meters_y"] = min_meters_y
        result_metadata["bounds"]["max_meters_y"] = max_meters_y
        cls.write_json(metadata_new_path, result_metadata)

if __name__ == "__main__":
    metadata_path = "./metadata.json"
    subset_path = "./compile_subset.json"
    subset_folder = "SubsetSelection"

    os.makedirs(subset_folder, exist_ok=True)
    print("Compiling metadata")
    TDOTSubset.compile_metadata(metadata_path, subset_path, subset_folder)
    print("Compiling OSM")
    TDOTSubset.compile_osm_subset(metadata_path, subset_path, subset_folder)
    print("Compiling DEM")
    TDOTSubset.compile_dem_subset(metadata_path, subset_path, subset_folder)
    print("Compiling LIDAR")
    TDOTSubset.compile_lidar_subset(metadata_path, subset_path, subset_folder)
