import os
import json
import pandas
from .tdot_utils import TDOTUtils
from joblib import Parallel, delayed


class TDOTMetadata:
    GEOJSON_path = "2022_Davidson_County_QL1_Tile_Index.geojson"
    DEM_path = "DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
    LAZ_path = (
        "PointCloud/Davidson_County_TN_2022_QL1_laz/Davidson_County_TN_2022_QL1_laz"
    )
    subset_json_path = "./compile_subset.json"
    original_data_path = None
    geojson_data = None
    tile_list = None

    def __init__(self, original_data_path):
        self.original_data_path = original_data_path
        self.GEOJSON_path = os.path.join(self.original_data_path, self.GEOJSON_path)
        self.DEM_path = os.path.join(self.original_data_path, self.DEM_path)
        self.LAZ_path = os.path.join(self.original_data_path, self.LAZ_path)
        self.subset_json_path = os.path.join(
            os.path.dirname(__file__), self.subset_json_path
        )
        self.get_geojson_data()
        self.get_all_tiles()

    @staticmethod
    def getBoundingBox(metadata_entry):
        try:
            geojson_coordinates = TDOTUtils.getCoordinatePairs(
                metadata_entry["GEOJSON"]["coordinates"]
            )
            southwest = TDOTUtils.getSouthWestCoordinate(geojson_coordinates)
            northeast = TDOTUtils.getNorthEastCoordinate(geojson_coordinates)
            return [southwest[0], southwest[1], northeast[0], northeast[1]]
        except:
            print(metadata_entry["GEOJSON"]["coordinates"])
            raise Exception(str(metadata_entry["GEOJSON"]["coordinates"]))

    @staticmethod
    def getEntryCoords(metadata_entry):
        try:
            geojson_coordinates = TDOTUtils.getCoordinatePairs(
                metadata_entry["GEOJSON"]["coordinates"]
            )
            return geojson_coordinates
        except:
            print(metadata_entry["GEOJSON"]["coordinates"])
            raise Exception(str(metadata_entry["GEOJSON"]["coordinates"]))

    @staticmethod
    def get_geojson_data(metadata):
        with open(metadata.GEOJSON_path, "r") as f:
            data = json.load(f)
        result = {}
        for entry in data["features"]:
            name = entry["properties"]["File_name"]
            result[name] = entry["geometry"]
        return result

    @staticmethod
    def get_all_tiles(metadata):
        return [
            os.path.splitext(entry)[0]
            for entry in os.listdir(metadata.DEM_path)
            if (".tif" == os.path.splitext(entry)[1])
            and ("." not in os.path.splitext(entry)[0])
        ]

    @staticmethod
    def fetchDEMData(metadata, tile):
        tif_path = os.path.join(metadata.DEM_path, tile + ".tif")
        asc_path = os.path.join(metadata.DEM_path, tile + ".asc")
        bin_path = os.path.join(metadata.DEM_path, tile + ".bin")
        csv_path = os.path.join(metadata.DEM_path, tile + ".csv")
        asc_data_loaded = pandas.read_csv(
            asc_path, names=["x", "y", "z"], sep=" ", skiprows=6
        )
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

    @staticmethod
    def fetchLAZData(metadata, tile):
        laz_path = os.path.join(metadata.LAZ_path, tile + ".laz")
        las_path = os.path.join(metadata.LAZ_path, tile + ".las")
        pcd_path = os.path.join(metadata.LAZ_path, tile + ".pcd")
        laz_metadata = {}
        laz_metadata["path"] = laz_path
        laz_metadata["laz_path"] = laz_path
        laz_metadata["las_path"] = las_path
        laz_metadata["pcd_path"] = pcd_path

        return laz_metadata

    @staticmethod
    def fetchGEOJSONData(metadata, tile):
        if tile in metadata.GEOJSON_data:
            print("found tile!")
            return metadata.GEOJSON_data[tile]
        print("Failed to find tile!")
        return None

    @staticmethod
    def fetchMetaData(metadata, tile):
        result = {}
        result["DEM"] = TDOTMetadata.fetchDEMData(metadata, tile)
        result["LAZ"] = TDOTMetadata.fetchLAZData(metadata, tile)
        result["GEOJSON"] = TDOTMetadata.fetchGEOJSONData(metadata, tile)

        return result

    def compileSourceMetadata(self, tile_list):
        json_result = {}
        parallel_result = Parallel(n_jobs=64, backend="multiprocessing")(
            delayed(TDOTMetadata.fetchMetaData)(self, tile) for tile in tile_list
        )
        for i in range(len(tile_list)):
            json_result[tile_list[i]] = parallel_result[i]
        json_result["selected_origin"] = "148110"
        return json_result

    def compileMetadata(self, tile_list):
        cell_size = 2.0
        cell_delta = cell_size / 2.0
        metadata = cls.loadJson(metadata_path)
        subset = cls.loadJson(subset_path)
        metadata_new_path = os.path.join(root_folder, "metadata.json")
        subset_tiles = subset["tiles"]
        min_long, min_lat, max_long, max_lat = getFinalOSMBound(metadata, subset_tiles)

        result_metadata = {}
        result_metadata["tiles"] = {}
        for tile in subset_tiles:
            tile = str(tile)
            tile_data = {}
            tile_data["DEM"] = {}
            tile_data["DEM"]["path"] = os.path.join("dem", tile + ".csv")
            tile_data["DEM"]["x"] = {
                "min": (metadata[tile]["DEM"]["x"]["min"] - cell_delta)
                * cls.feet_to_meters,
                "max": (metadata[tile]["DEM"]["x"]["max"] + cell_delta)
                * cls.feet_to_meters,
            }
            tile_data["DEM"]["y"] = {
                "min": (metadata[tile]["DEM"]["y"]["min"] - cell_delta)
                * cls.feet_to_meters,
                "max": (metadata[tile]["DEM"]["y"]["max"] + cell_delta)
                * cls.feet_to_meters,
            }
            tile_data["DEM"]["z"] = {
                "min": (metadata[tile]["DEM"]["z"]["min"] - cell_delta)
                * cls.feet_to_meters,
                "max": (metadata[tile]["DEM"]["z"]["max"] + cell_delta)
                * cls.feet_to_meters,
            }
            tile_data["LAZ"] = {}
            tile_data["LAZ"]["path"] = os.path.join("pcd", tile + ".pcd")
            tile_data["LAZ"]["bounds"] = {}
            tile_data["LAZ"]["bounds"]["x"] = {
                "min": (metadata[tile]["DEM"]["x"]["min"] - cell_delta)
                * cls.feet_to_meters,
                "max": (metadata[tile]["DEM"]["x"]["max"] + cell_delta)
                * cls.feet_to_meters,
            }
            tile_data["LAZ"]["bounds"]["y"] = {
                "min": (metadata[tile]["DEM"]["y"]["min"] - cell_delta)
                * cls.feet_to_meters,
                "max": (metadata[tile]["DEM"]["y"]["max"] + cell_delta)
                * cls.feet_to_meters,
            }
            tile_data["GEOJSON"] = metadata[tile]["GEOJSON"]
            tile_data["GEOJSON"]["coordinates"] = getCoordinatePairs(
                tile_data["GEOJSON"]["coordinates"]
            )
            tile_data["GEOJSON"]["bounds"] = {}
            tile_data["GEOJSON"]["bounds"]["min"] = getSouthWestCoordinate(
                tile_data["GEOJSON"]["coordinates"]
            )
            tile_data["GEOJSON"]["bounds"]["max"] = getNorthEastCoordinate(
                tile_data["GEOJSON"]["coordinates"]
            )
            result_metadata["tiles"][tile] = tile_data
        result_metadata["bounds"] = {}
        result_metadata["bounds"]["min_long"] = min_long
        result_metadata["bounds"]["min_lat"] = min_lat
        result_metadata["bounds"]["max_long"] = max_long
        result_metadata["bounds"]["max_lat"] = max_lat

        min_meters_x = [
            result_metadata["tiles"][entry]["DEM"]["x"]["min"]
            for entry in result_metadata["tiles"]
        ]
        max_meters_x = [
            result_metadata["tiles"][entry]["DEM"]["x"]["max"]
            for entry in result_metadata["tiles"]
        ]
        min_meters_y = [
            result_metadata["tiles"][entry]["DEM"]["y"]["min"]
            for entry in result_metadata["tiles"]
        ]
        max_meters_y = [
            result_metadata["tiles"][entry]["DEM"]["y"]["max"]
            for entry in result_metadata["tiles"]
        ]
        min_meters_x, min_meters_y, max_meters_x, max_meters_y = (
            min(min_meters_x),
            min(min_meters_y),
            max(max_meters_x),
            max(max_meters_y),
        )

        result_metadata["bounds"]["min_meters_x"] = min_meters_x
        result_metadata["bounds"]["max_meters_x"] = max_meters_x
        result_metadata["bounds"]["min_meters_y"] = min_meters_y
        result_metadata["bounds"]["max_meters_y"] = max_meters_y
