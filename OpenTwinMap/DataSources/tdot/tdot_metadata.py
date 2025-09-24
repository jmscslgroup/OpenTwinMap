import os
import json
import pandas
from .tdot_utils import TDOTUtils
from joblib import Parallel, delayed


class TDOTMetadata:
    GEOJSON_path: str = "2022_Davidson_County_QL1_Tile_Index.geojson"
    DEM_path: str = (
        "DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
    )
    LAZ_path: str = (
        "PointCloud/Davidson_County_TN_2022_QL1_laz/Davidson_County_TN_2022_QL1_laz"
    )
    subset_json_path: str = "./compile_subset.json"
    original_data_path: str = ""
    geojson_data = None
    all_tiles = []
    subset_tiles = []

    def __init__(self, original_data_path):
        self.original_data_path = original_data_path
        self.GEOJSON_path = os.path.join(self.original_data_path, self.GEOJSON_path)
        self.DEM_path = os.path.join(self.original_data_path, self.DEM_path)
        self.LAZ_path = os.path.join(self.original_data_path, self.LAZ_path)
        self.subset_json_path = os.path.join(
            os.path.dirname(__file__), self.subset_json_path
        )
        self.geojson_data = self.getGEOJSONData()
        self.all_tiles = self.getAllTiles()
        self.subset_tiles = self.getTileList()

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
    def getFinalOSMBound(metadata, tiles):
        coords = []
        for tile in tiles:
            tile = str(tile)
            coords += TDOTMetadata.getEntryCoords(metadata["tiles"][tile])
        southwest = TDOTUtils.getSouthWestCoordinate(coords)
        northeast = TDOTUtils.getNorthEastCoordinate(coords)
        print("SW ", southwest)
        print("NE ", northeast)
        return [southwest[0], southwest[1], northeast[0], northeast[1]]

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

    def getGEOJSONData(self):
        with open(self.GEOJSON_path, "r") as f:
            data = json.load(f)
        result = {}
        for entry in data["features"]:
            name = entry["properties"]["File_name"]
            result[name] = entry["geometry"]
        return result

    def getAllTiles(self):
        return [
            os.path.splitext(entry)[0]
            for entry in os.listdir(self.DEM_path)
            if (".tif" == os.path.splitext(entry)[1])
            and ("." not in os.path.splitext(entry)[0])
        ]

    @staticmethod
    def fetchDEMData(metadata, tile):
        tif_path = os.path.join(metadata.DEM_path, tile + ".tif")
        asc_path = os.path.join(metadata.DEM_path, tile + ".asc")
        bin_path = os.path.join(metadata.DEM_path, tile + ".bin")
        csv_path = os.path.join(metadata.DEM_path, tile + ".csv")
        csv_data_loaded = pandas.read_csv(
            csv_path, names=["x", "y", "z"], sep=" ", skiprows=0
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
        dem_metadata["x"]["min"] = csv_data_loaded["x"].min()
        dem_metadata["x"]["max"] = csv_data_loaded["x"].max()
        dem_metadata["y"]["min"] = csv_data_loaded["y"].min()
        dem_metadata["y"]["max"] = csv_data_loaded["y"].max()
        dem_metadata["z"]["min"] = csv_data_loaded["z"].min()
        dem_metadata["z"]["max"] = csv_data_loaded["z"].max()

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
        if tile in metadata.geojson_data:
            print("found tile!")
            return metadata.geojson_data[tile]
        print("Failed to find tile!")
        return None

    @staticmethod
    def fetchMetaData(metadata, tile):
        result = {}
        result["DEM"] = TDOTMetadata.fetchDEMData(metadata, tile)
        result["LAZ"] = TDOTMetadata.fetchLAZData(metadata, tile)
        result["GEOJSON"] = TDOTMetadata.fetchGEOJSONData(metadata, tile)

        return result

    def getTileList(self):
        return TDOTUtils.loadJson(self.subset_json_path)["tiles"]

    def compileSourceMetadata(self):
        json_result = {}
        parallel_result = Parallel(n_jobs=64, backend="multiprocessing")(
            delayed(TDOTMetadata.fetchMetaData)(self, tile)
            for tile in self.subset_tiles
        )
        for i in range(len(self.subset_tiles)):
            json_result[self.subset_tiles[i]] = parallel_result[i]
        json_result["selected_origin"] = "148110"
        return json_result

    def compileMetadata(self):
        source_metadata = self.compileSourceMetadata()
        result_metadata = {}
        result_metadata["tiles"] = {}
        for tile in self.subset_tiles:
            tile = str(tile)
            tile_data = {}
            tile_data["DEM"] = {}
            tile_data["DEM"]["original_tif_path"] = source_metadata[tile]["DEM"][
                "tif_path"
            ]
            tile_data["DEM"]["original_asc_path"] = source_metadata[tile]["DEM"][
                "asc_path"
            ]
            tile_data["DEM"]["original_bin_path"] = source_metadata[tile]["DEM"][
                "bin_path"
            ]
            tile_data["DEM"]["original_csv_path"] = source_metadata[tile]["DEM"][
                "csv_path"
            ]
            tile_data["DEM"]["path"] = os.path.join("dem", tile + ".csv")
            tile_data["DEM"]["x"] = {
                "min": (
                    source_metadata[tile]["DEM"]["x"]["min"] - TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
                "max": (
                    source_metadata[tile]["DEM"]["x"]["max"] + TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
            }
            tile_data["DEM"]["y"] = {
                "min": (
                    source_metadata[tile]["DEM"]["y"]["min"] - TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
                "max": (
                    source_metadata[tile]["DEM"]["y"]["max"] + TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
            }
            tile_data["DEM"]["z"] = {
                "min": (
                    source_metadata[tile]["DEM"]["z"]["min"] - TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
                "max": (
                    source_metadata[tile]["DEM"]["z"]["max"] + TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
            }
            tile_data["LAZ"] = {}
            tile_data["LAZ"]["original_laz_path"] = source_metadata[tile]["LAZ"][
                "laz_path"
            ]
            tile_data["LAZ"]["original_las_path"] = source_metadata[tile]["LAZ"][
                "las_path"
            ]
            tile_data["LAZ"]["original_pcd_path"] = source_metadata[tile]["LAZ"][
                "pcd_path"
            ]
            tile_data["LAZ"]["path"] = os.path.join("pcd", tile + ".pcd")
            tile_data["LAZ"]["bounds"] = {}
            tile_data["LAZ"]["bounds"]["x"] = {
                "min": (
                    source_metadata[tile]["DEM"]["x"]["min"] - TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
                "max": (
                    source_metadata[tile]["DEM"]["x"]["max"] + TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
            }
            tile_data["LAZ"]["bounds"]["y"] = {
                "min": (
                    source_metadata[tile]["DEM"]["y"]["min"] - TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
                "max": (
                    source_metadata[tile]["DEM"]["y"]["max"] + TDOTUtils.dem_cell_delta
                )
                * TDOTUtils.feet_to_meters,
            }
            tile_data["GEOJSON"] = source_metadata[tile]["GEOJSON"]
            tile_data["GEOJSON"]["coordinates"] = TDOTUtils.getCoordinatePairs(
                tile_data["GEOJSON"]["coordinates"]
            )
            tile_data["GEOJSON"]["bounds"] = {}
            tile_data["GEOJSON"]["bounds"]["min"] = TDOTUtils.getSouthWestCoordinate(
                tile_data["GEOJSON"]["coordinates"]
            )
            tile_data["GEOJSON"]["bounds"]["max"] = TDOTUtils.getNorthEastCoordinate(
                tile_data["GEOJSON"]["coordinates"]
            )
            result_metadata["tiles"][tile] = tile_data
        min_long, min_lat, max_long, max_lat = TDOTMetadata.getFinalOSMBound(
            result_metadata, self.subset_tiles
        )
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
        result_metadata["source_metadata"] = source_metadata
        return result_metadata
