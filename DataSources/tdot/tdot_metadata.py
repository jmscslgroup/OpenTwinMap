import os
import json
import pandas
from joblib import Parallel, delayed

class TDOTMetadata:
	GEOJSON_path = "2022_Davidson_County_QL1_Tile_Index.geojson"
	DEM_path = "DEM/Davidson_County_2022_QL1_DEM/Davidson_County_2022_QL1_DEM_tiles"
	LAZ_path = "PointCloud/Davidson_County_TN_2022_QL1_laz/Davidson_County_TN_2022_QL1_laz"
	original_data_path = None
	geojson_data = None
	tile_list = None

	def __init__(self, original_data_path):
		self.original_data_path = original_data_path
		self.GEOJSON_path = os.path.join(self.original_data_path, self.GEOJSON_path)
		self.DEM_path = os.path.join(self.original_data_path, self.DEM_path)
		self.LAZ_path = os.path.join(self.original_data_path, self.LAZ_path)
		self.get_geojson_data()
		self.get_all_tiles()

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
		return [os.path.splitext(entry)[0] for entry in os.listdir(metadata.DEM_path) if (".tif" == os.path.splitext(entry)[1]) and ("." not in os.path.splitext(entry)[0])]

	@staticmethod
	def fetchDEMData(metadata, tile):
		tif_path = os.path.join(metadata.DEM_path, tile + ".tif")
		asc_path = os.path.join(metadata.DEM_path, tile + ".asc")
		bin_path = os.path.join(metadata.DEM_path, tile + ".bin")
		csv_path = os.path.join(metadata.DEM_path, tile + ".csv")
		asc_data_loaded = pandas.read_csv(asc_path, names=["x", "y", "z"], sep=" ", skiprows=6)
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
		parallel_result = Parallel(n_jobs=64, backend="multiprocessing")(delayed(TDOTMetadata.fetchMetaData)(self, tile) for tile in tile_list)
		for i in range(len(tile_list)):
			json_result[tile_list[i]] = parallel_result[i]
		json_result["selected_origin"] = "148110"
		return json_result
	
	def compileMetadata(self, tile_list):
		pass




