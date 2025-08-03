from carla_asset_creator import CarlaAssetCreator
from tdot_subset import TDOTSubset

cooked_path = "CarlaCooked/"
tdot_subset_path = "SubsetSelection/"

map_data = TDOTSubset(tdot_subset_path)
converter = CarlaAssetCreator(map_data, cooked_path)

#converter.generateTerrainTile(converter.metadata["bounds"][0], converter.metadata["bounds"][1])
converter.generateTerrain()
converter.convertTerrainFromObjToFbx()
converter.generateRoads()
converter.convertRoadsFromObjToFbx()
converter.saveMetadata()