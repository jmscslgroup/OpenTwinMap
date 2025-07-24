from carla_asset_importer import CarlaAssetImporter
from tdot_subset import TDOTSubset

cooked_path = "CarlaCooked/"
tdot_subset_path = "SubsetSelection/"

map_data = TDOTSubset(tdot_subset_path)
converter = CarlaAssetImporter(map_data, cooked_path)

converter.copyXODR()
converter.generateTerrain()
converter.saveMetadata()