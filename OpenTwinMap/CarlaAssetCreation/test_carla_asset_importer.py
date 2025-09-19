from .carla_asset_importer import CarlaAssetImporter

cooked_path = "CarlaCooked/"
converter = CarlaAssetImporter(cooked_path)

converter.clearUnrealContent()
converter.convertTerrainFromFbxToUnreal()
converter.convertRoadsFromFbxToUnreal()
