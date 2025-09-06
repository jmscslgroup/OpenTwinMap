from DataSources.tdot.tdot_subset import TDOTSubset
from OpenDrive import osm_to_opendrive
from CarlaAssetCreation.carla_asset_creator import CarlaAssetCreator
from CarlaAssetCreation.carla_asset_importer import CarlaAssetImporter

class OpenTwinMap:
    unreal_engine_path = "/home/richarwa/SecondSSD/UnrealEngine_4.26/"
    carla_path = "/home/richarwa/SecondSSD/carla/"
    dataset_type = "tdot"
    dataset_path = "/home/richarwa/Documents/openstreetmap/TDOT_Davidson/"
    loaded_dataset_path = "./SubsetSelection/"
    full_cooked_dataset_path = "./CarlaCooked/"
    
    def __init__(self):
        pass

    def importSourceData(self):
        pass

    def performOSMLidarTuning(self):
        pass

    def convertOSMToOpenDrive(self):
        map_data = TDOTSubset(self.loaded_dataset_path)
        converter = osm_to_opendrive.OSMToOpenDrive(map_data)
        converter.readOSM()
        opendrive_data = converter.convertToOpenDrive()
        opendrive_data.writeFile(map_data.getXODRPath())

    def create3DAssets(self):
        map_data = TDOTSubset(self.loaded_dataset_path)
        converter = CarlaAssetCreator(map_data, self.full_cooked_dataset_path)
        converter.generateTerrain()
        converter.convertTerrainFromObjToFbx()
        converter.generateRoads()
        converter.convertRoadsFromObjToFbx()
        converter.generateMergedRoads()
        converter.convertMergedRoadsFromObjToFbx()
        converter.saveMetadata()

    def export3DAssetsToCARLA(self):
        converter = CarlaAssetImporter(self.unreal_engine_path, self.carla_path, self.full_cooked_dataset_path)
        converter.clearUnrealContent()
        converter.convertTerrainFromFbxToUnreal()
        converter.convertRoadsFromFbxToUnreal()

    def runPipeline(self):
        self.importSourceData()
        self.performOSMLidarTuning()
        self.convertOSMToOpenDrive()
        self.create3DAssets()
        self.export3DAssetsToCARLA()

if __name__ == "__main__":
    pipeline = OpenTwinMap()
    pipeline.runPipeline()
