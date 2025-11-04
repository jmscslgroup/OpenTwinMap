from .DataSources.tdot.tdot_subset import TDOTSubset
from .DataSources.tdot.tdot_creator import TDOTSubsetCreator
from .OpenDrive import osm_to_opendrive
from .CarlaAssetCreation.carla_asset_creator import CarlaAssetCreator
from .CarlaAssetCreation.carla_asset_importer import CarlaAssetImporter
from .OSMLidarCorrection.correct_osm_ways import OSMLidarCorrection
import json
import os


class OpenTwinMap:
    config_file = "../opentwinmap_config.json"

    def __init__(self):
        with open(os.path.join(os.path.dirname(__file__), self.config_file)) as f:
            config_data = json.load(f)
            self.bounds_polygon = config_data["bounds_polygon"],
            self.bounds_bbox = config_data["bounds_bbox"]
            self.unreal_engine_path = config_data["unreal_engine_path"]
            self.carla_path = config_data["carla_path"]
            self.dataset_type = config_data["dataset_type"]
            self.dataset_path = "/workspaces/TDOT_Davidson/"
            self.loaded_dataset_path = config_data["loaded_dataset_path"]
            self.full_cooked_dataset_path = config_data["full_cooked_dataset_path"]

    def importSourceData(self):
        if self.dataset_type == "tdot":
            creator = TDOTSubsetCreator(self.dataset_path, self.loaded_dataset_path)
            creator.compileMetadata()
            creator.compileDEMSubset()
            creator.compileLidarSubset()
            creator.compileOSMSubset()

    def performOSMLidarTuning(self):
        OSMLidarCorrection.performOSMLidarTuning(self.dataset_path, self.loaded_dataset_path, self.dataset_type)

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
        #converter.generateMergedRoads()
        #converter.convertMergedRoadsFromObjToFbx()
        converter.saveMetadata()

    def export3DAssetsToCARLA(self):
        converter = CarlaAssetImporter(
            self.unreal_engine_path, self.carla_path, self.full_cooked_dataset_path
        )
        converter.clearUnrealContent()
        converter.convertTerrainFromFbxToUnreal()
        converter.convertRoadsFromFbxToUnreal()

    def runPipeline(self):
        #self.importSourceData()
        self.performOSMLidarTuning()
        #self.convertOSMToOpenDrive()
        #self.create3DAssets()
        #self.export3DAssetsToCARLA()


if __name__ == "__main__":
    pipeline = OpenTwinMap()
    pipeline.runPipeline()
