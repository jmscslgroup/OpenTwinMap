import os
from .tdot_metadata import TDOTMetadata
from .tdot_utils import TDOTUtils
from .tdot_osm import TDOTOSMCreator
from .tdot_lidar import TDOTLidarCreator
from .tdot_dem import TDOTDEMCreator

class TDOTSubsetCreator:
    original_data_path = None
    subset_folder = None
    metadata_obj = None
    metadata_json = None
    osm_obj = None
    lidar_obj = None
    dem_obj = None
    metadata_path = None

    def __init__(self, original_data_path, subset_folder, bounds_bbox):
        self.original_data_path = original_data_path
        self.subset_folder = subset_folder
        self.bounds_bbox = bounds_bbox
        self.metadata_path = os.path.join(self.subset_folder, "metadata.json")
        os.makedirs(self.subset_folder, exist_ok=True)

    def compileDEMSubset(self):
        if self.metadata_obj is None:
            return
        self.dem_obj = TDOTDEMCreator(self.subset_folder, self.bounds_bbox)
        self.dem_obj.compileDEMSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileLidarSubset(self):
        if self.metadata_obj is None:
            return
        self.lidar_obj = TDOTLidarCreator(self.subset_folder, self.bounds_bbox)
        self.lidar_obj.compileLidarSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileOSMSubset(self):
        if self.metadata_obj is None:
            return
        self.osm_obj = TDOTOSMCreator(self.subset_folder, self.bounds_bbox)
        self.osm_obj.compileOSMSubset(
            self.metadata_json, self.metadata_obj.subset_tiles
        )

    def compileMetadata(self):
        self.metadata_obj = TDOTMetadata(self.original_data_path, self.bounds_bbox)
        TDOTDEMCreator.convertTIFs(self.metadata_obj.DEM_path, self.metadata_obj.subset_tiles)
        self.metadata_json = self.metadata_obj.compileMetadata()
        TDOTUtils.writeJson(self.metadata_path, self.metadata_json)
