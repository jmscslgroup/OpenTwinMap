import os
import json
import open3d
from carla_asset_dataset import CarlaAssetDataset

class CarlaAssetVisualization:
    dataset_path = None
    dataset = None

    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.dataset = CarlaAssetDataset(self.dataset_path, initialized=True)

    def visualizeTerrainMeshes(self):
        terrain_meshes = self.dataset.loadTerrainMeshList(self.dataset.getTerrainMeshesMetadata().values())
        open3d.visualization.draw_geometries(terrain_meshes, mesh_show_back_face=True)