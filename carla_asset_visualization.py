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
        print([self.dataset.getBounds()[0], self.dataset.getBounds()[1], self.dataset.getBounds()[2]])
        terrain_meshes.append(open3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0, origin=[self.dataset.getBounds()[0], self.dataset.getBounds()[1], self.dataset.getBounds()[2]]))
        open3d.visualization.draw_geometries(terrain_meshes)#, mesh_show_back_face=True)