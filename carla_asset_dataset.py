import os
import json

class CarlaAssetDataset:
    dataset_path = None
    terrain_path = None
    terrain_mesh_path = None
    xodr_path = None
    metadata_path = None
    initialized = None
    metadata = None
    terrain_tile_size = None

    def __init__(self, dataset_path, initialized=True):
        self.dataset_path = dataset_path
        self.terrain_path = "terrain/"
        self.terrain_mesh_path = os.path.join(self.terrain_path, "meshes/")
        self.xodr_path = "map.xodr"
        self.metadata_path = "metadata.json"
        self.initialized = initialized
        if self.initialized:
            self._loadMetadata()

    def _loadMetadata(self):
        with open(self.metadata_path, "r") as f:
            self.metadata = json.load(f)

    def getOriginalBounds(self):
        return self.metadata["original_bounds"]

    def getCarlaBounds(self):
        return self.metadata["carla_bounds"]

    def getTerrainMeshesMetadata(self):
        return self.metadata["terrain"]

    def getFullPath(self, relative_path):
        return os.path.join(self.dataset_path, relative_path)

    def getRelativePath(self, full_path):
        return os.path.relpath(full_path, self.dataset_path)

    @staticmethod
    def loadTerrainMesh(mesh_metadata, move_to_position=True):
        import open3d
        import numpy as np
        mesh = open3d.io.read_triangle_mesh(self.getFullPath(mesh_metadata["path"]))
        '''
        R_back = mesh.get_rotation_matrix_from_axis_angle([0, 0, np.radians(90)])
        mesh.rotate(R_back, center=(0, 0, 0))
        '''
        if move_to_position:
            mesh.translate(np.array([mesh_metadata["min_y"], mesh_metadata["min_x"], mesh_metadata["min_z"]]))
        mesh.compute_vertex_normals()
        #camera_location = np.array([(mesh_metadata["min_x"] + mesh_metadata["max_x"]) / 2.0, (mesh_metadata["min_y"] + mesh_metadata["max_y"]) / 2.0, 100000])
        #cpu_mesh = mesh.to_legacy()
        #cpu_mesh.orient_normals_towards_camera_location(camera_location)
        return mesh

    @staticmethod
    def loadTerrainMeshList(mesh_metadata_list):
        mesh_list = []
        for mesh_metadata in mesh_metadata_list:
            mesh = CarlaAssetDataset.loadTerrainMesh(mesh_metadata)
            mesh_list.append(mesh)
            print(mesh_metadata)
        return mesh_list