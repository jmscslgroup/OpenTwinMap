import os
import json
import trimesh
import numpy as np
import shutil

class CarlaAssetImporter:
    map_data = None
    cooked_path = None
    terrain_path = None
    xodr_path = None
    metadata_path = None
    metadata = {}
    dem_data = None
    tile_size = 20.0
    tile_point_interval = 0.1

    def __init__(self, map_data, cooked_path):
        self.map_data = map_data
        self.cooked_path = cooked_path
        self.terrain_path = os.path.join(self.cooked_path, "terrain/")
        self.xodr_path = os.path.join(self.cooked_path, "map.xodr")
        self.metadata_path = os.path.join(self.cooked_path, "metadata.json")
        os.makedirs(self.cooked_path, exist_ok=True)
        self.metadata["bounds"] = self.map_data.getBoundsInMeters().tolist()
        self.metadata["terrain"] = {}
        self.dem_data = self.map_data.loadDEMs(self.map_data.getAllTiles())

    def copyXODR(self):
        shutil.copy(self.map_data.getXODRPath(), self.xodr_path)
        self.metadata["xodr_path"] = self.xodr_path

    def generateTerrainPath(self, x, y):
        return os.path.join(self.terrain_path, "meshes/", f"{int(x*1000)}_{int(y*1000)}.obj")

    def generateTerrainTile(self, x, y):
        number_of_points_on_side = int(self.tile_size / self.tile_point_interval)
        mesh_folder = os.path.join(self.terrain_path, "meshes/")
        os.makedirs(mesh_folder, exist_ok=True)
        mesh_path = self.generateTerrainPath(x,y)
        xs = np.linspace(x, x + self.tile_size, number_of_points_on_side)
        ys = np.linspace(y, y + self.tile_size, number_of_points_on_side)
        xv, yv = np.meshgrid(xs, ys)
        zv = np.zeros_like(xv)
        for i in range(yv.shape[0]):
            for j in range(xv.shape[1]):
                zv[i][j] = self.map_data.minHeightAtXYMeters(self.dem_data, (xv[i][j], yv[i][j]))
        xv = xv.flatten()
        yv = yv.flatten()
        zv = zv.flatten()
        vertices = np.stack([xv, yv, zv], axis=1)
        faces = []
        for i in range(number_of_points_on_side - 1):
            for j in range(number_of_points_on_side - 1):
                i0 = i * number_of_points_on_side + j
                i1 = i0 + 1
                i2 = i0 + number_of_points_on_side
                i3 = i2 + 1
                # Two triangles per grid square
                faces.append([i0, i2, i1])
                faces.append([i1, i2, i3])
        faces = np.array(faces)
        uv_x = xv / self.tile_size
        uv_y = yv / self.tile_size
        uvs = np.stack([uv_x, uv_y], axis=1)
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        mesh.visual.uv = uvs
        # Remember, in Unreal, x becomes y, and y becomes x.
        R = trimesh.transformations.rotation_matrix(
            angle=np.radians(-90),
            direction=[0, 0, 1],
            point=[0, 0, 0]
        )
        mesh.apply_transform(R)
        #mesh.compute_vertex_normals()
        mesh.export(mesh_path)
        mesh_metadata = {}
        mesh_metadata["x"] = y
        mesh_metadata["y"] = x
        mesh_metadata["tile_size_x"] = self.tile_size
        mesh_metadata["tile_size_y"] = self.tile_size
        self.metadata["terrain"][mesh_path] = mesh_metadata

    def generateTerrain(self):
        for y in np.arange(self.metadata["bounds"][1], self.metadata["bounds"][3], self.tile_size):
            for x in np.arange(self.metadata["bounds"][0], self.metadata["bounds"][2], self.tile_size):
                self.generateTerrainTile(x,y)

    def saveMetadata(self):
        with open(self.metadata_path, "w") as f:
            json.dump(self.metadata, f, indent=4)