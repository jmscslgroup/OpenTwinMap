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

    @staticmethod
    def _generateTerrainTileMethod(mesh_path, map_data, dem_data, bounding_box, tile_point_interval):
        min_x, min_y, max_x, max_y = bounding_box
        tile_size_x = max_x - min_x
        tile_size_y = max_y - min_y

        number_of_points_on_side_x = int(tile_size_x / tile_point_interval)
        number_of_points_on_side_y = int(tile_size_y / tile_point_interval)

        xs = np.linspace(min_x, max_x, number_of_points_on_side_x)
        ys = np.linspace(min_y, max_y, number_of_points_on_side_y)
        xv, yv = np.meshgrid(xs, ys)
        zv = np.zeros_like(xv)
        for i in range(yv.shape[0]):
            for j in range(xv.shape[1]):
                zv[i][j] = map_data.minHeightAtXYMeters(dem_data, (xv[i][j], yv[i][j]))
        xv = xv.flatten()
        yv = yv.flatten()
        zv = zv.flatten()
        vertices = np.stack([xv, yv, zv], axis=1)

        faces = []
        for i in range(number_of_points_on_side_y - 1):
            for j in range(number_of_points_on_side_x - 1):
                i0 = i * number_of_points_on_side_x + j
                i1 = i0 + 1
                i2 = i0 + number_of_points_on_side_x
                i3 = i2 + 1
                # Two triangles per grid square
                faces.append([i0, i2, i1])
                faces.append([i1, i2, i3])
        faces = np.array(faces)

        uv_x = xv / tile_size_x
        uv_y = yv / tile_size_y
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
        mesh_metadata["min_x"] = min_x
        mesh_metadata["min_y"] = min_y
        mesh_metadata["max_x"] = max_x
        mesh_metadata["max_y"] = max_y
        return mesh_metadata

    def getTerrainTileBoundingBox(self, x, y):
        min_x, min_y = x, y
        max_x, max_y = min(x + self.tile_size, self.metadata["bounds"][2]), min(y + self.tile_size, self.metadata["bounds"][3])
        return np.array([min_x, min_y, max_x, max_y])

    def generateTerrainTile(self, x, y):
        mesh_folder = os.path.join(self.terrain_path, "meshes/")
        os.makedirs(mesh_folder, exist_ok=True)
        tile_bounding_box = self.getTerrainTileBoundingBox(x, y)
        dem_data = self.map_data.loadDEMsFromBoundingBoxMeters(tile_bounding_box)
        mesh_path = self.generateTerrainPath(x,y)
        mesh_metadata = self._generateTerrainTileMethod(mesh_path, self.map_data, dem_data, tile_bounding_box, self.tile_point_interval)
        self.metadata["terrain"][mesh_path] = mesh_metadata

    def generateTerrain(self):
        for y in np.arange(self.metadata["bounds"][1], self.metadata["bounds"][3], self.tile_size):
            for x in np.arange(self.metadata["bounds"][0], self.metadata["bounds"][2], self.tile_size):
                self.generateTerrainTile(x,y)
                print(y,x)

    def saveMetadata(self):
        with open(self.metadata_path, "w") as f:
            json.dump(self.metadata, f, indent=4)