import os
import json
import numpy as np
import shutil
import subprocess
import concurrent.futures
from carla_asset_dataset import CarlaAssetDataset

def _generateTerrainTileMethod(full_mesh_path, mesh_path, map_data, bounding_box, tile_point_interval):
    import trimesh
    import numpy as np
    import os

    # Limit NumPy/OpenBLAS threads
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["NUMEXPR_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    dem_data = map_data.loadDEMsFromBoundingBoxMeters(bounding_box)
    min_x, min_y, max_x, max_y = bounding_box
    tile_size_x = max_x - min_x
    tile_size_y = max_y - min_y

    number_of_points_on_side_x = int(tile_size_x / tile_point_interval)
    number_of_points_on_side_y = int(tile_size_y / tile_point_interval)

    #xs = np.linspace(min_x, max_x, number_of_points_on_side_x)
    #ys = np.linspace(min_y, max_y, number_of_points_on_side_y)
    xs = np.linspace(min_x, max_x, number_of_points_on_side_x)
    ys = np.linspace(min_y, max_y, number_of_points_on_side_y)
    xv, yv = np.meshgrid(xs, ys)
    zv = np.zeros_like(xv)
    bottom_left_corner_height = map_data.minHeightAtXYMeters(dem_data, (min_x, min_y))
    for i in range(yv.shape[0]):
        for j in range(xv.shape[1]):
            zv[i][j] = map_data.minHeightAtXYMeters(dem_data, (xv[i][j], yv[i][j]))
    
    xv = xv.flatten()
    yv = yv.flatten()
    zv = zv.flatten()
    min_z, max_z = zv.min(), zv.max()
    xv -= min_x
    yv -= min_y
    zv -= min_z
    vertices = np.stack([xv, yv, zv], axis=1)

    faces = []
    for i in range(number_of_points_on_side_y - 1):
        for j in range(number_of_points_on_side_x - 1):
            i0 = i * number_of_points_on_side_x + j
            i1 = i0 + 1
            i2 = i0 + number_of_points_on_side_x
            i3 = i2 + 1
            # Two triangles per grid square
            faces.append([i0, i1, i2])
            faces.append([i1, i3, i2])
    faces = np.array(faces)

    uv_x = xv / tile_size_x
    uv_y = yv / tile_size_y
    uvs = np.stack([uv_x, uv_y], axis=1)

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.visual.uv = uvs
    # Remember, in Unreal, x becomes y, and y becomes x.
    '''
    R = trimesh.transformations.rotation_matrix(
        angle=np.radians(-90),
        direction=[0, 0, 1],
        point=[0, 0, 0]
    )
    if not np.all(np.isfinite(mesh.vertices)):
        raise ValueError("Vertices contain NaNs or Infs")
    mesh.apply_transform(R)
    '''
    
    #mesh.compute_vertex_normals()
    mesh.export(full_mesh_path)

    mesh_metadata = {}
    mesh_metadata["min_y"] = min_x
    mesh_metadata["min_x"] = min_y
    mesh_metadata["max_y"] = max_x
    mesh_metadata["max_x"] = max_y
    mesh_metadata["min_z"] = min_z
    mesh_metadata["max_z"] = max_z
    mesh_metadata["tile_point_interval"] = tile_point_interval
    mesh_metadata["obj_path"] = mesh_path
    mesh_metadata["fbx_path"] = mesh_metadata["obj_path"].replace(".obj", ".fbx")
    mesh_metadata["material"] = "/Game/Carla/Static/GenericMaterials/Ground/MI_LargeLandscape_Grass.MI_LargeLandscape_Grass"
    return mesh_metadata

def _convertObjToFbxMethod(obj_path, fbx_path):
    convert_script_path = "./blender_obj_to_fbx.py"
    try:
        result = subprocess.run(
            ["blender", "--background", "--python", convert_script_path, "--", obj_path, fbx_path],
            capture_output=True,
            text=True,
            check=True
        )
        print(result)
        return obj_path, fbx_path
    except subprocess.CalledProcessError as e:
        raise e

class CarlaAssetImporter:
    map_data = None
    cooked_path = None
    metadata = {}
    dem_data = None
    tile_size = 500.0
    tile_point_interval = 0.3048*5 # 2 ft resolution
    feet_to_meters = 0.3048

    def __init__(self, map_data, cooked_path):
        self.map_data = map_data
        self.cooked_path = cooked_path
        self.cooked_dataset = CarlaAssetDataset(self.cooked_path, initialized=False)
        os.makedirs(self.cooked_path, exist_ok=True)
        map_data_bounds = self.map_data.getBoundsInMeters().tolist()
        self.metadata["original_bounds"] = self.map_data.getBoundsInMeters().tolist()
        dem_data_tmp = self.map_data.loadDEMs(self.map_data.getAllTiles())
        min_z =  dem_data_tmp.get_min() * self.feet_to_meters
        max_z = dem_data_tmp.get_max() * self.feet_to_meters
        self.metadata["original_bounds"] = [map_data_bounds[0], map_data_bounds[1], min_z, map_data_bounds[2], map_data_bounds[3], max_z]
        self.metadata["carla_bounds"] = [map_data_bounds[1], map_data_bounds[0], min_z, map_data_bounds[3], map_data_bounds[2], max_z]
        del dem_data_tmp
        self.metadata["terrain"] = {}

    def copyXODR(self):
        shutil.copy(self.map_data.getXODRPath(), self.cooked_dataset.getFullPath(self.cooked_dataset.xodr_path))
        self.metadata["xodr_path"] = self.cooked_dataset.xodr_path

    def generateTerrainPath(self, x, y):
        return os.path.join(self.cooked_dataset.terrain_mesh_path, f"{int(x*1000)}_{int(y*1000)}.obj")

    def getTerrainTileBoundingBox(self, x, y):
        min_x, min_y = x, y
        max_x, max_y = min(x + self.tile_size, self.metadata["original_bounds"][3]), min(y + self.tile_size, self.metadata["original_bounds"][4])
        return np.array([min_x, min_y, max_x, max_y])

    def generateTerrainTile(self, x, y):
        mesh_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.terrain_mesh_path)
        os.makedirs(mesh_folder, exist_ok=True)
        tile_bounding_box = self.getTerrainTileBoundingBox(x, y)
        mesh_path = self.generateTerrainPath(x,y)
        full_mesh_path = self.cooked_dataset.getFullPath(mesh_path)
        mesh_metadata = _generateTerrainTileMethod(full_mesh_path, mesh_path, self.map_data, tile_bounding_box, self.tile_point_interval)
        self.metadata["terrain"][mesh_path] = mesh_metadata

    def generateTerrain(self, n_jobs=48):
        terrain_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.terrain_path)
        os.makedirs(terrain_folder, exist_ok=True)
        mesh_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.terrain_mesh_path)
        os.makedirs(mesh_folder, exist_ok=True)

        jobs = []
        for y in np.arange(self.metadata["original_bounds"][1], self.metadata["original_bounds"][4], self.tile_size):
            for x in np.arange(self.metadata["original_bounds"][0], self.metadata["original_bounds"][3], self.tile_size):
                tile_bounding_box = self.getTerrainTileBoundingBox(x, y)
                mesh_path = self.generateTerrainPath(x,y)
                full_mesh_path = self.cooked_dataset.getFullPath(mesh_path)
                jobs.append([full_mesh_path, mesh_path, self.map_data, tile_bounding_box, self.tile_point_interval])

        if n_jobs == 1:
            for full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval in jobs:
                mesh_metadata = _generateTerrainTileMethod(full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval)
                self.metadata["terrain"][mesh_metadata["obj_path"]] = mesh_metadata
                print(mesh_path)
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_generateTerrainTileMethod, full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval) for full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    mesh_metadata = future.result()
                    self.metadata["terrain"][mesh_metadata["obj_path"]] = mesh_metadata
                    print(mesh_metadata["obj_path"])

    def convertTerrainFromObjToFbx(self, n_jobs=48):
        jobs = []
        for k in self.metadata["terrain"]:
            mesh_metadata = self.metadata["terrain"][k]
            jobs.append([self.cooked_dataset.getFullPath(mesh_metadata["obj_path"]), self.cooked_dataset.getFullPath(mesh_metadata["fbx_path"])])
        
        if n_jobs == 1:
            for obj_path, fbx_path in jobs:
                _convertObjToFbxMethod(obj_path, fbx_path)
                print(obj_path, fbx_path)
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_convertObjToFbxMethod, obj_path, fbx_path) for obj_path, fbx_path in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    print(future.result())

    def saveMetadata(self):
        with open(self.cooked_dataset.getFullPath(self.cooked_dataset.metadata_path), "w") as f:
            json.dump(self.metadata, f, indent=4)