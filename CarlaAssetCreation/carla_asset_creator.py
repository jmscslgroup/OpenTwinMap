import os
import json
import numpy as np
import hashlib
import shutil
import rtree
import subprocess
import shapely.geometry
import concurrent.futures
from .carla_asset_dataset import CarlaAssetDataset
from .opendrive_parser import OpenDriveParser

def _generateTerrainTileMethod(carla_asset_root, full_mesh_path, mesh_path, map_data, bounding_box, tile_point_interval, original_bounds):
    import trimesh
    import numpy as np
    import os

    # Limit NumPy/OpenBLAS threads
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["NUMEXPR_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    dem_data = map_data.loadDEMsFromBoundingBoxMeters(bounding_box)
    min_x_dem, min_y_dem, max_x_dem, max_y_dem = bounding_box
    tile_size_x = max_x_dem - min_x_dem
    tile_size_y = max_y_dem - min_y_dem

    number_of_points_on_side_x = int(tile_size_x / tile_point_interval)
    number_of_points_on_side_y = int(tile_size_y / tile_point_interval)

    #xs = np.linspace(min_x, max_x, number_of_points_on_side_x)
    #ys = np.linspace(min_y, max_y, number_of_points_on_side_y)
    xs = np.linspace(min_x_dem, max_x_dem, number_of_points_on_side_x)
    ys = np.linspace(min_y_dem, max_y_dem, number_of_points_on_side_y)
    xv, yv = np.meshgrid(xs, ys)
    zv = np.zeros_like(xv)
    for i in range(yv.shape[0]):
        for j in range(xv.shape[1]):
            zv[i][j] = map_data.minHeightAtXYMeters(dem_data, (xv[i][j], yv[i][j]))
    
    xv = xv.flatten()
    yv = yv.flatten()
    zv = zv.flatten()
    min_z_dem, max_z_dem = zv.min(), zv.max()
    xv -= min_x_dem
    yv -= min_y_dem
    zv -= min_z_dem
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
    
    # Let trimesh compute vertex normals for smooth shading
    #mesh.compute_vertex_normals()
    mesh.export(full_mesh_path)

    min_x, min_y, min_z = min_x_dem - original_bounds[0], min_y_dem - original_bounds[1], min_z_dem - original_bounds[2]
    max_x, max_y, max_z = max_x_dem - original_bounds[0], max_y_dem - original_bounds[1], max_z_dem - original_bounds[2]
    mesh_metadata = {}
    mesh_metadata["full_mesh_path"] = full_mesh_path
    mesh_metadata["min_y"] = min_x
    mesh_metadata["min_x"] = min_y
    mesh_metadata["max_y"] = max_x
    mesh_metadata["max_x"] = max_y
    mesh_metadata["min_z"] = min_z
    mesh_metadata["max_z"] = max_z
    mesh_metadata["tile_point_interval"] = tile_point_interval
    mesh_metadata["obj_path"] = mesh_path
    mesh_metadata["fbx_path"] = mesh_metadata["obj_path"].replace(".obj", ".fbx")
    mesh_metadata["name"] = os.path.splitext(os.path.basename(mesh_path))[0]
    mesh_metadata["unreal_path"] = f'{carla_asset_root}/{mesh_metadata["name"]}'
    mesh_metadata["material"] = "/Game/Carla/Static/GenericMaterials/Ground/MI_LargeLandscape_Grass.MI_LargeLandscape_Grass"
    return mesh_metadata

def _convertObjToFbxMethod(obj_path, fbx_path):
    convert_script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "./blender_obj_to_fbx.py"))
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

def _generateRoadMeshMethod(carla_asset_root, full_mesh_path, mesh_path, road, original_bounds, max_step=0.1, thickness = 0.5):
    import math
    import trimesh
    import numpy as np
    import os

    # Limit NumPy/OpenBLAS threads
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["NUMEXPR_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    """Generate a trimesh mesh for this road.

    The mesh is built by sampling points along the road reference line,
    computing the left and right offsets based on lane widths, and extruding
    those edges downwards by `thickness`.  Triangles are generated for
    the top surface, bottom surface, and the sides and ends of the road.

    Args:
        thickness: thickness (depth) of the road in metres.  The road
            surface sits at z=0 and the bottom face sits at z=-thickness.

    Returns:
        A trimesh.Trimesh object containing the vertices, faces and UVs.
    """
    # Sample along reference line
    ref_samples = road.sampleReferenceLine(max_step=max_step)
    num = len(ref_samples)
    vertices = []
    faces = []
    uvs = []

    # Precompute left/right widths for each sample
    widths_lr = []
    elevations = []
    for s, x, y, phi in ref_samples:
        ls = road.laneSectionAt(s)
        lw, rw = ls.widthAt(s)
        widths_lr.append((lw, rw))
        elevations.append(road.elevationAt(s) - original_bounds[2])

    # Build vertices and UVs: for each sample create four vertices (top left,
    # top right, bottom left, bottom right).  We'll normalise u by road
    # length and v by width or depth accordingly.
    for idx, ((s, x, y, phi), (lw, rw), z_elev) in enumerate(zip(ref_samples, widths_lr, elevations)):
        # unit normal (perpendicular to heading)
        n_x = -math.sin(phi)
        n_y = math.cos(phi)
        # Top surface points
        left_x = x + n_x * lw
        left_y = y + n_y * lw
        right_x = x - n_x * rw
        right_y = y - n_y * rw
        # Superelevation angle
        sup = road.superElevationAt(s)
        # Height offsets due to sup: z_offset = tan(sup) * t (t is positive to left, negative to right)
        z_offset_left = math.tan(sup) * (lw)
        z_offset_right = math.tan(sup) * (-rw)
        # Base top z-values
        top_z_left = z_elev + z_offset_left
        top_z_right = z_elev + z_offset_right
        # Compute a local normal (plane normal) rotated by sup around road direction
        dx = math.cos(phi)
        dy = math.sin(phi)
        # Rotated normal (not normalised). See Rodrigues' rotation formula in analysis.
        # vertical vector = (0,0,1). cross(dir, vertical) = (dy, -dx, 0)
        rn_x = dy * math.sin(sup)
        rn_y = -dx * math.sin(sup)
        rn_z = math.cos(sup)
        # Normalize the rotated normal
        norm = math.sqrt(rn_x * rn_x + rn_y * rn_y + rn_z * rn_z)
        if norm > 0:
            rn_x /= norm
            rn_y /= norm
            rn_z /= norm
        # Bottom vertices: extrude along negative normal
        bottom_z_left = top_z_left - thickness * rn_z
        bottom_z_right = top_z_right - thickness * rn_z
        bottom_x_left = left_x - thickness * rn_x
        bottom_y_left = left_y - thickness * rn_y
        bottom_x_right = right_x - thickness * rn_x
        bottom_y_right = right_y - thickness * rn_y
        # Normalised coordinate along length
        u_coord = s / road.road_length if road.road_length > 0 else 0.0
        # Add vertices: order matters for indexing later
        v_left_top = [left_x, left_y, top_z_left]
        v_right_top = [right_x, right_y, top_z_right]
        v_left_bottom = [left_x, left_y, bottom_z_left]
        v_right_bottom = [right_x, right_y, bottom_z_right]
        vertices.extend([v_left_top, v_right_top, v_left_bottom, v_right_bottom])
        # UVs for these four vertices
        # Top surface: v=0 at left, v=1 at right
        uvs.extend([
            [u_coord, 0.0],  # left top
            [u_coord, 1.0],  # right top
            [u_coord, 0.0],  # left bottom uses same v as top for simplicity
            [u_coord, 1.0],  # right bottom
        ])

    # Build faces
    # For each segment between consecutive samples we generate faces
    for i in range(num - 1):
        # base index for sample i
        idx0 = i * 4
        idx1 = (i + 1) * 4
        # Indices of vertices
        lt0, rt0, lb0, rb0 = idx0, idx0 + 1, idx0 + 2, idx0 + 3
        lt1, rt1, lb1, rb1 = idx1, idx1 + 1, idx1 + 2, idx1 + 3
        # Top surface (two triangles)
        faces.append([lt0, lt1, rt1])
        faces.append([lt0, rt1, rt0])
        # Bottom surface (note: reverse winding for correct normals)
        faces.append([lb0, rb0, rb1])
        faces.append([lb0, rb1, lb1])
        # Left side
        faces.append([lb0, lb1, lt1])
        faces.append([lb0, lt1, lt0])
        # Right side
        faces.append([rt0, rt1, rb1])
        faces.append([rt0, rb1, rb0])
    # Caps (start and end)
    # Start cap: first sample index 0-3
    lt0, rt0, lb0, rb0 = 0, 1, 2, 3
    # Two triangles
    faces.append([lt0, rt0, rb0])
    faces.append([lt0, rb0, lb0])
    # End cap: last sample indices
    lt1, rt1, lb1, rb1 = (num - 1) * 4, (num - 1) * 4 + 1, (num - 1) * 4 + 2, (num - 1) * 4 + 3
    faces.append([lt1, lb1, rb1])
    faces.append([lt1, rb1, rt1])

    vertices_np = np.array(vertices)
    faces_np = np.array(faces)
    uvs_np = np.array(uvs)
    min_x, min_y, min_z = vertices_np.min(axis=0)
    max_x, max_y, max_z = vertices_np.max(axis=0)
    vertices_np -= np.array([min_x, min_y, min_z])
    #min_x, min_y, min_z = min_x - original_bounds[0], min_y - original_bounds[1], min_z - original_bounds[2]
    #max_x, max_y, max_z = max_x - original_bounds[0], max_y - original_bounds[1], max_z - original_bounds[2]
    # Create the mesh
    mesh = trimesh.Trimesh(vertices=vertices_np, faces=faces_np, process=False)
    mesh.visual.uv = uvs
    # Let trimesh compute vertex normals for smooth shading
    #mesh.compute_vertex_normals()
    mesh.export(full_mesh_path)

    mesh_metadata = {}
    mesh_metadata["full_mesh_path"] = full_mesh_path
    mesh_metadata["min_y"] = min_x
    mesh_metadata["min_x"] = min_y
    mesh_metadata["max_y"] = max_x
    mesh_metadata["max_x"] = max_y
    mesh_metadata["min_z"] = min_z
    mesh_metadata["max_z"] = max_z
    mesh_metadata["road_data"] = road.toDict()
    mesh_metadata["obj_path"] = mesh_path
    mesh_metadata["fbx_path"] = mesh_metadata["obj_path"].replace(".obj", ".fbx")
    mesh_metadata["name"] = os.path.splitext(os.path.basename(mesh_path))[0]
    mesh_metadata["unreal_path"] = f'{carla_asset_root}/{mesh_metadata["name"]}'
    mesh_metadata["material"] = "/Game/Carla/Static/GenericMaterials/RoadPainterMaterials/MI_Road_01.MI_Road_01"
    return mesh_metadata

def _generateMergedRoadMethod(carla_asset_root, group, name, full_mesh_path, mesh_path):
    import numpy as np
    import trimesh
    import os

    # Limit NumPy/OpenBLAS threads
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["NUMEXPR_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    min_y, min_x, min_z = min([group[group_key]["min_y"] for group_key in group]), min([group[group_key]["min_x"] for group_key in group]), min([group[group_key]["min_z"] for group_key in group])
    max_y, max_x, max_z = max([group[group_key]["max_y"] for group_key in group]), max([group[group_key]["max_x"] for group_key in group]), max([group[group_key]["max_z"] for group_key in group])

    meshes = []
    for group_key in group:
        group_entry = group[group_key]
        current_mesh = trimesh.load(group_entry["full_mesh_path"], force="mesh")
        min_y_diff, min_x_diff, min_z_diff = group_entry["min_y"] - min_y, group_entry["min_x"] - min_x, group_entry["min_z"] - min_z
        # Define the translation vector (x, y, z)
        translation_vector = np.array([min_y_diff, min_x_diff, min_z_diff])  # Example: move 5 units in x, -2 units in z

        # Create the translation matrix
        translation_matrix = trimesh.transformations.translation_matrix(translation_vector)

        # Apply the transformation to the mesh
        current_mesh.apply_transform(translation_matrix)
        meshes.append(current_mesh)
    final_mesh = trimesh.util.concatenate(meshes)
    final_mesh.export(full_mesh_path)
    mesh_metadata = {}
    mesh_metadata["full_mesh_path"] = full_mesh_path
    mesh_metadata["min_y"] = min_y
    mesh_metadata["min_x"] = min_x
    mesh_metadata["max_y"] = max_y
    mesh_metadata["max_x"] = max_x
    mesh_metadata["min_z"] = min_z
    mesh_metadata["max_z"] = max_z
    mesh_metadata["road_data"] = list(group.keys())
    mesh_metadata["obj_path"] = mesh_path
    mesh_metadata["fbx_path"] = mesh_metadata["obj_path"].replace(".obj", ".fbx")
    mesh_metadata["name"] = name
    mesh_metadata["unreal_path"] = f'{carla_asset_root}/{name}'
    mesh_metadata["material"] = "/Game/Carla/Static/GenericMaterials/RoadPainterMaterials/MI_Road_01.MI_Road_01"
    return mesh_metadata

class CarlaAssetCreator:
    opendrive_data = None
    map_data = None
    cooked_path = None
    metadata = {}
    dem_data = None
    tile_size = 500.0
    tile_point_interval = 0.3048*5 # 2 ft resolution
    feet_to_meters = 0.3048

    def __init__(self, map_data, cooked_path):
        self.metadata["map_name"] = "I24"
        self.metadata["carla_asset_root"] = f'/Game/CarlaIngestion/{self.metadata["map_name"]}'
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
        self.metadata["asset_types"] = ["terrain", "roads", "merged_roads"]
        self.metadata["terrain"] = {}
        self.metadata["roads"] = {}
        self.metadata["merged_roads"] = {}
        self.copyXODR()
        self.opendrive_data = OpenDriveParser(self.cooked_dataset.getFullPath(self.cooked_dataset.xodr_path))
        self.opendrive_data.parseOpenDriveFile()

    def copyXODR(self):
        shutil.copy(self.map_data.getXODRPath(), self.cooked_dataset.getFullPath(self.cooked_dataset.xodr_path))
        self.metadata["xodr_path"] = self.cooked_dataset.xodr_path

    def generateTerrainPath(self, x, y):
        return os.path.join(self.cooked_dataset.terrain_mesh_path, f"{int(x*1000)}_{int(y*1000)}.obj")

    def generateRoadPath(self, road):
        return os.path.join(self.cooked_dataset.roads_mesh_path, f"{int(road.id)}.obj")

    def generateMergedRoadName(self, group):
        group_list = sorted(list(group))
        return hashlib.sha256(" ".join([str(group_entry) for group_entry in group_list]).encode()).hexdigest()[:20]

    def generateMergedRoadPath(self, group):
        name = self.generateMergedRoadName(group)
        return os.path.join(self.cooked_dataset.merged_roads_mesh_path, f"{name}.obj")

    def getTerrainTileBoundingBox(self, x, y):
        min_x, min_y = x, y
        max_x, max_y = min(x + self.tile_size, self.metadata["original_bounds"][3]), min(y + self.tile_size, self.metadata["original_bounds"][4])
        return np.array([min_x, min_y, max_x, max_y])

    def generateTerrain(self, n_jobs=100):
        terrain_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.terrain_path)
        os.makedirs(terrain_folder, exist_ok=True)
        mesh_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.terrain_mesh_path)
        os.makedirs(mesh_folder, exist_ok=True)

        terrain_asset_root = f'{self.metadata["carla_asset_root"]}/terrain'

        jobs = []
        for y in np.arange(self.metadata["original_bounds"][1], self.metadata["original_bounds"][4], self.tile_size):
            for x in np.arange(self.metadata["original_bounds"][0], self.metadata["original_bounds"][3], self.tile_size):
                tile_bounding_box = self.getTerrainTileBoundingBox(x, y)
                mesh_path = self.generateTerrainPath(x,y)
                full_mesh_path = self.cooked_dataset.getFullPath(mesh_path)
                jobs.append([terrain_asset_root, full_mesh_path, mesh_path, self.map_data, tile_bounding_box, self.tile_point_interval, self.metadata["original_bounds"]])

        if n_jobs == 1:
            for terrain_asset_root, full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval, original_bounds in jobs:
                mesh_metadata = _generateTerrainTileMethod(terrain_asset_root, full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval, original_bounds)
                self.metadata["terrain"][mesh_metadata["name"]] = mesh_metadata
                print(mesh_metadata["name"])
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_generateTerrainTileMethod, terrain_asset_root, full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval, original_bounds) for terrain_asset_root, full_mesh_path, mesh_path, map_data, tile_bounding_box, tile_point_interval, original_bounds in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    mesh_metadata = future.result()
                    self.metadata["terrain"][mesh_metadata["name"]] = mesh_metadata
                    print(mesh_metadata["name"])

    def generateRoads(self, n_jobs=100):
        roads_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.roads_path)
        os.makedirs(roads_folder, exist_ok=True)
        mesh_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.roads_mesh_path)
        os.makedirs(mesh_folder, exist_ok=True)

        road_carla_root = f'{self.metadata["carla_asset_root"]}/roads'

        jobs = []
        for road_id in self.opendrive_data.roads:
            road = self.opendrive_data.roads[road_id]
            mesh_path = self.generateRoadPath(road)
            full_mesh_path = self.cooked_dataset.getFullPath(mesh_path)
            jobs.append([road_carla_root, full_mesh_path, mesh_path, road, self.metadata["original_bounds"]])
        
        if n_jobs == 1:
            for road_carla_root, full_mesh_path, mesh_path, road, original_bounds in jobs:
                mesh_metadata = _generateRoadMeshMethod(road_carla_root, full_mesh_path, mesh_path, road, original_bounds)
                self.metadata["roads"][mesh_metadata["name"]] = mesh_metadata
                print(mesh_metadata["name"])
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_generateRoadMeshMethod, road_carla_root, full_mesh_path, mesh_path, road, original_bounds) for road_carla_root, full_mesh_path, mesh_path, road, original_bounds in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    mesh_metadata = future.result()
                    self.metadata["roads"][mesh_metadata["name"]] = mesh_metadata
                    print(mesh_metadata["name"])

    def createMergedRoadsGroups(self, group_size=500):
        groups = []

        road_keys = list(self.metadata["roads"].keys())
        # Step 2: Build R-tree index
        index = rtree.index.Index()
        bounding_boxes = {}
        for i, road_name in enumerate(road_keys):
            road_data = self.metadata["roads"][road_name]
            bounds = (road_data["min_y"], road_data["min_x"], road_data["max_y"], road_data["max_x"])
            index.insert(i, bounds)
            bounding_boxes[i] = bounds

        # Step 3: Group nearby meshes (by intersecting 2D AABBs)
        visited = set()

        for i in bounding_boxes:
            if i in visited:
                continue
            group = set()
            to_visit = [i]
            while to_visit:
                if len(group) >= group_size:
                    break
                current = to_visit.pop()
                if current in visited:
                    continue
                visited.add(current)
                group.add(road_keys[current])
                # Query overlapping meshes
                neighbors = index.intersection(bounding_boxes[current])
                for n in neighbors:
                    if n not in visited:
                        to_visit.append(n)
            groups.append(group)
        return groups

    def generateMergedRoads(self, n_jobs=100):
        merged_roads_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.merged_roads_path)
        os.makedirs(merged_roads_folder, exist_ok=True)
        mesh_folder = self.cooked_dataset.getFullPath(self.cooked_dataset.merged_roads_mesh_path)
        os.makedirs(mesh_folder, exist_ok=True)

        road_carla_root = f'{self.metadata["carla_asset_root"]}/merged_roads'
        groups = self.createMergedRoadsGroups()
        print(len(groups))
        jobs = []
        for group in groups:
            name = self.generateMergedRoadName(group)
            group_metadata = {}
            for group_entry in group:
                group_metadata[group_entry] = self.metadata["roads"][group_entry]
            mesh_path = self.generateMergedRoadPath(group)
            full_mesh_path = self.cooked_dataset.getFullPath(mesh_path)
            jobs.append([road_carla_root, group_metadata, name, full_mesh_path, mesh_path])
        
        if n_jobs == 1:
            for carla_root, group, name, full_mesh_path, mesh_path in jobs:
                mesh_metadata = _generateMergedRoadMethod(carla_root, group, name, full_mesh_path, mesh_path)
                self.metadata["merged_roads"][mesh_metadata["name"]] = mesh_metadata
                print(group, name, new_path)
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_generateMergedRoadMethod, carla_root, group, name, full_mesh_path, mesh_path) for carla_root, group, name, full_mesh_path, mesh_path in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    mesh_metadata = future.result()
                    self.metadata["merged_roads"][mesh_metadata["name"]] = mesh_metadata

    def convertAssetTypeFromObjToFbx(self, asset_type, n_jobs=48):
        jobs = []
        for k in self.metadata[asset_type]:
            mesh_metadata = self.metadata[asset_type][k]
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
        
    def convertTerrainFromObjToFbx(self, n_jobs=48):
        self.convertAssetTypeFromObjToFbx("terrain", n_jobs)

    def convertRoadsFromObjToFbx(self, n_jobs=48):
        self.convertAssetTypeFromObjToFbx("roads", n_jobs)

    def convertMergedRoadsFromObjToFbx(self, n_jobs=48):
        self.convertAssetTypeFromObjToFbx("merged_roads", n_jobs)

    def saveMetadata(self):
        with open(self.cooked_dataset.metadata_path, "w") as f:
            json.dump(self.metadata, f, indent=4)