import unreal
import sys
import os
import json

dataset_path = sys.argv[1]
manifest_path = os.path.join(dataset_path, "metadata.json")
asset_type = sys.argv[2]
asset_names = sys.argv[3:]

with open(manifest_path, "r") as f:
    manifest_data = json.load(f)

def importAssets(manifest_data, asset_type, asset_names):
    asset_type_data = manifest_data[asset_type]
    tasks = []
    for asset_name in asset_names:
        fbx_path = os.path.join(dataset_path, asset_type_data[asset_name]["fbx_path"])
        dest_name = os.path.basename(asset_type_data[asset_name]["unreal_path"])
        dest_path = os.path.dirname(asset_type_data[asset_name]["unreal_path"])
        original_basename = os.path.splitext(os.path.basename(fbx_path))[0]
        task = unreal.AssetImportTask()
        task.filename = fbx_path
        task.destination_name = dest_name
        task.destination_path = dest_path
        task.automated = True
        task.save = True
        tasks.append(task)
    
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
    
    for asset_name in asset_names:
        full_unreal_path = asset_type_data[asset_name]["unreal_path"]
        dest_name = os.path.basename(asset_type_data[asset_name]["unreal_path"])
        dest_path = os.path.dirname(asset_type_data[asset_name]["unreal_path"])
        material_path = asset_type_data[asset_name]["material"]
        original_path = f"{dest_path}/{dest_name}_{dest_name}"
        unreal.EditorAssetLibrary.rename_asset(original_path, full_unreal_path)
        mesh = unreal.EditorAssetLibrary.load_asset(full_unreal_path)
        material = unreal.load_asset(material_path)

        if isinstance(mesh, unreal.StaticMesh) or isinstance(mesh, unreal.SkeletalMesh):
            mesh.set_material(0, material)
            unreal.EditorAssetLibrary.save_loaded_asset(mesh)

importAssets(manifest_data, asset_type, asset_names)