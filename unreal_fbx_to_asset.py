import unreal
import sys

fbx_path = sys.argv[-2]  # Absolute path to FBX
dest_path = sys.argv[-1] # Unreal content path, like '/Game/Assets/MyBatch'

task = unreal.AssetImportTask()
task.filename = fbx_path
task.destination_path = dest_path
task.automated = True
task.save = True

unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
print(f"Imported {fbx_path} to {dest_path}")