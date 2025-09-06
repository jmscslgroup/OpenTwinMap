import unreal
import sys


def delete_assets_in_folder(content_path):
    if not unreal.EditorAssetLibrary.does_directory_exist(content_path):
        raise Exception("Does not exist!")
        return

    unreal.EditorAssetLibrary.delete_directory(content_path)


dest_root = sys.argv[-1]
delete_assets_in_folder(dest_root)
