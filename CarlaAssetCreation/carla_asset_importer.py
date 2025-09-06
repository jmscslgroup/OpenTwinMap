import os
import subprocess
import concurrent.futures
from .carla_asset_dataset import CarlaAssetDataset

def _importFbxToUnrealMethodBase(ue4_root, project_path, dataset_path, asset_type, asset_names):
    editor_cmd = os.path.join(ue4_root, "Engine/Binaries/Linux/UE4Editor-Cmd")
    script_name = os.path.abspath(os.path.join(os.path.dirname(__file__), "./unreal_fbx_to_asset.py"))
    dataset_full_path = os.path.abspath(dataset_path)
    args = [
        editor_cmd,
        project_path,
        "-run=pythonscript",
        f'-Script={script_name} {dataset_full_path} {asset_type} {" ".join(asset_names)}'
    ]
    print(args)
    return subprocess.run(args, capture_output=True, text=True, check=False)

def _importFbxToUnrealMethodGeneral(ue4_root, project_path, dataset_path, asset_type, asset_names, max_asset_amount=5000):
    asset_names_chunked = [asset_names[i:i+max_asset_amount] for i in range(0, len(asset_names), max_asset_amount)]
    for chunk in asset_names_chunked:
        _importFbxToUnrealMethodBase(ue4_root, project_path, dataset_path, asset_type, chunk)

class CarlaAssetImporter:
    cooked_path = None
    dataset = None

    def __init__(self, ue4_root, carla_path, cooked_path):
        self.ue4_root = ue4_root
        self.project_path = os.path.join(carla_path, "Unreal/CarlaUE4/CarlaUE4.uproject")
        self.cooked_path = cooked_path
        self.dataset = CarlaAssetDataset(self.cooked_path, initialized=True)

    def clearUnrealContent(self):
        carla_asset_root = self.dataset.metadata["carla_asset_root"]
        editor_cmd = os.path.join(self.ue4_root, "Engine/Binaries/Linux/UE4Editor-Cmd")
        script_name = os.path.abspath(os.path.join(os.path.dirname(__file__), "./unreal_clear_assets.py"))
        args = [
            editor_cmd,
            self.project_path,
            "-run=pythonscript",
            f'-Script={script_name} {carla_asset_root}'
        ]
        print(args)
        print(subprocess.run(args, capture_output=True, text=True, check=False))

    def convertAssetTypeFromFbxToUnreal(self, asset_type, n_jobs=48):
        total_asset_names = list(self.dataset.metadata[asset_type].keys())
        jobs = []
        total_asset_names_size = len(total_asset_names)
        total_asset_names_chunk_size = int(total_asset_names_size / n_jobs) + 1
        for i in range(n_jobs):
            asset_names_job_chunk = total_asset_names[(i*total_asset_names_chunk_size):((i + 1)*total_asset_names_chunk_size)]
            jobs.append([self.ue4_root, self.project_path, self.cooked_path, asset_type, asset_names_job_chunk])

        if n_jobs == 1:
            for ue4_root, project_path, dataset_path, asset_type, chunk in jobs:
                _importFbxToUnrealMethodGeneral(ue4_root, project_path, dataset_path, asset_type, chunk)
                print(len(chunk))
        else:
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_jobs) as executor:
                futures = [executor.submit(_importFbxToUnrealMethodGeneral, ue4_root, project_path, dataset_path, asset_type, chunk) for ue4_root, project_path, dataset_path, asset_type, chunk in jobs]
                
                for future in concurrent.futures.as_completed(futures):
                    print(future.result())

    def convertTerrainFromFbxToUnreal(self, n_jobs=1):
        self.convertAssetTypeFromFbxToUnreal("terrain", n_jobs)

    def convertRoadsFromFbxToUnreal(self, n_jobs=1):
        self.convertAssetTypeFromFbxToUnreal("merged_roads", n_jobs)