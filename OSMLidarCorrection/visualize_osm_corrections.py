modules_path = "/home/richarwa/SecondSSD/CarlaIngestion"
import sys
sys.path.append(modules_path)
import open3d
import numpy
from DEM_python import DEM
from tdot_subset import TDOTSubset
import correct_osm_ways

if __name__ == "__main__":
    tdot_subset = TDOTSubset("SubsetSelection/", "osm_subset_corrected_all_lidar_for_elevation.osm")
    tdot_subset.processOSM()
    correct_osm_ways.visualizeWaysWithLidar(tdot_subset)