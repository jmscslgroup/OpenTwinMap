import open3d
import numpy
from DEM_python import DEM
from tdot_subset import TDOTSubset
import correct_osm_ways

if __name__ == "__main__":
    tdot_subset = TDOTSubset("SubsetSelection/", "osm_subset_corrected.osm")
    tdot_subset.processOSM()
    correct_osm_ways.visualizeWaysWithLidar(tdot_subset)