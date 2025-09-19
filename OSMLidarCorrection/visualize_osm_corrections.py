import open3d
import numpy
from .DataSources.tdot.tdot_subset import TDOTSubset
from ..DEM_python import DEM
from .utils import WayNodeCollectorLidarCorrection, meters_to_feet, feet_to_meters
from .correct_osm_ways import OSMLidarCorrection

if __name__ == "__main__":

    dataset = TDOTSubset("SubsetSelection/")
    osm_handler = OSMLidarCorrection.loadOSM(dataset)
    osm_handler.generateImplicitWays()

    OSMLidarCorrection.visualizeWaysWithLidar(dataset, osm_handler)
