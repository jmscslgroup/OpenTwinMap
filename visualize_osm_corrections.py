import open3d
import numpy
from DEM_python import DEM
import correct_osm_ways

if __name__ == "__main__":
    #Medium size
    minlat=36.06033995
    minlon=-86.70032724
    maxlat=36.07699495
    maxlon=-86.67001764

    handler = correct_osm_ways.WayNodeCollector(minlon, minlat)

    # Load OSM
    print("Loading OSM")
    #osm_file = "148110_full_corrected_2022.osm"
    #target_osm_file = "148110_full_corrected_2022_lidar.osm"
    osm_file = "SubsetSelection/osm_subset_corrected_lidar.osm"
    handler.apply_file(osm_file)

    print("Reading DEM")
    dem_data = DEM.from_csv("SubsetSelection/dem_subset.csv", 2.0, -999999)
    #dem_data = DEM("148110.asc", 2.0, -999999)

    print("Loading lidar....")
    #lidar_path = "148110.pcd"
    lidar_path = "SubsetSelection/lidar_subset.pcd"
    pcd_points = correct_osm_ways.preprocessInputLidar(lidar_path, False)

    print("Lidar visualization....")
    ways, ways_info = correct_osm_ways.convertWaysToLidar(handler, dem_data, pcd_points)
    correct_osm_ways.visualizeWaysWithLidar(ways, ways_info, pcd_points)

