from DataSources.tdot.tdot_subset import TDOTSubset
import open3d

map_data = TDOTSubset("./SubsetSelection/")
laz_data = map_data.loadLAZs(map_data.getAllTiles(), voxel_size=1.0)
print(laz_data)

open3d.visualization.draw_geometries(laz_data, zoom=0.25)