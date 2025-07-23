import OSM2XODRConverter
from tdot_subset import TDOTSubset

map_data = TDOTSubset("./SubsetSelection/")
OSM2XODRConverter.convert(map_data)