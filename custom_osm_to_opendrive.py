import xml.etree.ElementTree as ET
from OpenDrive import osm_to_opendrive
from tdot_subset import TDOTSubset

map_data = TDOTSubset("./SubsetSelection/")
converter = osm_to_opendrive.OSMToOpenDrive(map_data)
converter.readOSM()
opendrive_data = converter.convertToOpenDrive()
opendrive_data.writeFile(map_data.getXODRPath())