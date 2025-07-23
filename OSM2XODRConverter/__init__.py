from .osm_to_xodr_parser import OSMToXODRParser
from .xodr_writer import XODRWriter

def convert(map_data):
    parser = OSMToXODRParser(map_data)
    parser.parseAll()
    writer = XODRWriter(parser, map_data.getXODRPath())
    writer.startBasicXODRFile()
    writer.fillNormalRoads()
    writer.fillJunctionRoads()