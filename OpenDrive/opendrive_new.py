import xml.etree.ElementTree as ET
from . import road
from . import junction

class OpenDriveHeader:
    # We are implementing 1.4 only.
    rev_major = 1
    rev_minor = 4
    name = None
    version = None
    date = None
    north = None
    south = None
    east = None
    west = None
    georeference = None

    def __init__(self, name, version, date, north, south, east, west, georeference):
        self.name = name
        self.version = version
        self.date = date
        self.north = north
        self.south = south
        self.east = east
        self.west = west
        self.georeference = georeference

    def toXML(self):
        header_xml = ET.Element("header", 
            {"revMajor": self.rev_major, 
             "revMinor": self.rev_minor,
             "name": self.name,
             "version": self.version,
             "date": self.date,
             "north": self.north,
             "south": self.south,
             "east": self.east,
             "west": self.west
            }
        )
        georeference_xml = ET.SubElement(header_xml, "geoReference")
        georeference_xml.text = self.georeference
        return header_xml

class OpenDrive:
    header = None
    roads = None
    junctions = None
    controllers = None # Currently doing no controllers at all!

    def __init__(self, header):
        self.header = header
        self.roads = {}
        self.junctions = []
        self.controllers = [] # Currently doing no controllers at all!

    def addRoad(self, road):
        self.roads[road.id] = road

    def addJunction(self, junction):
        self.junctions += junction

    def toXML(self):
        opendrive_xml = ET.Element("OpenDRIVE")
        opendrive_xml.append(self.header.toXML())
        for road in self.roads:
            opendrive_xml.append(self.roads[road].toXML())
        for junction in self.junctions:
            opendrive_xml.append(junction.toXML())
        return opendrive_xml