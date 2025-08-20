import xml.etree.ElementTree as ET

class RoadLinkEntry:
    predecessor_or_successor = None # String "predecessor" or "successor"
    element_type = None
    element_id = None
    contact_point = None
    element_s = None
    element_dir = None

    def __init__(self, predecessor_or_successor, element_type, element_id, contact_point, element_s, element_dir):
        self.predecessor_or_successor = predecessor_or_successor
        self.element_type = element_type
        self.element_id = element_id
        self.contact_point = contact_point
        self.element_s = element_s
        self.element_dir = element_dir

    def toXML(self):
        attributes = {
            "elementType": self.element_type,
            "elementId": self.element_id 
        }
        if self.contact_point is not None:
            attributes["contactPoint"] = self.contact_point
        else:
            attributes["elementS"] = self.element_s
            attributes["elementDir"] = self.element_dir
        entry_xml = ET.Element(self.predecessor_or_successor, attributes)
        return entry_xml

"""
Link tag that resides within road classes. Specifies the road that comes before, and the road that comes after.
Ignoring neighbor, since it is a 'legacy' concept.
"""
class RoadLink:
    predecessor = None
    successor = None

    def __init__(self):
        self.predecessor = None
        self.successor = None

    def toXML(self):
        link = ET.Element("link")
        if self.predecessor is not None:
            link.append(self.predecessor.toXML())
        if self.successor is not None:
            link.append(self.successor.toXML())

class RoadType:
    s = None
    road_type = None
    country = None
    speed_max = None
    speed_unit = None

    def __init__(self, s, road_type, country, speed_max, speed_unit="m/s"):
        self.s = s
        self.road_type = road_type
        self.country = country
        self.speed_max = speed_max
        self.speed_unit = speed_unit

    def toXML(self):
        attributes = {
            "s": self.s,
            "type": self.road_type,
            "country": self.country
        }
        road_type_xml = ET.Element("type", attributes)
        if self.speed_max is not None:
            speed_attributes = {
                "max": self.speed_max,
                "unit": self.speed_unit
            }
            speed_xml = ET.Element("speed", speed_attributes)
            road_type_xml.append(speed_xml)
        return road_type_xml

class RoadGeometry:
    s = None
    x = None
    y = None
    hdg = None
    geometry_length = None
    geometry_shape = None

    def __init__(self, s, x, y, hdg, geometry_length, geometry_shape):
        self.s = s
        self.x = x
        self.y = y
        self.hdg = hdg
        self.geometry_length = geometry_length
        self.geometry_shape = geometry_shape

    def toXML(self):
        attributes = {
            "s": self.s,
            "x": self.x,
            "y": self.y,
            "hdg": self.hdg,
            "length": self.geometry_length
        }
        geometry_xml = ET.Element("planView")

class PlanView:


class Road:
    name = None
    plan_length = None
    id = None
    junction = None
    rule = None

    links = None
    types = None
    plan_view = None
    elevation_profile = None
    lanes = None
    signals = None
    objects = None

    def __init__(self, name, plan_length, id, junction, rule="RHT"):
        self.name = name
        self.plan_length = plan_length
        self.id = id
        self.junction = junction
        self.rule = rule
        
        self.links = RoadLink()
        self.types = []

    def