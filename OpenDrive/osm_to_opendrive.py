import osmium
import pyproj
import numpy as np
from . import opendrive

def _generatePairs(items):
    return list(zip(items, items[1:]))

def _distance(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

def _giveHeading(x1,y1,x2,y2):
    assert not (x1==x2 and y1==y2), "Can't give heading without a direction"
    x = [x1,x2]
    y = [y1,y2]
    x_arr=np.array(x)-x[0]
    y_arr=np.array(y)-y[0]
    if x_arr[1] > 0:
        phi = np.arctan(y_arr[1]/x_arr[1])
    elif x_arr[1] == 0:
        if y_arr[1] > 0:
                phi = np.pi/2
        else:
                phi = -np.pi/2
    else:
        if y_arr[1] >= 0:
                phi = np.arctan(y_arr[1]/x_arr[1])+np.pi
        else:
                phi = np.arctan(y_arr[1]/x_arr[1])-np.pi
    return _getPositiveHeading(phi)

def _getPositiveHeading(hdg):
    while hdg < 0.0:
         hdg+=2.0*np.pi
    return hdg%(np.pi*2.0)

class OSMToOpenDrive(osmium.SimpleHandler):
    feet_to_meters = 0.3048
    origin = None
    bounds_meters = None
    bounds_coords = None
    proj = None
    resolution = None
    nodes = {}
    ways = {}
    junctions = {}
    opendrive_id_count = 0

    def __init__(self, dataset, resolution=0.1):
        self.dataset = dataset
        self.bounds_meters = self.dataset.getBoundsInMeters()
        self.bounds_coords = self.dataset.getBoundsInCoords()
        self.proj = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:6576", always_xy=True)
        self.resolution = resolution

    def projectToMeters(self, coordinates, fix_to_origin=True):
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        x, y = self.proj.transform(x,y)
        x = (x * self.feet_to_meters)
        y = (y * self.feet_to_meters)
        if (fix_to_origin):
            x -= self.bounds_meters[0]
            y -= self.bounds_meters[1]
        return np.array([x, y, z])

    def node(self, n):
        n_id = str(n.id)
        coordinates = [n.location.lon, n.location.lat, 0.0] # No elevation for now
        meters_coordinates = self.projectToMeters(coordinates)
        self.nodes[n_id] = {
            "nid": n_id,
            "node_data": n,
            "meters_coordinates": meters_coordinates,
            "connected_ways": [],
            "junction_id": "-1"
        }

    def way(self, w):
        w_id = str(w.id)
        if 'highway' in w.tags:
            oneway = ("oneway" in w.tags) and (w.tags["oneway"] == "yes")
            bridge = ("bridge" in w.tags) and (w.tags["bridge"] == "yes")
            lane_width_by_highway = {
                'motorway': 3.75,
                'motorway_link': 3.5,
                'primary': 3.5,
                'secondary': 3.0,
                'tertiary': 2.8,
                'residential': 2.7,
                'service': 2.5,
            }
            lane_type = w.tags['highway']
            lane_width = float(lane_width_by_highway[lane_type]) if lane_type in lane_width_by_highway else 3.75
            nodes = []
            lanes = w.tags.get('lanes')
            lane_count = 2
            if lanes is not None:
                try:
                    lane_count = int(lanes)
                except ValueError:
                    pass
            for n in w.nodes:
                n_ref = str(n.ref)
                nodes.append(n_ref)
            lane_backward = int(w.tags["lanes:backward"]) if "lanes:backward" in w.tags else int(lane_count / 2)
            lane_forward = int(w.tags["lanes:forward"]) if "lanes:forward" in w.tags else int(lane_count / 2)
            lane_backward_turn = w.tags["turn:lanes:backward"] if "turn:lanes:backward" in w.tags else ""
            lane_forward_turn = w.tags["turn:lanes:forward"] if "turn:lanes:forward" in w.tags else ""
            if oneway:
                lane_backward = 0
                lane_forward = lane_count
            else:
                if ((lane_forward + lane_backward) < lane_count):
                    lane_backward += 1
            way = {
                "lane_width": lane_width,
                "lane_count": lane_count,
                "bridge": bridge,
                "oneway": oneway,
                "lane_count_backward": lane_backward,
                "lane_count_forward": lane_forward,
                "lane_turn_backward": lane_backward_turn,
                "lane_turn_forward": lane_forward_turn,
                "nodes": nodes,
                "wid": w_id,
                "connected_ways": [],
                "predecessor_way": None,
                "successor_way": None,
                "opendrive_id": self.generateOpenDriveID()
            }
            self.ways[w_id] = way

    def readOSM(self):
        self.apply_file(self.dataset.getOSMPath())
        self.annotateNodesWithWays()
        self.detectConnectedWays()
        self.detectJunctions()

    def annotateNodesWithWays(self):
        for wid in self.ways:
            way = self.ways[wid]
            for nid in way["nodes"]:
                node = self.nodes[nid]
                node["connected_ways"].append(wid)

    def detectConnectedWays(self):
        for wid in self.ways:
            way = self.ways[wid]
            beginning_node = way["nodes"][0]
            ending_node = way["nodes"][-1]
            nodes_of_interest = [beginning_node, ending_node]
            for nid in nodes_of_interest:
                node = self.nodes[nid]
                #At least one other way!
                if len(node["connected_ways"]) > 1:
                    for entry in node["connected_ways"]:
                        if entry != wid:
                            way["connected_ways"].append(entry)
                            if nid == beginning_node:
                                way["predecessor_way"] = entry
                            elif nid == ending_node:
                                way["successor_way"] = entry

    def detectJunctions(self):
        for nid in self.nodes:
            node = self.nodes[nid]
            # Junction!
            if len(node["connected_ways"]) > 2:
                junction_id = self.generateOpenDriveID()
                junction_data = {
                    "ways": node["connected_ways"],
                    "id": junction_id,
                    "node_id": nid
                }
                self.junctions[junction_id] = junction_data
                node["junction_id"] = junction_id
    
    def generateOpenDriveHeader(self):
        geoReference = opendrive.GeoReference(proj4="<![CDATA[+proj=tmerc +lat_0={0:0.10f} +lon_0={1:0.10f} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs]]>".format(self.bounds_coords[0], self.bounds_coords[1]))
        return opendrive.Header(revMajor=1, revMinor=4, north=self.bounds_meters[3]-self.bounds_meters[1], south=0.0, east=self.bounds_meters[2]-self.bounds_meters[0], west=0.0, geoReference=geoReference)
    
    def generateOpenDriveID(self):
        new_id = self.opendrive_id_count + 1
        self.opendrive_id_count = new_id
        return str(new_id)
    
    # For now, even curves are modeled as lines. Each pair of nodes in a way constitutes a line.
    def generatePlanViewFromOSMWayData(self, way_data):
        geometries = []
        s = 0.0
        if len(way_data["nodes"]) > 1: # At least two nodes!
            # Assume each node is in succession of the previous.
            node_pairs = _generatePairs(way_data["nodes"])
            for pair in node_pairs:
                node1 = self.nodes[pair[0]]
                node2 = self.nodes[pair[1]]
                x1 = node1["meters_coordinates"][0]
                y1 = node1["meters_coordinates"][1]
                x2 = node2["meters_coordinates"][0]
                y2 = node2["meters_coordinates"][1]
                hdg = _giveHeading(x1, y1, x2, y2)
                pair_length = _distance(x1, y1, x2, y2)
                geometries.append(opendrive.Geometry(s=s, x=x1, y=y1, hdg=hdg, length=pair_length, shape=opendrive.Line()))
                s = s + pair_length
        return opendrive.PlanView(geometries=geometries), s
    
    def generateLaneOffsetsFromOSMWayData(self, way_data):
        lane_offsets = []
        if way_data["oneway"]:
            # Center will be to the lefthand side of the road completely - all lanes will be in the right section.
            lane_offsets.append(opendrive.LaneOffset(s=0.0, a=(way_data["lane_count"] / 2) * way_data["lane_width"], b=0.0, c=0.0, d=0.0))
        else:
            lane_offsets.append(opendrive.LaneOffset(s=0.0, a=0.0, b=0.0, c=0.0, d=0.0))
        return lane_offsets
    
    # Any opposite direction lanes go here.
    def generateLeftLanesFromOSMWayData(self, way_data):
        lanes = []
        lane_count = way_data["lane_count_backward"]
        lane_width = way_data["lane_width"]
        for i in range(lane_count, 0, -1):
            lane_type = "driving"
            if (i == lane_count):
                lane_type = "border"
            lanes.append(opendrive.Lane(id=i, type=lane_type, level=False, widths=[opendrive.Width(sOffset=0.0, a=lane_width, b=0.0, c=0.0, d=0.0)]))
        return opendrive.LaneGroup(lanes=lanes)
    
    def generateCenterLaneFromOSMWayData(self, way_data):
        return opendrive.LaneGroup(lanes=[opendrive.Lane(id=0)])

    def generateRightLanesFromOSMWayData(self, way_data):
        lanes = []
        lane_count = way_data["lane_count_forward"]
        lane_width = way_data["lane_width"]
        for i in range(lane_count):
            lane_id = (i + 1) * -1
            lane_type = "driving"
            if (i == (lane_count - 1)):
                lane_type = "border"
            lanes.append(opendrive.Lane(id=lane_id, type=lane_type, level=False, widths=[opendrive.Width(sOffset=0.0, a=lane_width, b=0.0, c=0.0, d=0.0)]))
        return opendrive.LaneGroup(lanes=lanes)
    
    # By default, only one lane section
    def generateLaneSectionsFromOSMWayData(self, way_data):
        lane_sections = []
        s = 0.0
        single_side = False
        left_group = self.generateLeftLanesFromOSMWayData(way_data)
        center_group = self.generateCenterLaneFromOSMWayData(way_data)
        right_group = self.generateRightLanesFromOSMWayData(way_data)

        lane_sections.append(opendrive.LaneSection(s=s, singleSide=single_side, left=left_group, center=center_group, right=right_group))
        return lane_sections
    
    def generateLanesFromOSMWayData(self, way_data):
        lane_offsets = self.generateLaneOffsetsFromOSMWayData(way_data)
        lane_sections = self.generateLaneSectionsFromOSMWayData(way_data)

        return opendrive.Lanes(laneOffsets=lane_offsets, laneSections=lane_sections)
    
    def generateRoadLinkageFromOSMWayData(self, way_data):
        # Generate predecessor
        predecessor = None
        if way_data["predecessor_way"] != None:
            predecessor_way = self.ways[way_data["predecessor_way"]]
            shared_nid = list(set(way_data["nodes"]) & set(predecessor_way["nodes"]))[0]
            shared_node = self.nodes[shared_nid]
            predecessor_type = "junction" if (shared_node["junction_id"] != "-1") else "road"
            element_id = predecessor_way["opendrive_id"] if (predecessor_type == "road") else shared_node["junction_id"]
            predecessor = opendrive.PredecessorSuccessor(elementType=predecessor_type, elementId=element_id, tag="predecessor")
            if predecessor_type == "junction":
                predecessor = None
        # Generate successor
        successor = None
        if way_data["successor_way"] != None:
            successor_way = self.ways[way_data["successor_way"]]
            shared_nid = list(set(way_data["nodes"]) & set(successor_way["nodes"]))[0]
            shared_node = self.nodes[shared_nid]
            successor_type = "junction" if (shared_node["junction_id"] != "-1") else "road"
            element_id = successor_way["opendrive_id"] if (successor_way == "road") else shared_node["junction_id"]
            successor = opendrive.PredecessorSuccessor(elementType=successor_type, elementId=element_id, tag="successor")
            if successor_type == "junction":
                successor = None
        return opendrive.Link(predecessor=predecessor, successor=successor)

    def generateRoadFromOSMWay(self, wid):
        way_data = self.ways[wid]
        id = self.ways[wid]["opendrive_id"]
        planView, road_length = self.generatePlanViewFromOSMWayData(way_data)
        lanes = self.generateLanesFromOSMWayData(way_data)
        linkage = self.generateRoadLinkageFromOSMWayData(way_data)
        return opendrive.Road(id=id, length=road_length, planView=planView, lanes=lanes, link=linkage)
    
    def generateRoadsFromOSMWays(self):
        roads = {}
        for wid in self.ways:
            road = self.generateRoadFromOSMWay(wid)
            roads[road.id] = road
        return roads

    def convertToOpenDrive(self):
        header = self.generateOpenDriveHeader()
        roads = self.generateRoadsFromOSMWays()
        return opendrive.OpenDRIVE(header=header, roads=roads)