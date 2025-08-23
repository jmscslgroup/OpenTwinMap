import osmium
import pyproj
import numpy as np
import math
from scipy.optimize import curve_fit
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
    reference_line_resolution = None
    elevation_line_max_length = None
    dem_data = None
    nodes = {}
    ways = {}
    junctions = {}
    opendrive_id_count = 0

    def __init__(self, dataset, reference_line_resolution=0.1, elevation_line_max_length=25.0):
        self.dataset = dataset
        self.bounds_meters = self.dataset.getBoundsInMeters()
        self.bounds_coords = self.dataset.getBoundsInCoords()
        self.proj = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:6576", always_xy=True)
        self.reference_line_resolution = reference_line_resolution
        self.elevation_line_max_length = elevation_line_max_length
        self.dem_data = self.dataset.loadDEMs(self.dataset.getAllTiles())

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
        coordinates = [n.location.lon, n.location.lat, float(n.tags["ele"]) if "ele" in n.tags else 0.0]
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

    def isNodeInBridge(self, nid):
        node = self.nodes[nid]
        for wid in node["connected_ways"]:
            way = self.ways[wid]
            if way["bridge"]:
                return True
        return False

    def readOSM(self):
        self.apply_file(self.dataset.getCorrectedOSMPath())
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
        # Generate successor
        successor = None
        if way_data["successor_way"] != None:
            successor_way = self.ways[way_data["successor_way"]]
            shared_nid = list(set(way_data["nodes"]) & set(successor_way["nodes"]))[0]
            shared_node = self.nodes[shared_nid]
            successor_type = "junction" if (shared_node["junction_id"] != "-1") else "road"
            element_id = successor_way["opendrive_id"] if (successor_way == "road") else shared_node["junction_id"]
            successor = opendrive.PredecessorSuccessor(elementType=successor_type, elementId=element_id, tag="successor")
        return opendrive.Link(predecessor=predecessor, successor=successor)
    
    def getDEMElevationAtPoint(self, position):
        return self.dataset.minHeightAtXYMeters(self.dem_data, (position[1] + self.bounds_meters[0], position[2] + self.bounds_meters[1]))
    
    def getDEMElevationsAtPoints(self, positions):
        elevations = []
        for position in positions:
            elevations.append(self.getDEMElevationAtPoint(position))
        return np.array(elevations)
    
    def regressElevationEquationFromReferenceLine(self, positions, elevations):
        #print(positions[:, 0].min(), elevations.min(), positions[:, 0].max(), elevations.max())
        degree = len(positions) if len(positions) < 4 else 4
        s0 = positions[0][0]
        if degree == 1:
            return opendrive.Elevation(s=s0, a=elevations[0], b=0.0, c=0.0, d=0.0)
        L = positions[-1][0] - s0
        sn = ((positions[:, 0] - s0) / L)
        coeff = None
        if degree == 2:
            def model(sn, a, b):
                s = L*sn
                return a + b*s
            coeff, _ = curve_fit(model, sn, elevations)
            coeff = np.array([coeff[0], coeff[1], 0.0, 0.0])
            #print(coeff)
        elif degree == 3:
            def model(sn, a, b, c):
                s = L*sn
                return a + b*s + c*(s**2)
            coeff, _ = curve_fit(model, sn, elevations)
            coeff = np.array([coeff[0], coeff[1], coeff[2], 0.0])
            #print(coeff)
        else:
            def model(sn, a, b, c, d):
                s = L*sn
                return a + b*s + c*(s**2) + d*(s**3)
            coeff, _ = curve_fit(model, sn, elevations)
            coeff = np.array([coeff[0], coeff[1], coeff[2], coeff[3]])
            #print(coeff)
        return opendrive.Elevation(s=s0, a=coeff[0], b=coeff[1], c=coeff[2], d=coeff[3])
    
    def generateElevationSequencesFromReferenceline(self, positions, elevations):
        position_elevation_splits = []
        elevation_entries = []
        current_position_split = []
        current_elevation_split = []
        for position, elevation in zip(positions, elevations):
            position_s, position_x, position_y, position_phi = position
            if int(position_s / self.elevation_line_max_length) > len(position_elevation_splits):
                current_position_split = np.array(current_position_split)
                current_elevation_split = np.array(current_elevation_split)
                position_elevation_splits.append([current_position_split, current_elevation_split])
                current_position_split = []
                current_elevation_split = []
            current_position_split.append(position)
            current_elevation_split.append(elevation)
        if len(current_position_split) > 0:
            current_position_split = np.array(current_position_split)
            current_elevation_split = np.array(current_elevation_split)
            position_elevation_splits.append([current_position_split, current_elevation_split])
        for (position_split, elevation_split) in position_elevation_splits:
            elevation_entries.append(self.regressElevationEquationFromReferenceLine(position_split, elevation_split))
        return elevation_entries
    
    def markPositionsWithBridges(self, way_data, positions, plan_view):
        regions = []
        for i, nid in enumerate(way_data["nodes"]):
            if self.isNodeInBridge(nid):
                bridge_node = self.nodes[nid]
                bridge_node_coordinates = bridge_node["meters_coordinates"]
                bridge_s, _ = plan_view.projectXYToS(bridge_node_coordinates[0], bridge_node_coordinates[1])
                # Mark nodes behind and ahead as connected to the bridge
                behind_id = i - 1
                ahead_id = i + 1
                if (behind_id >= 0):
                    behind_nid = way_data["nodes"][behind_id]
                    behind_node = self.nodes[behind_nid]
                    behind_node_coordinates = behind_node["meters_coordinates"]
                    behind_s, _ = plan_view.projectXYToS(behind_node_coordinates[0], behind_node_coordinates[1])
                    behind_elevation = self.getDEMElevationAtPoint([None, behind_node_coordinates[0], behind_node_coordinates[1], None])
                    regions.append([behind_s, bridge_s, behind_elevation, bridge_node_coordinates[2]])
                if (ahead_id < len(way_data["nodes"])):
                    ahead_nid = way_data["nodes"][ahead_id]
                    ahead_node = self.nodes[ahead_nid]
                    ahead_node_coordinates = ahead_node["meters_coordinates"]
                    ahead_s, _ = plan_view.projectXYToS(ahead_node_coordinates[0], ahead_node_coordinates[1])
                    ahead_elevation = self.getDEMElevationAtPoint([None, ahead_node_coordinates[0], ahead_node_coordinates[1], None])
                    regions.append([bridge_s, ahead_s, bridge_node_coordinates[2], ahead_elevation])
        return regions
    
    def annotateDEMElevationsWithBridgeData(self, elevations_reference_line, way_data, positions, positions_bridges):
        new_elevations_reference_line = []
        for i, position in enumerate(positions):
            position_s = position[0]
            current_elevation = elevations_reference_line[i]
            for (bounds_min, bounds_max, elevation_min, elevation_max) in positions_bridges:
                if (bounds_min <= position_s) and (position_s <= bounds_max):
                    bounds_length = bounds_max - bounds_min
                    elevation_max_weight = ((position_s - bounds_min) / bounds_length)
                    elevation_min_weight = 1.0 - elevation_max_weight
                    current_elevation = (elevation_min * elevation_min_weight) + (elevation_max * elevation_max_weight)
                    #print(bounds_min, bounds_max, elevation_min, elevation_max, current_elevation, position_s)
            new_elevations_reference_line.append(current_elevation)
        return np.array(new_elevations_reference_line)
    
    def generateElevationProfileFromOSMWayData(self, way_data, plan_view, road_length):
        elevations = []
        positions = plan_view.sampleReferenceLine(self.reference_line_resolution)
        positions_bridges = self.markPositionsWithBridges(way_data, positions, plan_view)
        elevations_reference_line = None
        # For bridges we linearly interpolate between the elevations given from the start to the end
        # We assume bridges do not dip, or curve up significantly between nodes.
        if way_data["bridge"]:
            beginning_node = self.nodes[way_data["nodes"][0]]
            ending_node = self.nodes[way_data["nodes"][-1]]
            beginning_node_coordinates = beginning_node["meters_coordinates"]
            ending_node_coordinates = ending_node["meters_coordinates"]
            hdg = _giveHeading(beginning_node_coordinates[0], beginning_node_coordinates[1], ending_node_coordinates[0], ending_node_coordinates[1])
            positions = np.array([[0.0, beginning_node_coordinates[0], beginning_node_coordinates[1], hdg], [road_length, ending_node_coordinates[0], ending_node_coordinates[1], hdg]])
            elevations_reference_line = np.array([beginning_node_coordinates[2], ending_node_coordinates[2]])
        else:
            elevations_reference_line = self.getDEMElevationsAtPoints(positions)
            elevations_reference_line = self.annotateDEMElevationsWithBridgeData(elevations_reference_line, way_data, positions, positions_bridges)
        elevations = self.generateElevationSequencesFromReferenceline(positions, elevations_reference_line)
        return opendrive.ElevationProfile(elevations=elevations)

    def generateRoadFromOSMWay(self, wid):
        way_data = self.ways[wid]
        id = self.ways[wid]["opendrive_id"]
        plan_view, road_length = self.generatePlanViewFromOSMWayData(way_data)
        elevation_profile = self.generateElevationProfileFromOSMWayData(way_data, plan_view, road_length)
        lanes = self.generateLanesFromOSMWayData(way_data)
        linkage = self.generateRoadLinkageFromOSMWayData(way_data)
        return opendrive.Road(id=id, length=road_length, planView=plan_view, elevationProfile=elevation_profile, lanes=lanes, link=linkage)
    
    def generateRoadsFromOSMWays(self):
        roads = {}
        for wid in self.ways:
            road = self.generateRoadFromOSMWay(wid)
            roads[road.id] = road
        return roads
    
    def generateJunctionsFromOSMNodes(self):
        junctions = []
        for nid in self.nodes:
            node = self.nodes[nid]
            junction_id = node["junction_id"]
            if junction_id != "-1":
                connections = []
                #opendrive.Connection()
                junctions.append(opendrive.Junction(id=junction_id, name=junction_id, type="default", connections=connections))
        return junctions

    def convertToOpenDrive(self):
        header = self.generateOpenDriveHeader()
        roads = self.generateRoadsFromOSMWays()
        junctions = self.generateJunctionsFromOSMNodes()
        return opendrive.OpenDRIVE(header=header, roads=roads, junctions=junctions)