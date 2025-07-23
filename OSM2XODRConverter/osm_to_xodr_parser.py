import osmium
import copy
import numpy as np
from .utils import createVirtualLastPointForJunctionRoads, giveHeading, getDeltaHdg, distance
from .arc_curves import getArcCurvatureAndLength

class OSMToXODRParser(osmium.SimpleHandler):
    map_data = None
    dem_data = None
    nodes = None
    preway = None
    allWays = None
    element_id = 0
    idNr = 0
    junctionNodes = {}

    def __init__(self, map_data):
        self.map_data = map_data
        self.dem_data = self.map_data.loadDEMs(self.map_data.getAllTiles())
        self.nodes = {}
        self.preway = {}
        self.allWays = {}
        self.element_id = 0
        self.idNr = 0
        self.junctionNodes = {}

    def getBoundsInCoords(self):
        return self.map_data.getBoundsInCoords()

    def getBoundsInMeters(self):
        return self.map_data.getBoundsInMeters()

    def giveNextElementID(self):
        self.element_id += 1
        return str(self.element_id)

    def parseAll(self, minimum_height=0.0, maximum_height=100.0, curve_radius=8):
        
        osm_path = self.map_data.getCorrectedOSMPath()
        self.apply_file(osm_path)

        for preWay in self.preway.keys():
            self.evaluatePreWay(preWay)

        for node_key in self.nodes.keys():
            node_value = self.nodes[node_key]
            for way in node_value["incomingWays"]:
                self.createConnections(node_key, way)
            for way in node_value["outgoingWays"]:
                self.createConnections(node_key, way)
            self.evaluateJunction(node_key)

        for node_key in self.nodes.keys():
            self.evaluateJunction2(node_key)

        for way in self.allWays.values():
            way["roadElements"], way["elevationElements"] = self.createOSMWayNodeList2XODRRoadLine(way)

    def node(self, n):
        #rNode(n, substractMin=topoParameter)
        nid = str(n.id)
        node_data = {}
        node_data["tags"] = n.tags
        bounds = self.getBoundsInMeters()
        
        node_data["x"], node_data["y"] = self.map_data.convertToMeters([float(n.lon), float(n.lat)])

        node_data["x"] -= bounds[0]
        node_data["y"] -= bounds[1]
        try: 
            self.height = float(node_data["tags"]["ele"])
        except: 
            print("CHUNGUS")
            self.height = 0.0

        node_data["Junction"] = ""
        node_data["JunctionRoads"] = []
        node_data["wayList"] = []
        node_data["_PreWayIdList"] = []

        node_data["incomingWays"] = []
        node_data["incomingrNodes"] = []
        node_data["incomingLanes"] = []
        node_data["incomingLanesOpposite"] = []
        node_data["incomingTurnTags"] = []

        node_data["outgoingLanes"] = []
        node_data["outgoingLanesOpposite"] = []
        node_data["outgoingLanesOppositeTurnTags"] = []
        node_data["outgoingrNodes"] = []
        node_data["outgoingWays"] = []

        node_data["Connections"] = {}
        node_data["id"] = nid

        self.nodes[nid] = node_data

    def _givePossibleTurnIdxs(self, nid, way):
        '''Gives the Indexes of the registered Ways with >0 outgoing Lanes'''
        node_data = self.nodes[nid]
        turnIdxToIncoming = []
        turnIdxToOutgoing = []
        if way in node_data["incomingWays"] or way in node_data["outgoingWays"]:
            for incIdx in range(len(node_data["incomingWays"])):
                if way != node_data["incomingWays"][incIdx]:     # no U-Turn
                    if node_data["incomingLanesOpposite"][incIdx] > 0:  # no Turning in One-Way Streets
                        turnIdxToIncoming.append(incIdx)
            for outIdx in range(len(node_data["outgoingWays"])):
                if way != node_data["outgoingWays"][outIdx]:     # no U-Turn
                    if node_data["outgoingLanes"][outIdx] > 0:  # no Turning in One-Way Streets
                        turnIdxToOutgoing.append(outIdx)
        return turnIdxToIncoming, turnIdxToOutgoing

    def giveTurnPossibilities(self, nid, incomingWay):
        '''Gives the Angles, Lanes, Ways, rNodes and Directions of all valid Turns from a Way as an incoming Way'''
        node_data = self.nodes[nid]
        turnsInc = []
        turnsOut = []
        selfHeading = 0
        if incomingWay in node_data["incomingWays"]: # no turn possibilities for a not really incoming way (way with no incoming lanes)
            if node_data["incomingLanes"][node_data["incomingWays"].index(incomingWay)] == 0:
                return{'Angles':[],'rNodes': [],'Lanes': [],'Ways': [],'WayDirection': []}
        elif incomingWay in node_data["outgoingWays"]:
            if node_data["outgoingLanesOpposite"][node_data["outgoingWays"].index(incomingWay)] == 0:
                return{'Angles':[],'rNodes': [],'Lanes': [],'Ways': [],'WayDirection': []}

        incIdx, outIdx = self._givePossibleTurnIdxs(nid, incomingWay)
        if incomingWay in node_data["incomingWays"]:
            selfincrNode = node_data["incomingrNodes"][node_data["incomingWays"].index(incomingWay)]
        elif incomingWay in node_data["outgoingWays"]:
            selfincrNode = node_data["outgoingrNodes"][node_data["outgoingWays"].index(incomingWay)]
        selfHeading = giveHeading(selfincrNode["x"], selfincrNode["y"], node_data["x"], node_data["y"]) # heading von "incoming" Node to self
        for i in range(len(incIdx)):
            nodeHeading = giveHeading(node_data["x"], node_data["y"], node_data["incomingrNodes"][incIdx[i]]["x"], node_data["incomingrNodes"][incIdx[i]]["y"])
            turn = getDeltaHdg(selfHeading,nodeHeading)
            turnsInc.append(turn)
        for i in range(len(outIdx)):
            nodeHeading = giveHeading(node_data["x"], node_data["y"], node_data["outgoingrNodes"][outIdx[i]]["x"], node_data["outgoingrNodes"][outIdx[i]]["y"])
            turn = getDeltaHdg(selfHeading,nodeHeading)
            turnsOut.append(turn)
        return {'Angles':turnsInc+turnsOut,
                'rNodes': [node_data["incomingrNodes"][i] for i in incIdx]+[node_data["outgoingrNodes"][i] for i in outIdx],
                'Lanes': [node_data["incomingLanesOpposite"][i] for i in incIdx]+[node_data["outgoingLanes"][i] for i in outIdx],
                'Ways': [node_data["incomingWays"][i] for i in incIdx]+[node_data["outgoingWays"][i] for i in outIdx],
                'WayDirection': [False]*len(turnsInc)+[True]*len(turnsOut)}

    def createConnections(self, nid, way):
        '''Creates Laneconnections ([Lane, successorLane]) of the way for all successors and stores them in self.Connections[Way][Successorway].
        The Laneconnections are already adjusted for Waydirection'''
        # check if way is incoming or outgoing and get the incoming lanes as well as the index
        node_data = self.nodes[nid]
        positiveIncLanes = True
        lanenumbers = 0
        wayIdx = -1
        if way in node_data["incomingWays"]:
            positiveIncLanes = False
            wayIdx = node_data["incomingWays"].index(way)
            lanenumbers = node_data["incomingLanes"][wayIdx]
        else:
            wayIdx = node_data["outgoingWays"].index(way)
            lanenumbers = node_data["outgoingLanesOpposite"][wayIdx]
        turnPossibilities = self.giveTurnPossibilities(nid, way)
        sortangles = copy.copy(turnPossibilities['Angles'])
        sortidx = sorted(range(len(sortangles)), key=lambda k: sortangles[k])
        sortangles.sort()
        if lanenumbers == 0 or sum(turnPossibilities['Lanes']) == 0:
            return
        wayConnections = {}
        #sort all turnPossibilities according to angles
        for key in turnPossibilities.keys():
            tmp = []
            for i in range(len(turnPossibilities[key])):
                tmp.append(turnPossibilities[key][sortidx[i]])
            turnPossibilities[key] = tmp

        for idx in range(len(turnPossibilities['Angles'])):
            lanesum = 0
            for i in range(idx):     # get the lanenumber of the incoming lane
                lanesum += turnPossibilities['Lanes'][i]
                if lanesum > lanenumbers:
                    lanesum = lanenumbers-1      #lanes, die bisher verbraucht wurden

            # ist noch platz für die outgoinglanes?
            if lanesum+turnPossibilities['Lanes'][idx] > lanenumbers:
                lanesum = max(0,lanesum-turnPossibilities['Lanes'][idx])
            laneConnections = []
            for i in range(turnPossibilities['Lanes'][idx]):   #lanes, die hier outgoing sind
                if lanesum+i+1 > lanenumbers: # more lanes to turn into in one Possibility than incoming lanes
                    if len(sortangles) == 1: # merging and splitting Lanes - all Lanes should be accessible
                        lanesum -= 1
                    else:
                        break   # turning into a main street - only use outer lane
                # create Connection
                if positiveIncLanes:  # Way is in OutgoingWays
                    laneConnections.append([lanesum+i+1, -i-1 if turnPossibilities['WayDirection'][idx] else i+1])
                else:
                    laneConnections.append([-lanesum-i-1, -i-1 if turnPossibilities['WayDirection'][idx] else i+1])
            # extra merging lanes
            if turnPossibilities['Lanes'][idx] < lanenumbers and len(node_data["wayList"])==2:
                for i in range(lanenumbers-turnPossibilities['Lanes'][idx]):
                    # create Connection
                    if positiveIncLanes:  # Way is in OutgoingWays
                        laneConnections.append([turnPossibilities['Lanes'][idx]+i+1, -turnPossibilities['Lanes'][idx] if turnPossibilities['WayDirection'][idx] else turnPossibilities['Lanes'][idx]])
                    else:
                        laneConnections.append([-turnPossibilities['Lanes'][idx]-i-1, -turnPossibilities['Lanes'][idx] if turnPossibilities['WayDirection'][idx] else turnPossibilities['Lanes'][idx]])

            wayConnections[(turnPossibilities['Ways'][idx])["id"]] = laneConnections
        if positiveIncLanes: # Way is in OutgoingWays
            try: node_data["Connections"][way["id"]]["Opposite"] =  wayConnections
            except:
                node_data["Connections"][way["id"]] = {}
                node_data["Connections"][way["id"]]["Opposite"] =  wayConnections
        else:
            try: node_data["Connections"][way["id"]]["Direction"] =  wayConnections
            except:
                node_data["Connections"][way["id"]] = {}
                node_data["Connections"][way["id"]]["Direction"] =  wayConnections

    def evaluateJunction(self, nid):
        node_data = self.nodes[nid]
        if len(node_data["wayList"])>1:
            node_data["Junction"] = self.giveNextElementID()
            jrxs = []
            jrys = []
            jx,jy = [node_data["x"], node_data["y"]]
            nodes = node_data["incomingrNodes"] + node_data["outgoingrNodes"]
            for node in nodes:
                if len(node["wayList"]) > 1: #junction to junction -> do not go beyond half of the way
                    jrxs.append((node["x"]+node_data["x"])/2.0)
                    jrys.append((node["y"]+node_data["y"])/2.0)
                else:
                    jrxs.append(node["x"])
                    jrys.append(node["y"])
            maxlanes = max(node_data["incomingLanes"]+node_data["outgoingLanes"])
            radius = 4.0 * maxlanes
            node_data["lastPoints"] = createVirtualLastPointForJunctionRoads(jx,jy,jrxs,jrys,radius = radius)
            for way in node_data["wayList"]:
                if node_data["id"] == way["OSMNodes"][0]:
                    way["startJunction"] = node_data["Junction"]
                else:
                    way["endJunction"] = node_data["Junction"]

    def evaluateJunction2(self, nid):
        #get all way connections in direction relevant from wayX to wayY
        node_data = self.nodes[nid]
        laneconnections = {}
        for wayX in node_data["Connections"].keys():
            wayXisIncoming = True if self.allWays[wayX]["OSMNodes"][-1] == node_data["id"] else False
            for direc in node_data["Connections"][wayX].keys():
                if wayXisIncoming and direc == "Opposite": continue
                if not wayXisIncoming and direc == "Direction": continue
                for wayY in node_data["Connections"][wayX][direc].keys():
                    CName = wayX+"_"+wayY# if wayX < wayY else wayY+"_"+wayX
                    if CName in laneconnections:
                        dic = laneconnections[CName]
                    else:
                        laneconnections[CName] = {}
                        dic = laneconnections[CName]
                    for connection in node_data["Connections"][wayX][direc][wayY]:
                        startlane = connection[0]# if wayX < wayY else connection[1]
                        endlane = connection[1]# if wayX < wayY else connection[0]
                        if startlane in dic:
                            dic[startlane].append(endlane)
                        else:
                            dic[startlane] = [endlane]
        #create a road for every connection
        for key in laneconnections.keys():
            keys = key.split("_")
            predecessorway = self.allWays[keys[0]]
            successorway = self.allWays[keys[1]]
            node_data["JunctionRoads"] += self.createJunctionRoadsForConnection(predecessorway,successorway,node_data)

    def getRelevantLastPoint(self, nid, way):
        node_data = self.nodes[nid]
        ways = node_data["incomingWays"] + node_data["outgoingWays"]
        wayidx = ways.index(way)
        return node_data["lastPoints"][wayidx]

    def way(self, w):
        if "highway" in w.tags and not "stairs" in w.tags["highway"] and not "steps" in w.tags["highway"] and not "pedestrian" in w.tags["highway"] and not "elevator" in w.tags["highway"] and not "footway" in w.tags["highway"] and not "bridleway" in w.tags["highway"] and not "cycleway" in w.tags["highway"] and not "path" in w.tags["highway"]:
            way_data = {}
            wid = str(w.id)
            way_data["id"] = wid
            way_data["tags"] = w.tags
            way_data["rNodes"] = []

            for node in w.nodes:
                if str(node) not in self.nodes:
                    continue
                way_data["rNodes"].append(self.nodes[str(node)]["id"])
            if len(way_data["rNodes"]) > 1:
                for node in w.nodes:
                    self.nodes[str(node)]["_PreWayIdList"].append(wid)
            self.preway[wid] = way_data

    def evaluatePreWay(self, wid):
        way_data = self.preway[wid]
        startIdx = 0
        endIdx = -1
        if len(way_data["rNodes"]) < 2:
            return
        lastIdx = -1
        for rNodeId in way_data["rNodes"]:
            node = self.nodes[rNodeId]
            idx = way_data["rNodes"].index(rNodeId)
            if idx < lastIdx:
                if startIdx == lastIdx:
                    idx = lastIdx+1
                else:
                    #create fake mergeRoad
                    idx = lastIdx
                    endIdx = idx+1
                    self.createAllWay(way_data["id"], way_data["tags"], way_data["rNodes"][startIdx:endIdx], way_data["rNodes"][startIdx], way_data["rNodes"][endIdx-1])
                    startIdx = idx
                    idx = lastIdx+1
            if len(node["_PreWayIdList"]) > 1:
                if idx == startIdx or idx == endIdx-1:
                    continue
                else:
                    endIdx = idx+1
                    self.createAllWay(way_data["id"], way_data["tags"], way_data["rNodes"][startIdx:endIdx], way_data["rNodes"][startIdx], way_data["rNodes"][endIdx-1])
                    startIdx = idx
            lastIdx = idx
        if endIdx < len(way_data["rNodes"]):
            endIdx = len(way_data["rNodes"])
            self.createAllWay(way_data["id"], way_data["tags"], way_data["rNodes"][startIdx:endIdx], way_data["rNodes"][startIdx], way_data["rNodes"][endIdx-1])

    def giveWayID(self):
        self.idNr += 1
        return self.idNr

    def createAllWay(self, wid, tags, way_nodes, way_node_start, way_node_end):
        way_data = {}
        way_data["id"] = str(self.giveWayID())
        way_data["xodrID"] = self.giveNextElementID()
        way_data["OSMId"] = wid
        way_data["tags"] = tags
        way_data["OSMNodes"] = way_nodes

        way_data["laneNumberDirection"] = -1
        way_data["laneNumberOpposite"] = -1

        way_data["K1rNode"] = way_node_end
        way_data["K2rNode"] = way_node_start
        way_data["startJunction"] = ""
        way_data["endJunction"] = ""
        way_data["K1Links"] = []
        way_data["K2Links"] = []
        way_data["lastPoints"] = []
        way_data["K1_turnLanesDirection"] = []
        way_data["K1_ConnectionsTurnLanesDirection"] = []
        way_data["K1_incomingLanesFromK1"] = []
        way_data["K2_turnLanesOpposite"] = []
        way_data["K2_ConnectionsTurnLanesOpposite"] = []
        way_data["K2_incomingLanesFromK2"] = []
        way_data["roadElements"] = []
        way_data["elevationElements"] = []

        '''
        checks how many Lanes this street should have
        '''
        #laneNumberDirection und laneNumberOpposite sind die grobxodrIDen Uebersichten.
        laneNumberDirection = -1
        laneNumberOpposite = -1
        way_data["K1_turnLanesDirection"] = []
        way_data["K2_turnLanesOpposite"] = []
        lanes = -1
        oneWay = False
        try:
            if 'yes' in way_data["tags"]["oneway"]:
                oneWay = True
                #print("oneway found")
        except:  pass

        try:
            lanes = int(way_data["tags"]["lanes"])
            #print("lanes found")
        except: pass
        try:
            laneNumberDirection = int(way_data["tags"]["lanes:forward"])
            #print("lanes:forward found")
        except: pass
        try:
            laneNumberOpposite = int(way_data["tags"]["lanes:backward"])
            #print("lanes:backward found")
        except: pass
        try: way_data["K1_turnLanesDirection"] = way_data["tags"]["turn:lanes:forward"].replace("slight_left","slight_l").replace("slight_right","slight_r").replace("merge_to_right","merge_r").replace("merge_to_left", "merge_l").split("|")
        except:
            try: way_data["K1_turnLanesDirection"] = way_data["tags"]["turn:lanes"].replace("slight_left","slight_l").replace("slight_right","slight_r").replace("merge_to_right","merge_r").replace("merge_to_left", "merge_l").split("|")
            except: pass
        try:way_data["K2_turnLanesOpposite"] = way_data["tags"]["turn:lanes:backward"].replace("slight_left","slight_l").replace("slight_right","slight_r").replace("merge_to_right","merge_r").replace("merge_to_left", "merge_l").split("|")
        except: pass
        continue_lane_check = True
        if lanes > 0 and laneNumberDirection + laneNumberOpposite == lanes:  #best case
            #print("all clear")
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite
        if lanes > 0 and oneWay:
            laneNumberOpposite = 0
            laneNumberDirection = lanes
            #print("all clear")
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite
            continue_lane_check = False
        if laneNumberDirection > 0 and oneWay and continue_lane_check:
            #print("all clear")
            lanes = laneNumberDirection
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite
            continue_lane_check = False
        if laneNumberDirection > 0 and laneNumberOpposite>0 and continue_lane_check:
            #print("all clear")
            lanes = laneNumberDirection + laneNumberOpposite
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite
            continue_lane_check = False
        if (len(way_data["K1_turnLanesDirection"]) > 0 or len(way_data["K2_turnLanesOpposite"]) > 0) and lanes == -1 and continue_lane_check:
            lanes = len(way_data["K1_turnLanesDirection"]) + len(way_data["K2_turnLanesOpposite"])
            way_data["laneNumberDirection"] = len(way_data["K1_turnLanesDirection"])
            way_data["laneNumberOpposite"] = len(way_data["K2_turnLanesOpposite"])
            continue_lane_check = False
        if lanes > 0 and laneNumberDirection >= 0 and continue_lane_check:
            laneNumberOpposite = lanes - laneNumberDirection
        if lanes > 0 and laneNumberOpposite >= 0 and continue_lane_check:
            laneNumberDirection = lanes - laneNumberOpposite
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite
            continue_lane_check = False

        if continue_lane_check:
            if lanes == -1:
                lanes = 1 if oneWay else 2
            
            laneNumberDirection = lanes if oneWay else 1
            laneNumberOpposite = 0 if oneWay else 1
            if len(way_data["K1_turnLanesDirection"]) > 0:
                laneNumberDirection = len(way_data["K1_turnLanesDirection"])
                laneNumberOpposite = lanes-laneNumberDirection
            if len(way_data["K2_turnLanesOpposite"]) > 0:
                laneNumberOpposite = len(way_data["K2_turnLanesOpposite"])
                laneNumberDirection = lanes-laneNumberOpposite
            way_data["laneNumberDirection"] = laneNumberDirection
            way_data["laneNumberOpposite"] = laneNumberOpposite

        previousrNode = None
        if len(way_data["OSMNodes"]) > 1:
            for nodeid in way_data["OSMNodes"]:
                node = self.nodes[nodeid]
                node["wayList"].append(way_data)
                if previousrNode is not None:
                    previousrNode["outgoingrNodes"].append(node)
                    previousrNode["outgoingWays"].append(way_data)
                    previousrNode["outgoingLanes"].append(way_data["laneNumberDirection"])
                    previousrNode["outgoingLanesOpposite"].append(way_data["laneNumberOpposite"])
                    previousrNode["outgoingLanesOppositeTurnTags"].append(way_data["K2_turnLanesOpposite"])
                    node["incomingrNodes"].append(previousrNode)
                    node["incomingWays"].append(way_data)
                    node["incomingLanes"].append(way_data["laneNumberDirection"])
                    node["incomingLanesOpposite"].append(way_data["laneNumberOpposite"])
                    node["incomingTurnTags"].append(way_data["K1_turnLanesDirection"])
                    if previousrNode:
                        if len(previousrNode["wayList"]) > 1:
                            assert len(previousrNode["wayList"]) == len(previousrNode["incomingWays"])+len(previousrNode["outgoingWays"])
                    if len(node["wayList"]) > 1:
                        assert len(node["wayList"]) == len(node["incomingWays"])+len(node["outgoingWays"])
                previousrNode = node

        if len(way_data["K1_turnLanesDirection"]) < way_data["laneNumberDirection"]:
            way_data["K1_turnLanesDirection"] = [""]*way_data["laneNumberDirection"]
        if len(way_data["K2_turnLanesOpposite"]) < way_data["laneNumberOpposite"]:
            way_data["K2_turnLanesOpposite"] = [""]*way_data["laneNumberOpposite"]

        for i in range(way_data["laneNumberDirection"]):
            way_data["K2_incomingLanesFromK2"].append([])
            way_data["K1_ConnectionsTurnLanesDirection"].append([])
        for i in range(way_data["laneNumberOpposite"]):
            way_data["K1_incomingLanesFromK1"].append([])
            way_data["K2_ConnectionsTurnLanesOpposite"].append([])

        self.allWays[way_data["id"]] = way_data

    def createJunctionRoadsForConnection(self,predecessorway,successorway,junctionNode, maxerror=2.0):
        contactPointPredecessor = "start" if predecessorway["OSMNodes"][0] == junctionNode["id"] else "end"
        contactPointSuccessor = "start" if successorway["OSMNodes"][0] == junctionNode["id"] else "end"
        roadElements,elevationElements = self.createOSMJunctionRoadLine(predecessorway,successorway,junctionNode, maxerror=maxerror)
        predecessor2successorLaneConnections = []
        try:
            predecessorconnections = junctionNode["Connections"][predecessorway["id"]]
            try:
                if successorway["id"] in predecessorconnections["Direction"]:
                    for connection in predecessorconnections["Direction"][successorway["id"]]:
                        predecessor2successorLaneConnections.append(connection)
            except: pass
            try:
                if successorway["id"] in predecessorconnections["Opposite"]:
                    for connection in predecessorconnections["Opposite"][successorway["id"]]:
                        predecessor2successorLaneConnections.append(connection)
            except: pass
        except: pass
        try:
            successorconnections = junctionNode["Connections"][successorway["id"]]
            try:
                if predecessorway["id"] in successorconnections['Direction']:
                    for connection in successorconnections["Direction"][predecessorway["id"]]:
                        predecessor2successorLaneConnections.append([connection[1],connection[0]])
            except: pass
            try:
                if predecessorway["id"] in successorconnections["Opposite"]:
                    for connection in successorconnections["Opposite"][predecessorway["id"]]:
                        predecessor2successorLaneConnections.append([connection[1],connection[0]])
            except: pass
        except: pass
        roads = []
        for connection in predecessor2successorLaneConnections:
            roads.append(self.createJunctionRoad(predecessorway,successorway,connection[0],connection[1],junctionNode,contactPointPredecessor,contactPointSuccessor,roadElements,elevationElements))
        return roads

    def giveJunctionDict(self, junctionNode):
        if junctionNode["Junction"] in self.junctionNodes:
            return self.junctionNodes[junctionNode["Junction"]]
        else:
            self.junctionNodes[junctionNode["Junction"]] = {}
            return self.junctionNodes[junctionNode["Junction"]]

    def createJunctionRoad(self,predecessorway,successorway,startlane,endlane,junctionNode,contactPointPredecessor,contactPointSuccessor,roadElements,elevationElements):
        junction_data = {}
        junction_data["id"] = str(predecessorway["id"])+"_to_"+str(successorway["id"])+"_lane_"+str(startlane)+"_to_"+str(endlane)
        junction_data["xodrID"] = str(self.giveNextElementID())
        junctionDict = self.giveJunctionDict(junctionNode)
        if (str(predecessorway["id"])+"_to_"+str(successorway["id"])) in junctionDict:
            waydic = junctionDict[str(predecessorway["id"])+"_to_"+str(successorway["id"])]
        else:
            waydic = {}
            junctionDict[str(predecessorway["id"])+"_to_"+str(successorway["id"])] = waydic
        waydic[str(startlane)+"_to_"+str(endlane)] = junction_data
        junction_data["predecessorlane"] = startlane
        junction_data["successorlane"] = endlane
        junction_data["junctionNode"] = junctionNode
        junction_data["contactPointPredecessor"] = contactPointPredecessor
        junction_data["contactPointSuccessor"] = contactPointSuccessor
        junction_data["roadElements"] = roadElements
        junction_data["elevationElements"] = elevationElements
        junction_data["laneWidth"] = 4.0
        length = 0.0
        for element in junction_data["roadElements"]:
            length += element["length"]
        predecessorIsBackward = True if junction_data["contactPointPredecessor"] == "start" else False
        successorIsBackward = True if junction_data["contactPointSuccessor"] == "end" else False
        junction_data["laneOffsetA"] = (abs(junction_data["predecessorlane"])-1.0)* np.sign(junction_data["predecessorlane"]) * junction_data["laneWidth"]
        laneOffsetEnd = (abs(junction_data["successorlane"])-1.0)* np.sign(junction_data["successorlane"]) * junction_data["laneWidth"]
        junction_data["laneOffsetB"] = -(junction_data["laneOffsetA"]-laneOffsetEnd)/length
        return junction_data

    def createOSMJunctionRoadLine(self,way1,way2,junctionNode, maxerror=2.0):
        x1,y1 = self.getRelevantLastPoint(junctionNode["id"], way1)
        x2 = junctionNode["x"]
        y2 = junctionNode["y"]
        x3,y3 = self.getRelevantLastPoint(junctionNode["id"], way2)
        #calculate the parameter
        xarc,yarc,xendline,yendline,curvature,length = getArcCurvatureAndLength(x1,y1,x3,y3,x2,y2, maxerror = 999999.9, minradius = 0.5, iterations = 10)
        RoadElements = [] #xstart,ystart, length, heading, curvature
        ElevationElements = []
        z1 = self.map_data.minHeightAtXYMeters(self.dem_data, (x1, y1))
        z2 = self.map_data.minHeightAtXYMeters(self.dem_data, (x2, y2))
        z3 = self.map_data.minHeightAtXYMeters(self.dem_data, (x3, y3))
        if distance(x1,y1,xarc,yarc) > 0.1:
                    RoadElements.append({"xstart":x1,"ystart":y1, "length":distance(x1,y1,xarc,yarc), "heading":giveHeading(x1,y1,x2,y2), "curvature":0.0})
                    ElevationElements.append({"xstart":x1,"ystart":y1,"zstart":z1,
                                            "steigung":(self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc))-z1)/distance(x1,y1,xarc,yarc),"length":distance(x1,y1,xarc,yarc)})
        RoadElements.append({"xstart":xarc,"ystart":yarc, "length":length, "heading":giveHeading(x1,y1,x2,y2), "curvature":curvature})
        ElevationElements.append({"xstart":xarc,"ystart":yarc,"zstart":self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc)),
                                        "steigung":(self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline))-self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc)))/length,"length":length})
        if distance(xendline,yendline,x3,y3) > 0.1:
                    RoadElements.append({"xstart":xendline,"ystart":yendline, "length":distance(xendline,yendline,x3,y3), "heading":giveHeading(xendline,yendline,x3,y3), "curvature":0.0})
                    ElevationElements.append({"xstart":xendline,"ystart":yendline,"zstart":self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline)),
                                            "steigung":(z3-self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline)))/distance(xendline,yendline,x3,y3),"length":distance(xendline,yendline,x3,y3)})
        return RoadElements,ElevationElements

    def createOSMWayNodeList2XODRRoadLine(self, way, maxerror=2.0):
        Points = []
        hdgs = []
        RoadElements = [] #xstart,ystart, length, heading, curvature
        ElevationElements = []

        #prepare raw points
        #first element:
        firstNode = self.nodes[way["OSMNodes"][0]]
        if len(firstNode["wayList"]) == 1:  #firstnode ist sackgasse
            #get the full node involved
            Points.append([firstNode["x"],firstNode["y"], self.map_data.minHeightAtXYMeters(self.dem_data, (firstNode["x"], firstNode["y"]))])
        else: #firstnode is junction
            #get the relevant lastPoint as NodePoint
            x,y = self.getRelevantLastPoint(firstNode["id"], way)
            Points.append([x,y,self.map_data.minHeightAtXYMeters(self.dem_data, (x, y))])

        #middle element:
        for nodeId in way["OSMNodes"][1:-1]:
            node = self.nodes[nodeId]
            hdgs.append(giveHeading(Points[-1][0],Points[-1][1],node["x"],node["y"]))
            Points.append([node["x"],node["y"], self.map_data.minHeightAtXYMeters(self.dem_data, (node["x"], node["y"]))])


        #last element:
        lastNode = self.nodes[way["OSMNodes"][-1]]
        if len(lastNode["wayList"]) == 1:  #firstnode ist sackgasse
            #get the full node involved
            hdgs.append(giveHeading(Points[-1][0],Points[-1][1],lastNode["x"],lastNode["y"]))
            Points.append([lastNode["x"],lastNode["y"], self.map_data.minHeightAtXYMeters(self.dem_data, (lastNode["x"], lastNode["y"]))])

        else: #lastnode is junction
            #get the relevant lastPoint as NodePoint
            x,y = self.getRelevantLastPoint(lastNode["id"], way)
            Points.append([x,y,self.map_data.minHeightAtXYMeters(self.dem_data, (x,y))])
            hdgs.append(giveHeading(x,y,lastNode["x"],lastNode["y"]))

        if len(Points) == 2:
            #junction to junction -> Points can be the same!
            length = distance(Points[0][0], Points[0][1],Points[1][0],Points[1][1])
            x1,y1,z1 = Points[0]
            x2,y2,z2 = Points[1]
            RoadElements.append({"xstart":x1,"ystart":y1, "length":length, "heading":giveHeading(firstNode["x"],firstNode["y"],lastNode["x"],lastNode["y"]), "curvature":0.0})
            ElevationElements.append({"xstart":x1,"ystart":y1,"zstart":z1,"steigung":(z2-z1)/(length+0.00000001),"length":length})
        else: #mehr als 1 Punkt auf dem Weg
            for i in range(len(Points)-2):
                x1,y1,z1 = Points[i]
                x2,y2,z2 = Points[i+1]
                x3,y3,z3 = Points[i+2] #hdgs sind automatisch korrekt bei 3 point curves
                #for all but the first and last Point: get the Point halfway between x1/x2 and x2/x3
                if i == 0:
                    pass
                else:
                    x1 = (x1+x2)/2.0
                    y1 = (y1+y2)/2.0
                    z1 = self.map_data.minHeightAtXYMeters(self.dem_data, (x1, y1))
                if i == len(Points)-3:
                    pass
                else:
                    x3 = (x3+x2)/2.0
                    y3 = (y3+y2)/2.0
                    z3 = self.map_data.minHeightAtXYMeters(self.dem_data, (x3, y3))
                #calculate the parameter
                xarc,yarc,xendline,yendline,curvature,length = getArcCurvatureAndLength(x1,y1,x3,y3,x2,y2, maxerror = maxerror, minradius = 0.5, iterations = 10)

                if distance(x1,y1,xarc,yarc) > 0.1:
                    RoadElements.append({"xstart":x1,"ystart":y1, "length":distance(x1,y1,xarc,yarc), "heading":hdgs[i], "curvature":0.0})
                    ElevationElements.append({"xstart":x1,"ystart":y1,"zstart":z1,
                                            "steigung":(self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc))-z1)/distance(x1,y1,xarc,yarc),"length":distance(x1,y1,xarc,yarc)})
                RoadElements.append({"xstart":xarc,"ystart":yarc, "length":length, "heading":hdgs[i], "curvature":curvature})
                ElevationElements.append({"xstart":xarc,"ystart":yarc,"zstart":self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc)),
                                        "steigung":(self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline))-self.map_data.minHeightAtXYMeters(self.dem_data, (xarc, yarc)))/length,"length":length})
                if distance(xendline,yendline,x3,y3) > 0.1:
                    RoadElements.append({"xstart":xendline,"ystart":yendline, "length":distance(xendline,yendline,x3,y3), "heading":giveHeading(xendline,yendline,x3,y3), "curvature":0.0})
                    ElevationElements.append({"xstart":xendline,"ystart":yendline,"zstart":self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline)),
                                            "steigung":(z3-self.map_data.minHeightAtXYMeters(self.dem_data, (xendline, yendline)))/distance(xendline,yendline,x3,y3),"length":distance(xendline,yendline,x3,y3)})

        return RoadElements,ElevationElements