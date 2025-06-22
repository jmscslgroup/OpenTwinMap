from math import floor, pi
import numpy as np
from OSMParser.testing import TestEntity, _test_nodes, testSimpleRoad, test_3WayTCrossing2
from OSMParser.osmParsing import parseAll,rNode, OSMWay,JunctionRoad, OSMWayEndcap, createOSMJunctionRoadLine, createOSMWayNodeList2XODRRoadLine
from OSMParser.xodrWriting import startBasicXODRFile,fillNormalRoads,fillJunctionRoads
from DEM_python import DEM

osmPfad = '148110_full_corrected_2022_lidar.osm'
dem_path = '148110.asc'
xodrPfad = '148110_full_2022_lidar.xodr'

dem_data = DEM(dem_path, 2.0, -999999)

parseAll(osmPfad, dem=dem_data, minimumHeight = 163.0, maximumHeight= 192.0, curveRadius=12)

startBasicXODRFile(xodrPfad)
fillNormalRoads(xodrPfad)
fillJunctionRoads(xodrPfad)

