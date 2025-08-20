import osmium
import pyproj
import numpy as np
from . import opendrive

class OSMToOpenDrive(osmium.SimpleHandler):
    feet_to_meters = 0.3048
    origin = None
    bounds_meters = None
    bounds_coords = None
    proj = None
    resolution = None
    nodes = {}
    ways = {}

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
            "meters_coordinates": meters_coordinates
        }

    def way(self, w):
        w_id = str(w.id)
        if 'highway' in w.tags:
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
            lane_width = float(lane_width_by_highway[lane_type]) if lane_type in lane_width_by_highway else 3.65
            nodes = []
            lanes = w.tags.get('lanes')
            lane_count = 1
            if lanes is not None:
                try:
                    lane_count = int(lanes)
                except ValueError:
                    pass
            for n in w.nodes:
                n_ref = str(n.ref)
                nodes.append(n_ref)
            way = {
                "lane_width": lane_width,
                "lane_count": lane_count,
                "bridge": bridge,
                "nodes": nodes,
                "wid": w_id
            }
            self.ways[w_id] = way

    def readOSM(self):
        self.apply_file(self.dataset.getOSMPath())

    def generateOpenDriveHeader(self):
        geoReference = opendrive.GeoReference(proj4="<![CDATA[+proj=tmerc +lat_0={0:0.10f} +lon_0={1:0.10f} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs]]>".format(self.bounds_coords[0], self.bounds_coords[1]))
        return opendrive.Header(revMajor=1, revMinor=4, north=self.bounds_meters[3]-self.bounds_meters[1], south=0.0, east=self.bounds_meters[2]-self.bounds_meters[0], west=0.0, geoReference=geoReference)

    def convertToOpenDrive(self):
        header = self.generateOpenDriveHeader()
        return opendrive.OpenDRIVE(header=header)