import xml.etree.ElementTree as ET
from . import opendrive

class OpenDriveParser:
    opendrive_data = None
    xodr_path = None

    def __init__(self, xodr_path):
        self.opendrive_data = opendrive.OpenDrive()
        self.xodr_path = xodr_path

    @classmethod
    def parseRoad(cls, road_tag):
        road_id = road_tag.get('id', '')
        length = float(road_tag.get('length', '0'))
        # Parse plan view geometries
        geometries = []
        plan_view = road_tag.find('planView')
        if plan_view is not None:
            for geo in plan_view.findall('geometry'):
                s = float(geo.get('s', '0'))
                x = float(geo.get('x', '0'))
                y = float(geo.get('y', '0'))
                hdg = float(geo.get('hdg', '0'))
                length_g = float(geo.get('length', '0'))
                geo_type = None
                curvature = 0.0
                if geo.find('line') is not None:
                    geo_type = 'line'
                elif geo.find('arc') is not None:
                    geo_type = 'arc'
                    curvature = float(geo.find('arc').get('curvature', '0'))
                else:
                    continue
                geometries.append(opendrive.RoadGeometry(s, x, y, hdg, length_g, geo_type, curvature))
        # Parse lane sections
        lane_sections = []
        lanes_elem = road_tag.find('lanes')
        if lanes_elem is not None:
            for ls_elem in lanes_elem.findall('laneSection'):
                s_start = float(ls_elem.get('s', '0'))
                lanes_dict = {}
                for side in ['left', 'center', 'right']:
                    side_elem = ls_elem.find(side)
                    if side_elem is None:
                        continue
                    for lane_elem in side_elem.findall('lane'):
                        try:
                            lane_id = int(lane_elem.get('id'))
                        except (TypeError, ValueError):
                            continue
                        width_segments = cls.parseWidthSegments(lane_elem)
                        lanes_dict[lane_id] = opendrive.Lane(lane_id, width_segments)
                lane_sections.append(opendrive.LaneSection(s_start, lanes_dict))
        # Parse elevation profile
        elevation_segments = []
        elev_profile = road_tag.find('elevationProfile')
        if elev_profile is not None:
            for elev in elev_profile.findall('elevation'):
                try:
                    s_offset = float(elev.get('s', elev.get('sOffset', '0')))
                    a = float(elev.get('a', '0'))
                    b = float(elev.get('b', '0'))
                    c = float(elev.get('c', '0'))
                    d = float(elev.get('d', '0'))
                except (TypeError, ValueError):
                    continue
                elevation_segments.append((s_offset, a, b, c, d))
        return opendrive.Road(
            road_id,
            length,
            geometries,
            lane_sections,
            elevation_segments=elevation_segments
        )

    @classmethod
    def parseJunction(cls, junction_tag):
        connections = []
        for conn in junction_tag.findall('connection'):
            incoming_road = conn.get('incomingRoad')
            connecting_road = conn.get('connectingRoad')
            contact_point = conn.get('contactPoint', 'end')
            if incoming_road and connecting_road:
                connections.append(opendrive.JunctionConnection(incoming_road, connecting_road, contact_point))
        return connections

    @classmethod
    def parseWidthSegments(self, lane_tag):
        """Extract width polynomial segments from a lane element.

        Each `<width>` entry in OpenDRIVE has coefficients a, b, c, d and
        an sOffset which defines the start of the segment relative to the
        beginning of its lane section.  The polynomial is evaluated as
        width(ds) = a + b*ds + c*ds**2 + d*ds**3【701275300476046†L203-L284】.

        Returns:
            A list of tuples (s_offset, a, b, c, d) sorted by s_offset.
        """
        segments = []
        for width in lane_tag.findall('width'):
            try:
                s_offset = float(width.get('sOffset', '0'))
                a = float(width.get('a', '0'))
                b = float(width.get('b', '0'))
                c = float(width.get('c', '0'))
                d = float(width.get('d', '0'))
            except (TypeError, ValueError):
                # Skip any malformed width entries
                continue
            segments.append((s_offset, a, b, c, d))
        # sort by sOffset so we can traverse them in order
        segments.sort(key=lambda seg: seg[0])
        return segments

    def parseOpenDriveFile(self):
        tree = ET.parse(self.xodr_path)
        root = tree.getroot()
        for road_tag in root.findall('road'):
            road = self.parseRoad(road_tag)
            self.opendrive_data.addRoad(road)
        connections = []
        for junction_tag in root.findall('junction'):
            connections += self.parseJunction(junction_tag)
        self.opendrive_data.addConnections(connections)