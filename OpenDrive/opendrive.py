import math

class Lane:
    """Represents a single lane with its ID and width polynomial segments."""

    def __init__(self, lane_id, width_segments):
        self.id = lane_id
        self.width_segments = width_segments

    def toDict(self):
        return vars(self)

    def widthAt(self, ds):
        """Return the width of the lane at distance ds from the start of its lane section.

        If no width segments are defined the lane is assumed to be 3.5 m wide.
        """
        if not self.width_segments:
            return 3.5  # default lane width in metres
        # Find the last segment whose s_offset <= ds
        current_segment = self.width_segments[0]
        for seg in self.width_segments:
            if ds >= seg[0]:
                current_segment = seg
            else:
                break
        s_offset, a, b, c, d = current_segment
        local_ds = max(ds - s_offset, 0.0)
        # Evaluate the cubic polynomial
        return a + b * local_ds + c * local_ds ** 2 + d * local_ds ** 3

class LaneSection:
    """Represents a lane section which groups a set of lanes for a range of s.

    Attributes:
        s_start: global s-coordinate where this lane section starts along the road.
        lanes: dictionary mapping lane IDs to Lane objects.
    """

    def __init__(self, s_start, lanes):
        self.s_start = s_start
        self.lanes = lanes

    def toDict(self):
        result = vars(self)
        result_lane_dict = {}
        for k in result["lanes"]:
            result_lane_dict[k] = result["lanes"][k].toDict()
        result["lanes"] = result_lane_dict
        
        return result

    def widthAt(self, s):
        """Compute the total left and right widths at absolute s within this lane section.

        Args:
            s: absolute distance along the road reference line.

        Returns:
            A tuple (left_width, right_width) where left_width is the total width
            of lanes with positive IDs and right_width is the total width of
            lanes with negative IDs.  The centre lane (ID 0) has zero width.
        """
        ds = s - self.s_start
        left_width = 0.0
        right_width = 0.0
        for lane_id, lane in self.lanes.items():
            if lane_id == 0:
                continue
            w = lane.widthAt(ds)
            if lane_id > 0:
                left_width += w
            else:
                right_width += w
        return left_width, right_width

class RoadGeometry:
    """Represents a single geometry segment of a road plan view."""

    def __init__(self, s, x, y, hdg, geom_length, geo_type, curvature = 0.0):
        self.s = s  # start s along the road
        self.x = x
        self.y = y
        self.hdg = hdg  # heading angle (radians)
        self.geom_length = geom_length
        self.type = geo_type
        self.curvature = curvature  # used for arcs; curvature = 1/radius

    def toDict(self):
        return vars(self)

    def samplePositions(self, num_samples):
        """Sample (x, y, phi) positions along this geometry.

        Args:
            num_samples: number of samples to generate along the geometry,
                including both endpoints.

        Returns:
            A list of tuples (x, y, phi) where phi is the orientation at the
            sampled point.
        """
        if num_samples < 2:
            raise ValueError("num_samples must be >= 2")
        positions = []
        if self.type == 'line':
            # Straight line: phi is constant
            for i in range(num_samples):
                ds = (self.geom_length) * i / (num_samples - 1)
                x = self.x + ds * math.cos(self.hdg)
                y = self.y + ds * math.sin(self.hdg)
                phi = self.hdg
                positions.append((x, y, phi))
        elif self.type == 'arc':
            # Circular arc: curvature k defines radius = 1/k
            k = self.curvature
            # avoid division by zero; treat near zero curvature as straight line
            if abs(k) < 1e-8:
                for i in range(num_samples):
                    ds = (self.geom_length) * i / (num_samples - 1)
                    x = self.x + ds * math.cos(self.hdg)
                    y = self.y + ds * math.sin(self.hdg)
                    phi = self.hdg
                    positions.append((x, y, phi))
            else:
                radius = 1.0 / k
                # Starting orientation
                phi0 = self.hdg
                # Centre of the circle
                cx = self.x - radius * math.sin(phi0)
                cy = self.y + radius * math.cos(phi0)
                for i in range(num_samples):
                    ds = (self.geom_length) * i / (num_samples - 1)
                    # angle change along the arc
                    phi = phi0 + k * ds
                    x = cx + radius * math.sin(phi)
                    y = cy - radius * math.cos(phi)
                    positions.append((x, y, phi))
        else:
            raise NotImplementedError(f"Unsupported geometry type: {self.type}")
        return positions

class Road:
    """Container for an OpenDRIVE road with plan view and lane sections."""
    id = None
    road_length = None
    geometries = None
    lane_sections = None
    elevation_segments = None
    superelevation_segments = None

    def __init__(self, road_id, road_length, geometries, lane_sections, elevation_segments = None, superelevation_segments = None):
        self.id = road_id
        self.road_length = road_length
        self.geometries = geometries
        self.lane_sections = sorted(lane_sections, key=lambda ls: ls.s_start)
        self.elevation_segments = sorted(elevation_segments or [], key=lambda seg: seg[0])
        self.superelevation_segments = sorted(superelevation_segments or [], key=lambda seg: seg[0])

    def toDict(self):
        result = vars(self)
        new_geometry_list = []
        for geom in result["geometries"]:
            new_geometry_list.append(geom.toDict())
        result["geometries"] = new_geometry_list

        new_lane_section_list = []
        for lane_section in result["lane_sections"]:
            new_lane_section_list.append(lane_section.toDict())
        result["lane_sections"] = new_lane_section_list

        return result

    def laneSectionAt(self, s):
        """Return the LaneSection that covers the given s coordinate."""
        # iterate through lane sections; the last one whose s_start <= s
        current = self.lane_sections[0]
        for ls in self.lane_sections:
            if s >= ls.s_start:
                current = ls
            else:
                break
        return current

    def elevationAt(self, s):
        """Compute elevation z at global s using the elevation profile.

        If no elevation segments are defined the result is 0.0.  The height is
        computed using a cubic polynomial z(ds) = a + b*ds + c*ds**2 + d*ds**3
        where ds is measured from the segment's start offset.
        """
        if not self.elevation_segments:
            return 0.0
        current_seg = self.elevation_segments[0]
        for seg in self.elevation_segments:
            if s >= seg[0]:
                current_seg = seg
            else:
                break
        s_offset, a, b, c, d = current_seg
        ds = max(s - s_offset, 0.0)
        return a + b * ds + c * ds ** 2 + d * ds ** 3

    def superElevationAt(self, s):
        """Compute superelevation angle at global s.

        The superelevation profile is defined by polynomial segments similar to
        elevation.  Positive values denote a road falling to the right side【701275300476046†L262-L281】.
        Returns 0.0 if no superelevation is defined.
        """
        if not self.superelevation_segments:
            return 0.0
        current_seg = self.superelevation_segments[0]
        for seg in self.superelevation_segments:
            if s >= seg[0]:
                current_seg = seg
            else:
                break
        s_offset, a, b, c, d = current_seg
        ds = max(s - s_offset, 0.0)
        return a + b * ds + c * ds ** 2 + d * ds ** 3

    def sampleReferenceLine(self, max_step = 5.0):
        """Sample the road's reference line along all geometries.

        Args:
            max_step: maximum distance between samples along the road (metres).

        Returns:
            A list of tuples (s, x, y, phi) where s is the global distance along
            the road reference line, x and y are world coordinates and phi is
            the orientation (heading).
        """
        samples = []
        for geom in self.geometries:
            geom_length = geom.geom_length
            # Determine number of segments for this geometry
            n = max(int(math.ceil(geom_length / max_step)) + 1, 2)
            positions = geom.samplePositions(n)
            for i, (x, y, phi) in enumerate(positions):
                s = geom.s + (geom.geom_length * i / (n - 1))
                samples.append((s, x, y, phi))
        return samples

class JunctionConnection:
    """Represents a connection between two roads inside a junction.

    Attributes:
        incoming_road_id: ID of the incoming road.
        connecting_road_id: ID of the connecting road.
        contact_point: 'start' or 'end', refers to which end of the connecting road
            touches the junction.
    """

    def __init__(self, incoming_road_id: str, connecting_road_id: str, contact_point: str):
        self.incoming_road_id = incoming_road_id
        self.connecting_road_id = connecting_road_id
        self.contact_point = contact_point.lower()

class OpenDrive:
    roads = None
    connections = None

    def __init__(self):
        self.roads = {}
        self.connections = []

    def addRoad(self, road):
        self.roads[road.id] = road

    def addConnection(self, connection):
        self.connections += connection

    def addConnections(self, connections):
        for connection in connections:
            self.addConnection(connection)