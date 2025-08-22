from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Type, TypeVar, Dict
import math
import numpy as np
import xml.etree.ElementTree as ET

T = TypeVar("T")

def _set_attrib(element: ET.Element, name: str, value) -> None:
    """Helper to set an XML attribute only if value is not None."""
    if value is not None:
        element.set(name, str(value))


def _get_attrib(element: ET.Element, name: str, cast=lambda x: x, default=None):
    """Retrieve an XML attribute from an element, casting it if present."""
    val = element.get(name)
    if val is None:
        return default
    try:
        return cast(val)
    except Exception:
        # If casting fails, return raw string
        return val


def _find_children(element: ET.Element, tag: str) -> List[ET.Element]:
    """Return a list of direct child elements matching tag."""
    return [child for child in element if child.tag == tag]


@dataclass
class GeoReference:
    """Geographic projection reference contained inside ``<geoReference>``."""

    proj4: str

    def toXML(self) -> ET.Element:
        elem = ET.Element("geoReference")
        elem.text = self.proj4
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(proj4=element.text or "")


@dataclass
class Header:
    """
    Represents the ``<header>`` element.  Many optional attributes are
    supported.  Only the major and minor revision numbers are required.
    """

    revMajor: int
    revMinor: int
    name: Optional[str] = None
    version: Optional[float] = None
    date: Optional[str] = None
    north: Optional[float] = None
    south: Optional[float] = None
    east: Optional[float] = None
    west: Optional[float] = None
    vendor: Optional[str] = None
    geoReference: Optional[GeoReference] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("header")
        # required attributes
        elem.set("revMajor", str(self.revMajor))
        elem.set("revMinor", str(self.revMinor))
        # optional attributes
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "version", self.version)
        _set_attrib(elem, "date", self.date)
        _set_attrib(elem, "north", self.north)
        _set_attrib(elem, "south", self.south)
        _set_attrib(elem, "east", self.east)
        _set_attrib(elem, "west", self.west)
        _set_attrib(elem, "vendor", self.vendor)
        # child
        if self.geoReference is not None:
            elem.append(self.geoReference.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        if element is None:
            raise ValueError("Header element is required")
        geo_ref_elem = element.find("geoReference")
        geo_ref = GeoReference.fromXML(geo_ref_elem) if geo_ref_elem is not None else None
        return cls(
            revMajor=_get_attrib(element, "revMajor", int),
            revMinor=_get_attrib(element, "revMinor", int),
            name=_get_attrib(element, "name", str),
            version=_get_attrib(element, "version", float),
            date=_get_attrib(element, "date", str),
            north=_get_attrib(element, "north", float),
            south=_get_attrib(element, "south", float),
            east=_get_attrib(element, "east", float),
            west=_get_attrib(element, "west", float),
            vendor=_get_attrib(element, "vendor", str),
            geoReference=geo_ref,
        )


@dataclass
class PredecessorSuccessor:
    """
    Represents a common base for ``<predecessor>`` and ``<successor>``
    elements at the road level.  The ``elementType``, ``elementId`` and
    ``contactPoint`` attributes describe connectivity between roads.
    ``elementS`` and ``elementDir`` are optional and used for mid‑road
    merges and splits.
    """

    elementType: str
    elementId: str
    contactPoint: Optional[str] = None
    elementS: Optional[float] = None
    elementDir: Optional[str] = None

    tag: str = ""

    def toXML(self) -> ET.Element:
        elem = ET.Element(self.tag)
        elem.set("elementType", self.elementType)
        elem.set("elementId", self.elementId)
        _set_attrib(elem, "contactPoint", self.contactPoint)
        _set_attrib(elem, "elementS", self.elementS)
        _set_attrib(elem, "elementDir", self.elementDir)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            elementType=_get_attrib(element, "elementType", str),
            elementId=_get_attrib(element, "elementId", str),
            contactPoint=_get_attrib(element, "contactPoint", str),
            elementS=_get_attrib(element, "elementS", float),
            elementDir=_get_attrib(element, "elementDir", str),
        )


@dataclass
class Neighbor:
    """Neighbor road link in ``<link>`` section of a road."""
    side: str
    elementId: str
    direction: str

    def toXML(self) -> ET.Element:
        elem = ET.Element("neighbor")
        elem.set("side", self.side)
        elem.set("elementId", self.elementId)
        elem.set("direction", self.direction)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            side=_get_attrib(element, "side", str),
            elementId=_get_attrib(element, "elementId", str),
            direction=_get_attrib(element, "direction", str),
        )


@dataclass
class Link:
    """Represents the ``<link>`` element inside a road."""
    predecessor: Optional[PredecessorSuccessor] = None
    successor: Optional[PredecessorSuccessor] = None
    neighbors: List[Neighbor] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("link")
        if self.predecessor is not None:
            pred_elem = self.predecessor.toXML()
            pred_elem.tag = "predecessor"
            elem.append(pred_elem)
        if self.successor is not None:
            succ_elem = self.successor.toXML()
            succ_elem.tag = "successor"
            elem.append(succ_elem)
        for n in self.neighbors:
            elem.append(n.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        pred_elem = element.find("predecessor")
        succ_elem = element.find("successor")
        pred = PredecessorSuccessor.fromXML(pred_elem) if pred_elem is not None else None
        if pred is not None:
            pred.tag = "predecessor"
        succ = PredecessorSuccessor.fromXML(succ_elem) if succ_elem is not None else None
        if succ is not None:
            succ.tag = "successor"
        neighbors = [Neighbor.fromXML(ne) for ne in element.findall("neighbor")]
        return cls(predecessor=pred, successor=succ, neighbors=neighbors)


@dataclass
class Speed:
    """Defines a speed limit in either road ``<type>`` or lane context."""
    max: Optional[str] = None
    unit: Optional[str] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("speed")
        _set_attrib(elem, "max", self.max)
        _set_attrib(elem, "unit", self.unit)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            max=_get_attrib(element, "max", str),
            unit=_get_attrib(element, "unit", str),
        )


@dataclass
class RoadType:
    """Represents a ``<type>`` record within a road."""
    s: float
    type: str
    country: Optional[str] = None
    speed: Optional[Speed] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("type")
        elem.set("s", str(self.s))
        elem.set("type", self.type)
        _set_attrib(elem, "country", self.country)
        if self.speed is not None:
            elem.append(self.speed.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        sp_elem = element.find("speed")
        sp = Speed.fromXML(sp_elem) if sp_elem is not None else None
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            type=_get_attrib(element, "type", str, ""),
            country=_get_attrib(element, "country", str),
            speed=sp,
        )


@dataclass
class Line:
    """Geometry shape for a straight line segment."""

    def toXML(self) -> ET.Element:
        return ET.Element("line")

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls()


@dataclass
class Arc:
    """Geometry shape for a circular arc segment."""
    curvature: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("arc")
        elem.set("curvature", str(self.curvature))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(curvature=_get_attrib(element, "curvature", float, 0.0))


@dataclass
class Spiral:
    """Geometry shape for a clothoid spiral segment."""
    curvStart: float
    curvEnd: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("spiral")
        elem.set("curvStart", str(self.curvStart))
        elem.set("curvEnd", str(self.curvEnd))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            curvStart=_get_attrib(element, "curvStart", float, 0.0),
            curvEnd=_get_attrib(element, "curvEnd", float, 0.0),
        )


@dataclass
class Poly3:
    """Geometry shape for a cubic polynomial segment."""
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("poly3")
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class ParamPoly3:
    """Geometry shape for a parametric cubic polynomial segment."""
    aU: float
    bU: float
    cU: float
    dU: float
    aV: float
    bV: float
    cV: float
    dV: float
    pRange: Optional[str] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("paramPoly3")
        elem.set("aU", str(self.aU))
        elem.set("bU", str(self.bU))
        elem.set("cU", str(self.cU))
        elem.set("dU", str(self.dU))
        elem.set("aV", str(self.aV))
        elem.set("bV", str(self.bV))
        elem.set("cV", str(self.cV))
        elem.set("dV", str(self.dV))
        _set_attrib(elem, "pRange", self.pRange)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            aU=_get_attrib(element, "aU", float, 0.0),
            bU=_get_attrib(element, "bU", float, 0.0),
            cU=_get_attrib(element, "cU", float, 0.0),
            dU=_get_attrib(element, "dU", float, 0.0),
            aV=_get_attrib(element, "aV", float, 0.0),
            bV=_get_attrib(element, "bV", float, 0.0),
            cV=_get_attrib(element, "cV", float, 0.0),
            dV=_get_attrib(element, "dV", float, 0.0),
            pRange=_get_attrib(element, "pRange", str),
        )


@dataclass
class Geometry:
    """An individual geometry segment inside ``<planView>``."""
    s: float
    x: float
    y: float
    hdg: float
    length: float
    shape: object

    def toXML(self) -> ET.Element:
        elem = ET.Element("geometry")
        elem.set("s", str(self.s))
        elem.set("x", str(self.x))
        elem.set("y", str(self.y))
        elem.set("hdg", str(self.hdg))
        elem.set("length", str(self.length))
        elem.append(self.shape.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        # Determine shape type by reading first child tag
        shape_elem = next((child for child in element if child.tag in {"line", "arc", "spiral", "poly3", "paramPoly3"}), None)
        shape: object
        if shape_elem is None:
            raise ValueError("Geometry element must contain a shape child")
        if shape_elem.tag == "line":
            shape = Line.fromXML(shape_elem)
        elif shape_elem.tag == "arc":
            shape = Arc.fromXML(shape_elem)
        elif shape_elem.tag == "spiral":
            shape = Spiral.fromXML(shape_elem)
        elif shape_elem.tag == "poly3":
            shape = Poly3.fromXML(shape_elem)
        elif shape_elem.tag == "paramPoly3":
            shape = ParamPoly3.fromXML(shape_elem)
        else:
            raise ValueError(f"Unknown geometry shape: {shape_elem.tag}")
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            x=_get_attrib(element, "x", float, 0.0),
            y=_get_attrib(element, "y", float, 0.0),
            hdg=_get_attrib(element, "hdg", float, 0.0),
            length=_get_attrib(element, "length", float, 0.0),
            shape=shape,
        )
    
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
        if isinstance(self.shape, Line):
            # Straight line: phi is constant
            for i in range(num_samples):
                ds = (self.length) * i / (num_samples - 1)
                x = self.x + ds * math.cos(self.hdg)
                y = self.y + ds * math.sin(self.hdg)
                phi = self.hdg
                positions.append((x, y, phi))
        elif isinstance(self.shape, Arc):
            # Circular arc: curvature k defines radius = 1/k
            k = self.shape.curvature
            # avoid division by zero; treat near zero curvature as straight line
            if abs(k) < 1e-8:
                for i in range(num_samples):
                    ds = (self.length) * i / (num_samples - 1)
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
            raise NotImplementedError(f"Unsupported geometry type: {self.shape}")
        return np.array(positions)

    def projectXYToS(self, x, y):
        s_local = None
        dist = None
        if isinstance(self.shape, Line):
            # Direction vector
            dx = math.cos(self.hdg)
            dy = math.sin(self.hdg)
            # Projection of point onto the infinite line
            relx = x - self.x
            rely = y - self.y
            proj = relx * dx + rely * dy
            proj_clamped = max(0.0, min(self.length, proj))
            # Closest point coordinates
            cx = self.x + dx * proj_clamped
            cy = self.y + dy * proj_clamped
            # Distance from point to line
            dist = math.hypot(x - cx, y - cy)
            s_local = proj_clamped
        elif isinstance(self.shape, Arc):
            k = self.shape.curvature
            # Treat nearly zero curvature as a line
            if abs(k) < 1e-12:
                # Same as line
                dx = math.cos(self.hdg)
                dy = math.sin(self.hdg)
                relx = x - self.x
                rely = y - self.y
                proj = relx * dx + rely * dy
                proj_clamped = max(0.0, min(self.length, proj))
                cx = x0 + dx * proj_clamped
                cy = y0 + dy * proj_clamped
                dist = math.hypot(x - cx, y - cy)
                s_local = proj_clamped
            else:
                R = 1.0 / k
                # Normal vector to heading (left turn if k > 0)
                nx = -math.sin(self.hdg)
                ny = math.cos(self.hdg)
                # Center of curvature
                cx0 = self.x + nx * R
                cy0 = self.y + ny * R
                # Start angle of the arc
                phi_start = math.atan2(self.y - cy0, self.x - cx0)
                # Angular extent of the arc
                dphi = k * self.length
                # Angle of the point relative to center
                phi_point = math.atan2(y - cy0, x - cx0)
                # Compute difference and clamp to arc range
                # Normalize to [-pi, pi]
                delta = (phi_point - phi_start + math.pi) % (2 * math.pi) - math.pi
                # Clamp delta into [0, dphi] if dphi >= 0 else [dphi, 0]
                if dphi >= 0.0:
                    delta_clamped = max(0.0, min(dphi, delta))
                else:
                    delta_clamped = min(0.0, max(dphi, delta))
                phi_clamped = phi_start + delta_clamped
                # Closest point on arc
                cx = cx0 + R * math.cos(phi_clamped)
                cy = cy0 + R * math.sin(phi_clamped)
                # Distance from point to arc
                dist = math.hypot(x - cx, y - cy)
                # s along arc = delta_clamped / k
                s_local = delta_clamped / k
        else:
            raise NotImplementedError(f"Unsupported geometry type: {self.shape}")
        return self.s + s_local, dist

@dataclass
class PlanView:
    """Container for a sequence of geometry segments."""
    geometries: List[Geometry] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("planView")
        for g in self.geometries:
            elem.append(g.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        geoms = [Geometry.fromXML(ge) for ge in element.findall("geometry")]
        return cls(geometries=geoms)

    def sampleReferenceLine(self, resolution):
        """Sample the planView's reference line along all geometries.

        Args:
            resolution: maximum distance between samples along the road (metres).

        Returns:
            A list of tuples (s, x, y, phi) where s is the global distance along
            the road reference line, x and y are world coordinates and phi is
            the orientation (heading).
        """
        samples = []
        for geom in self.geometries:
            geom_length = geom.length
            # Determine number of segments for this geometry
            n = max(int(math.ceil(geom_length / resolution)) + 1, 2)
            positions = geom.samplePositions(n)
            for i, (x, y, phi) in enumerate(positions):
                s = geom.s + (geom_length * i / (n - 1))
                samples.append((s, x, y, phi))
        return np.array(samples)

@dataclass
class Elevation:
    """Elevation profile segment (cubic polynomial)."""
    s: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("elevation")
        elem.set("s", str(self.s))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class ElevationProfile:
    """Container for a list of elevation entries."""
    elevations: List[Elevation] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("elevationProfile")
        for e in self.elevations:
            elem.append(e.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            elevations=[Elevation.fromXML(e) for e in element.findall("elevation")]
        )


@dataclass
class Superelevation:
    """Superelevation entry for lateral profile."""
    s: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("superelevation")
        elem.set("s", str(self.s))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class Crossfall:
    """Crossfall entry for lateral profile."""
    side: str
    s: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("crossfall")
        elem.set("side", self.side)
        elem.set("s", str(self.s))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            side=_get_attrib(element, "side", str, "both"),
            s=_get_attrib(element, "s", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class Shape:
    """Defines an arbitrary vertical shape (third order polynomial) in lateral profile."""
    s: float
    t: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("shape")
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            t=_get_attrib(element, "t", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class LateralProfile:
    """Container for superelevation, crossfall and shape entries."""
    superelevations: List[Superelevation] = field(default_factory=list)
    crossfalls: List[Crossfall] = field(default_factory=list)
    shapes: List[Shape] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("lateralProfile")
        for se in self.superelevations:
            elem.append(se.toXML())
        for cf in self.crossfalls:
            elem.append(cf.toXML())
        for sh in self.shapes:
            elem.append(sh.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            superelevations=[Superelevation.fromXML(e) for e in element.findall("superelevation")],
            crossfalls=[Crossfall.fromXML(e) for e in element.findall("crossfall")],
            shapes=[Shape.fromXML(e) for e in element.findall("shape")],
        )


@dataclass
class LaneOffset:
    """Represents a polynomial lateral offset of the lane coordinate system."""
    s: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("laneOffset")
        elem.set("s", str(self.s))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class LaneLink:
    """Maps lanes across road segments within lane context."""
    predecessor: List[int] = field(default_factory=list)
    successor: List[int] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("link")
        # multiple predecessor ids
        for pid in self.predecessor:
            pre = ET.Element("predecessor")
            pre.set("id", str(pid))
            elem.append(pre)
        for sid in self.successor:
            suc = ET.Element("successor")
            suc.set("id", str(sid))
            elem.append(suc)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        preds = [int(p.get("id")) for p in element.findall("predecessor")] if element is not None else []
        succs = [int(s.get("id")) for s in element.findall("successor")] if element is not None else []
        return cls(predecessor=preds, successor=succs)


@dataclass
class Width:
    """Defines lane width as a cubic polynomial along its length."""
    sOffset: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("width")
        elem.set("sOffset", str(self.sOffset))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class Border:
    """Defines a lane border as a cubic polynomial."""
    sOffset: float
    a: float
    b: float
    c: float
    d: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("border")
        elem.set("sOffset", str(self.sOffset))
        elem.set("a", str(self.a))
        elem.set("b", str(self.b))
        elem.set("c", str(self.c))
        elem.set("d", str(self.d))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            a=_get_attrib(element, "a", float, 0.0),
            b=_get_attrib(element, "b", float, 0.0),
            c=_get_attrib(element, "c", float, 0.0),
            d=_get_attrib(element, "d", float, 0.0),
        )


@dataclass
class RoadMark:
    """Defines a road marking on a lane boundary."""
    sOffset: float
    type: str
    weight: Optional[str] = None
    color: Optional[str] = None
    material: Optional[str] = None
    width: Optional[float] = None
    laneChange: Optional[str] = None
    height: Optional[float] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("roadMark")
        elem.set("sOffset", str(self.sOffset))
        elem.set("type", self.type)
        _set_attrib(elem, "weight", self.weight)
        _set_attrib(elem, "color", self.color)
        _set_attrib(elem, "material", self.material)
        _set_attrib(elem, "width", self.width)
        _set_attrib(elem, "laneChange", self.laneChange)
        _set_attrib(elem, "height", self.height)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            type=_get_attrib(element, "type", str, ""),
            weight=_get_attrib(element, "weight", str),
            color=_get_attrib(element, "color", str),
            material=_get_attrib(element, "material", str),
            width=_get_attrib(element, "width", float),
            laneChange=_get_attrib(element, "laneChange", str),
            height=_get_attrib(element, "height", float),
        )


@dataclass
class Material:
    """Specifies a lane surface material."""
    sOffset: float
    surface: str
    friction: Optional[float] = None
    roughness: Optional[float] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("material")
        elem.set("sOffset", str(self.sOffset))
        elem.set("surface", self.surface)
        _set_attrib(elem, "friction", self.friction)
        _set_attrib(elem, "roughness", self.roughness)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            surface=_get_attrib(element, "surface", str, ""),
            friction=_get_attrib(element, "friction", float),
            roughness=_get_attrib(element, "roughness", float),
        )


@dataclass
class Visibility:
    """Defines lane visibility distances."""
    sOffset: float
    forward: Optional[float] = None
    back: Optional[float] = None
    left: Optional[float] = None
    right: Optional[float] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("visibility")
        elem.set("sOffset", str(self.sOffset))
        _set_attrib(elem, "forward", self.forward)
        _set_attrib(elem, "back", self.back)
        _set_attrib(elem, "left", self.left)
        _set_attrib(elem, "right", self.right)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            forward=_get_attrib(element, "forward", float),
            back=_get_attrib(element, "back", float),
            left=_get_attrib(element, "left", float),
            right=_get_attrib(element, "right", float),
        )


@dataclass
class LaneSpeed:
    """Specifies a lane specific speed limit."""
    sOffset: float
    max: float
    unit: Optional[str] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("speed")
        elem.set("sOffset", str(self.sOffset))
        elem.set("max", str(self.max))
        _set_attrib(elem, "unit", self.unit)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            max=_get_attrib(element, "max", float, 0.0),
            unit=_get_attrib(element, "unit", str),
        )


@dataclass
class Access:
    """Specifies an access restriction for a lane."""
    sOffset: float
    rule: str
    restriction: str

    def toXML(self) -> ET.Element:
        elem = ET.Element("access")
        elem.set("sOffset", str(self.sOffset))
        elem.set("rule", self.rule)
        elem.set("restriction", self.restriction)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            rule=_get_attrib(element, "rule", str, "allow"),
            restriction=_get_attrib(element, "restriction", str, ""),
        )


@dataclass
class HeightRecord:
    """Defines a lane height offset (inner and outer) relative to road reference."""
    sOffset: float
    inner: float
    outer: float

    def toXML(self) -> ET.Element:
        elem = ET.Element("height")
        elem.set("sOffset", str(self.sOffset))
        elem.set("inner", str(self.inner))
        elem.set("outer", str(self.outer))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            inner=_get_attrib(element, "inner", float, 0.0),
            outer=_get_attrib(element, "outer", float, 0.0),
        )


@dataclass
class LaneRule:
    """Free‑text rule for a lane."""
    sOffset: float
    value: str

    def toXML(self) -> ET.Element:
        elem = ET.Element("rule")
        elem.set("sOffset", str(self.sOffset))
        elem.set("value", self.value)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            sOffset=_get_attrib(element, "sOffset", float, 0.0),
            value=_get_attrib(element, "value", str, ""),
        )


@dataclass
class Lane:
    """
    Represents an individual lane (child of left/center/right in a lane section).
    Only a subset of the attributes and child records are implemented here.
    """
    id: int
    type: Optional[str] = None
    level: Optional[bool] = None
    link: Optional[LaneLink] = None
    widths: List[Width] = field(default_factory=list)
    borders: List[Border] = field(default_factory=list)
    roadMarks: List[RoadMark] = field(default_factory=list)
    materials: List[Material] = field(default_factory=list)
    visibilities: List[Visibility] = field(default_factory=list)
    speeds: List[LaneSpeed] = field(default_factory=list)
    accesses: List[Access] = field(default_factory=list)
    heights: List[HeightRecord] = field(default_factory=list)
    rules: List[LaneRule] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("lane")
        elem.set("id", str(self.id))
        if self.type is not None:
            elem.set("type", self.type)
        if self.level is not None:
            elem.set("level", "true" if self.level else "false")
        # link
        if self.link is not None:
            elem.append(self.link.toXML())
        # widths / borders / roadMarks / materials / visibility / speed / access / height / rule
        for w in self.widths:
            elem.append(w.toXML())
        for b in self.borders:
            elem.append(b.toXML())
        for rm in self.roadMarks:
            elem.append(rm.toXML())
        for m in self.materials:
            elem.append(m.toXML())
        for v in self.visibilities:
            elem.append(v.toXML())
        for s in self.speeds:
            elem.append(s.toXML())
        for a in self.accesses:
            elem.append(a.toXML())
        for h in self.heights:
            elem.append(h.toXML())
        for r in self.rules:
            elem.append(r.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        lane_id = _get_attrib(element, "id", int, 0)
        lane_type = _get_attrib(element, "type", str, None)
        level_attr = element.get("level")
        level = None
        if level_attr is not None:
            level = True if level_attr.lower() == "true" else False
        # link (inside lane) optional
        link_elem = element.find("link")
        link = LaneLink.fromXML(link_elem) if link_elem is not None else None
        widths = [Width.fromXML(w) for w in element.findall("width")]
        borders = [Border.fromXML(b) for b in element.findall("border")]
        roadMarks = [RoadMark.fromXML(rm) for rm in element.findall("roadMark")]
        materials = [Material.fromXML(m) for m in element.findall("material")]
        visibilities = [Visibility.fromXML(v) for v in element.findall("visibility")]
        speeds = [LaneSpeed.fromXML(s) for s in element.findall("speed")]
        accesses = [Access.fromXML(a) for a in element.findall("access")]
        heights = [HeightRecord.fromXML(h) for h in element.findall("height")]
        rules = [LaneRule.fromXML(r) for r in element.findall("rule")]
        return cls(
            id=lane_id,
            type=lane_type,
            level=level,
            link=link,
            widths=widths,
            borders=borders,
            roadMarks=roadMarks,
            materials=materials,
            visibilities=visibilities,
            speeds=speeds,
            accesses=accesses,
            heights=heights,
            rules=rules,
        )


@dataclass
class LaneGroup:
    """Represents a grouping of lanes on a particular side (left, center, right)."""
    lanes: List[Lane] = field(default_factory=list)

    def toXML(self, tag: str) -> ET.Element:
        elem = ET.Element(tag)
        for lane in self.lanes:
            elem.append(lane.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(lanes=[Lane.fromXML(l) for l in element.findall("lane")])


@dataclass
class LaneSection:
    """Represents a ``<laneSection>`` inside the lanes container."""
    s: float
    singleSide: Optional[bool] = None
    left: Optional[LaneGroup] = None
    center: Optional[LaneGroup] = None
    right: Optional[LaneGroup] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("laneSection")
        elem.set("s", str(self.s))
        if self.singleSide is not None:
            elem.set("singleSide", "true" if self.singleSide else "false")
        if self.left is not None and self.left.lanes:
            elem.append(self.left.toXML("left"))
        if self.center is not None and self.center.lanes:
            elem.append(self.center.toXML("center"))
        if self.right is not None and self.right.lanes:
            elem.append(self.right.toXML("right"))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        single_side_attr = element.get("singleSide")
        single_side = None
        if single_side_attr is not None:
            single_side = True if single_side_attr.lower() == "true" else False
        left_elem = element.find("left")
        center_elem = element.find("center")
        right_elem = element.find("right")
        left_group = LaneGroup.fromXML(left_elem) if left_elem is not None else None
        center_group = LaneGroup.fromXML(center_elem) if center_elem is not None else None
        right_group = LaneGroup.fromXML(right_elem) if right_elem is not None else None
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            singleSide=single_side,
            left=left_group,
            center=center_group,
            right=right_group,
        )


@dataclass
class Lanes:
    """Container for lane sections and lane offsets."""
    laneOffsets: List[LaneOffset] = field(default_factory=list)
    laneSections: List[LaneSection] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("lanes")
        for lo in self.laneOffsets:
            elem.append(lo.toXML())
        for ls in self.laneSections:
            elem.append(ls.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        offsets = [LaneOffset.fromXML(o) for o in element.findall("laneOffset")]
        sections = [LaneSection.fromXML(ls) for ls in element.findall("laneSection")]
        return cls(laneOffsets=offsets, laneSections=sections)


@dataclass
class Validity:
    """Defines validity of a signal or object for a lane range."""
    fromLane: int
    toLane: int

    def toXML(self) -> ET.Element:
        elem = ET.Element("validity")
        elem.set("fromLane", str(self.fromLane))
        elem.set("toLane", str(self.toLane))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            fromLane=_get_attrib(element, "fromLane", int, 0),
            toLane=_get_attrib(element, "toLane", int, 0),
        )


@dataclass
class Dependency:
    """Defines a dependency between signals."""
    id: str
    type: Optional[str] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("dependency")
        elem.set("id", self.id)
        _set_attrib(elem, "type", self.type)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            id=_get_attrib(element, "id", str, ""),
            type=_get_attrib(element, "type", str),
        )


@dataclass
class Signal:
    """
    Represents a traffic signal or sign placed along a road.
    Many optional attributes are supported.
    """
    s: float
    t: float
    id: str
    name: Optional[str] = None
    dynamic: Optional[str] = None
    orientation: Optional[str] = None
    zOffset: Optional[float] = None
    country: Optional[str] = None
    countryRevision: Optional[str] = None
    type: Optional[str] = None
    subtype: Optional[str] = None
    value: Optional[float] = None
    unit: Optional[str] = None
    height: Optional[float] = None
    width: Optional[float] = None
    text: Optional[str] = None
    hOffset: Optional[float] = None
    pitch: Optional[float] = None
    roll: Optional[float] = None
    validities: List[Validity] = field(default_factory=list)
    dependencies: List[Dependency] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("signal")
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("id", self.id)
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "dynamic", self.dynamic)
        _set_attrib(elem, "orientation", self.orientation)
        _set_attrib(elem, "zOffset", self.zOffset)
        _set_attrib(elem, "country", self.country)
        _set_attrib(elem, "countryRevision", self.countryRevision)
        _set_attrib(elem, "type", self.type)
        _set_attrib(elem, "subtype", self.subtype)
        _set_attrib(elem, "value", self.value)
        _set_attrib(elem, "unit", self.unit)
        _set_attrib(elem, "height", self.height)
        _set_attrib(elem, "width", self.width)
        _set_attrib(elem, "text", self.text)
        _set_attrib(elem, "hOffset", self.hOffset)
        _set_attrib(elem, "pitch", self.pitch)
        _set_attrib(elem, "roll", self.roll)
        for v in self.validities:
            elem.append(v.toXML())
        for d in self.dependencies:
            elem.append(d.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            t=_get_attrib(element, "t", float, 0.0),
            id=_get_attrib(element, "id", str, ""),
            name=_get_attrib(element, "name", str),
            dynamic=_get_attrib(element, "dynamic", str),
            orientation=_get_attrib(element, "orientation", str),
            zOffset=_get_attrib(element, "zOffset", float),
            country=_get_attrib(element, "country", str),
            countryRevision=_get_attrib(element, "countryRevision", str),
            type=_get_attrib(element, "type", str),
            subtype=_get_attrib(element, "subtype", str),
            value=_get_attrib(element, "value", float),
            unit=_get_attrib(element, "unit", str),
            height=_get_attrib(element, "height", float),
            width=_get_attrib(element, "width", float),
            text=_get_attrib(element, "text", str),
            hOffset=_get_attrib(element, "hOffset", float),
            pitch=_get_attrib(element, "pitch", float),
            roll=_get_attrib(element, "roll", float),
            validities=[Validity.fromXML(v) for v in element.findall("validity")],
            dependencies=[Dependency.fromXML(d) for d in element.findall("dependency")],
        )


@dataclass
class SignalReference:
    """Represents a reference to another signal defined on another road."""
    s: float
    t: float
    id: str
    orientation: Optional[str] = None
    validities: List[Validity] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("signalReference")
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("id", self.id)
        _set_attrib(elem, "orientation", self.orientation)
        for v in self.validities:
            elem.append(v.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            t=_get_attrib(element, "t", float, 0.0),
            id=_get_attrib(element, "id", str, ""),
            orientation=_get_attrib(element, "orientation", str),
            validities=[Validity.fromXML(v) for v in element.findall("validity")],
        )


@dataclass
class Signals:
    """Container for signals and signal references in a road."""
    signals: List[Signal] = field(default_factory=list)
    references: List[SignalReference] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("signals")
        for s in self.signals:
            elem.append(s.toXML())
        for r in self.references:
            elem.append(r.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            signals=[Signal.fromXML(s) for s in element.findall("signal")],
            references=[SignalReference.fromXML(r) for r in element.findall("signalReference")],
        )


@dataclass
class Object:
    """Represents a static or dynamic object placed along a road."""
    type: str
    subtype: Optional[str] = None
    dynamic: Optional[str] = None
    name: Optional[str] = None
    id: Optional[str] = None
    s: Optional[float] = None
    t: Optional[float] = None
    zOffset: Optional[float] = None
    validLength: Optional[float] = None
    orientation: Optional[str] = None
    length: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    radius: Optional[float] = None
    hdg: Optional[float] = None
    pitch: Optional[float] = None
    roll: Optional[float] = None
    validities: List[Validity] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("object")
        elem.set("type", self.type)
        _set_attrib(elem, "subtype", self.subtype)
        _set_attrib(elem, "dynamic", self.dynamic)
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "id", self.id)
        _set_attrib(elem, "s", self.s)
        _set_attrib(elem, "t", self.t)
        _set_attrib(elem, "zOffset", self.zOffset)
        _set_attrib(elem, "validLength", self.validLength)
        _set_attrib(elem, "orientation", self.orientation)
        _set_attrib(elem, "length", self.length)
        _set_attrib(elem, "width", self.width)
        _set_attrib(elem, "height", self.height)
        _set_attrib(elem, "radius", self.radius)
        _set_attrib(elem, "hdg", self.hdg)
        _set_attrib(elem, "pitch", self.pitch)
        _set_attrib(elem, "roll", self.roll)
        for v in self.validities:
            elem.append(v.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            type=_get_attrib(element, "type", str, ""),
            subtype=_get_attrib(element, "subtype", str),
            dynamic=_get_attrib(element, "dynamic", str),
            name=_get_attrib(element, "name", str),
            id=_get_attrib(element, "id", str),
            s=_get_attrib(element, "s", float),
            t=_get_attrib(element, "t", float),
            zOffset=_get_attrib(element, "zOffset", float),
            validLength=_get_attrib(element, "validLength", float),
            orientation=_get_attrib(element, "orientation", str),
            length=_get_attrib(element, "length", float),
            width=_get_attrib(element, "width", float),
            height=_get_attrib(element, "height", float),
            radius=_get_attrib(element, "radius", float),
            hdg=_get_attrib(element, "hdg", float),
            pitch=_get_attrib(element, "pitch", float),
            roll=_get_attrib(element, "roll", float),
            validities=[Validity.fromXML(v) for v in element.findall("validity")],
        )


@dataclass
class ObjectReference:
    """Represents a reference to an object defined on another road."""
    s: float
    t: float
    id: str
    zOffset: Optional[float] = None
    validLength: Optional[float] = None
    orientation: Optional[str] = None
    validities: List[Validity] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("objectReference")
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("id", self.id)
        _set_attrib(elem, "zOffset", self.zOffset)
        _set_attrib(elem, "validLength", self.validLength)
        _set_attrib(elem, "orientation", self.orientation)
        for v in self.validities:
            elem.append(v.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            s=_get_attrib(element, "s", float, 0.0),
            t=_get_attrib(element, "t", float, 0.0),
            id=_get_attrib(element, "id", str, ""),
            zOffset=_get_attrib(element, "zOffset", float),
            validLength=_get_attrib(element, "validLength", float),
            orientation=_get_attrib(element, "orientation", str),
            validities=[Validity.fromXML(v) for v in element.findall("validity")],
        )


@dataclass
class Objects:
    """Container for objects and object references on a road."""
    objects: List[Object] = field(default_factory=list)
    references: List[ObjectReference] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("objects")
        for o in self.objects:
            elem.append(o.toXML())
        for r in self.references:
            elem.append(r.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            objects=[Object.fromXML(o) for o in element.findall("object")],
            references=[ObjectReference.fromXML(r) for r in element.findall("objectReference")],
        )


@dataclass
class LaneLinkJunction:
    """Represents a lane to lane mapping inside a junction connection."""
    fromId: int
    toId: int

    def toXML(self) -> ET.Element:
        elem = ET.Element("laneLink")
        elem.set("from", str(self.fromId))
        elem.set("to", str(self.toId))
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            fromId=_get_attrib(element, "from", int, 0),
            toId=_get_attrib(element, "to", int, 0),
        )


@dataclass
class Connection:
    """Represents a junction connection element."""
    id: str
    incomingRoad: Optional[str] = None
    connectingRoad: Optional[str] = None
    contactPoint: Optional[str] = None
    type: Optional[str] = None
    laneLinks: List[LaneLinkJunction] = field(default_factory=list)
    predecessor: Optional[PredecessorSuccessor] = None
    successor: Optional[PredecessorSuccessor] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("connection")
        elem.set("id", self.id)
        _set_attrib(elem, "incomingRoad", self.incomingRoad)
        _set_attrib(elem, "connectingRoad", self.connectingRoad)
        _set_attrib(elem, "contactPoint", self.contactPoint)
        _set_attrib(elem, "type", self.type)
        for ll in self.laneLinks:
            elem.append(ll.toXML())
        # Predecessor/successor for virtual junctions
        if self.predecessor is not None:
            pre = self.predecessor.toXML()
            pre.tag = "predecessor"
            elem.append(pre)
        if self.successor is not None:
            suc = self.successor.toXML()
            suc.tag = "successor"
            elem.append(suc)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        lane_links = [LaneLinkJunction.fromXML(ll) for ll in element.findall("laneLink")]
        pred_elem = element.find("predecessor")
        succ_elem = element.find("successor")
        pred = PredecessorSuccessor.fromXML(pred_elem) if pred_elem is not None else None
        if pred is not None:
            pred.tag = "predecessor"
        succ = PredecessorSuccessor.fromXML(succ_elem) if succ_elem is not None else None
        if succ is not None:
            succ.tag = "successor"
        return cls(
            id=_get_attrib(element, "id", str, ""),
            incomingRoad=_get_attrib(element, "incomingRoad", str),
            connectingRoad=_get_attrib(element, "connectingRoad", str),
            contactPoint=_get_attrib(element, "contactPoint", str),
            type=_get_attrib(element, "type", str),
            laneLinks=lane_links,
            predecessor=pred,
            successor=succ,
        )


@dataclass
class Junction:
    """Defines an intersection of roads."""
    id: str
    name: Optional[str] = None
    type: Optional[str] = None
    connections: List[Connection] = field(default_factory=list)
    controllers: List['Controller'] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("junction")
        elem.set("id", self.id)
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "type", self.type)
        for c in self.connections:
            elem.append(c.toXML())
        for ctrl in self.controllers:
            elem.append(ctrl.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        connections = [Connection.fromXML(c) for c in element.findall("connection")]
        controllers = [Controller.fromXML(ct) for ct in element.findall("controller")]
        return cls(
            id=_get_attrib(element, "id", str, ""),
            name=_get_attrib(element, "name", str),
            type=_get_attrib(element, "type", str),
            connections=connections,
            controllers=controllers,
        )


@dataclass
class Control:
    """Links a controller to a signal."""
    signalId: str
    type: Optional[str] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("control")
        elem.set("signalId", self.signalId)
        _set_attrib(elem, "type", self.type)
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            signalId=_get_attrib(element, "signalId", str, ""),
            type=_get_attrib(element, "type", str),
        )


@dataclass
class Controller:
    """Defines a group of signals that operate together."""
    id: str
    name: Optional[str] = None
    sequence: Optional[int] = None
    controls: List[Control] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("controller")
        elem.set("id", self.id)
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "sequence", self.sequence)
        for c in self.controls:
            elem.append(c.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        return cls(
            id=_get_attrib(element, "id", str, ""),
            name=_get_attrib(element, "name", str),
            sequence=_get_attrib(element, "sequence", int),
            controls=[Control.fromXML(c) for c in element.findall("control")],
        )


@dataclass
class Road:
    """Defines a single road with its geometry, lanes, signals, objects and type records."""
    id: str
    length: float
    name: Optional[str] = None
    junction: Optional[str] = "-1"
    rule: Optional[str] = None
    link: Optional[Link] = None
    types: List[RoadType] = field(default_factory=list)
    planView: Optional[PlanView] = None
    elevationProfile: Optional[ElevationProfile] = None
    lateralProfile: Optional[LateralProfile] = None
    lanes: Optional[Lanes] = None
    signals: Optional[Signals] = None
    objects: Optional[Objects] = None

    def toXML(self) -> ET.Element:
        elem = ET.Element("road")
        elem.set("id", self.id)
        elem.set("length", str(self.length))
        _set_attrib(elem, "name", self.name)
        _set_attrib(elem, "junction", self.junction)
        _set_attrib(elem, "rule", self.rule)
        # link
        if self.link is not None:
            elem.append(self.link.toXML())
        # type records
        for t in self.types:
            elem.append(t.toXML())
        # planView
        if self.planView is not None:
            elem.append(self.planView.toXML())
        # elevationProfile
        if self.elevationProfile is not None:
            elem.append(self.elevationProfile.toXML())
        # lateralProfile
        if self.lateralProfile is not None:
            elem.append(self.lateralProfile.toXML())
        # lanes
        if self.lanes is not None:
            elem.append(self.lanes.toXML())
        # signals
        if self.signals is not None:
            elem.append(self.signals.toXML())
        # objects
        if self.objects is not None:
            elem.append(self.objects.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        link_elem = element.find("link")
        link = Link.fromXML(link_elem) if link_elem is not None else None
        type_elems = element.findall("type")
        types = [RoadType.fromXML(t) for t in type_elems]
        plan_view_elem = element.find("planView")
        plan_view = PlanView.fromXML(plan_view_elem) if plan_view_elem is not None else None
        elev_elem = element.find("elevationProfile")
        elev = ElevationProfile.fromXML(elev_elem) if elev_elem is not None else None
        lat_elem = element.find("lateralProfile")
        lat = LateralProfile.fromXML(lat_elem) if lat_elem is not None else None
        lanes_elem = element.find("lanes")
        lanes = Lanes.fromXML(lanes_elem) if lanes_elem is not None else None
        signals_elem = element.find("signals")
        signals = Signals.fromXML(signals_elem) if signals_elem is not None else None
        objects_elem = element.find("objects")
        objects = Objects.fromXML(objects_elem) if objects_elem is not None else None
        return cls(
            id=_get_attrib(element, "id", str, ""),
            length=_get_attrib(element, "length", float, 0.0),
            name=_get_attrib(element, "name", str),
            junction=_get_attrib(element, "junction", str),
            rule=_get_attrib(element, "rule", str),
            link=link,
            types=types,
            planView=plan_view,
            elevationProfile=elev,
            lateralProfile=lat,
            lanes=lanes,
            signals=signals,
            objects=objects,
        )


@dataclass
class OpenDRIVE:
    """
    Top level container for an OpenDRIVE map.  Contains a header, roads,
    junctions and controllers.
    """
    header: Header
    roads: Dict[Road] = field(default_factory=dict)
    junctions: List[Junction] = field(default_factory=list)
    controllers: List[Controller] = field(default_factory=list)

    def toXML(self) -> ET.Element:
        elem = ET.Element("OpenDRIVE")
        # header must come first
        elem.append(self.header.toXML())
        # roads
        for r in self.roads:
            elem.append(self.roads[r].toXML())
        # junctions
        for j in self.junctions:
            elem.append(j.toXML())
        # controllers not contained within junctions
        for c in self.controllers:
            elem.append(c.toXML())
        return elem

    @classmethod
    def fromXML(cls: Type[T], element: ET.Element) -> T:
        if element.tag != "OpenDRIVE":
            raise ValueError("Expected OpenDRIVE root element")
        header_elem = element.find("header")
        if header_elem is None:
            raise ValueError("OpenDRIVE element must contain <header>")
        header = Header.fromXML(header_elem)
        roads = {}
        for r in element.findall("road"):
            road = Road.fromXML(r)
            roads[road.id] = road
        junctions = [Junction.fromXML(j) for j in element.findall("junction")]
        controllers = [Controller.fromXML(c) for c in element.findall("controller")]
        return cls(header=header, roads=roads, junctions=junctions, controllers=controllers)