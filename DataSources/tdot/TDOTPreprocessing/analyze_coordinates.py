import numpy as np
from shapely.geometry import Polygon
from shapely.affinity import scale
from pyproj import Transformer
import json

transformer = Transformer.from_crs("EPSG:4326", "EPSG:6576", always_xy=True)

target_json = "./metadata.json"
subset_json = "./compile_subset.json"

feet_to_meters = 0.3048
meters_to_feet = 1.0 / feet_to_meters
tiles_x = 3
tiles_y = 3
tile_width = 2000 * feet_to_meters
tile_height = 3000 * feet_to_meters


def get_metadata():
    with open(target_json, "r") as f:
        data = json.load(f)
    return data


def get_subset():
    with open(subset_json, "r") as f:
        data = json.load(f)
    return data


metadata = get_metadata()
subset = get_subset()


def PolyArea(x, y):
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def getCoordinatePairs(coordinates):
    total_entries = len(coordinates)
    if total_entries <= 0:
        return []
    if type(coordinates[0]) is float:
        if total_entries != 2:
            raise Exception("Weird pairs - {}".format(str(coordinates)))
        return [coordinates]
    if type(coordinates[0]) is list:
        result = []
        for i in range(total_entries):
            result = result + getCoordinatePairs(coordinates[i])
        return result


for tile in subset["tiles"]:
    # proj = pyproj.Proj(proj="utm", zone=16, ellps="WGS84")
    coords = metadata[str(tile)]["GEOJSON"]["coordinates"]
    coords = getCoordinatePairs(coords)
    coords_feet = [transformer.transform(coord[0], coord[1]) for coord in coords]
    # coords_feet = (np.array(coords_meters) * meters_to_feet).tolist()
    # print(coords_feet)
    coords_feet_x = [coord[0] for coord in coords_feet]
    coords_feet_y = [coord[1] for coord in coords_feet]
    # print(PolyArea(coords_feet_x, coords_feet_y))
    polygon = Polygon(zip(coords_feet_x, coords_feet_y))
    # print(polygon.area)

    # Get current width and height
    minx, miny, maxx, maxy = polygon.bounds
    width = maxx - minx
    height = maxy - miny

    new_width = width + (0.85)
    new_height = height + (0.85)

    # Compute scale factors
    scale_x = new_width / width
    scale_y = new_height / height

    # Scale about the center
    polygon_scaled = scale(polygon, xfact=scale_x, yfact=scale_y, origin="center")
    # print(polygon_scaled)
    print(polygon.area)
    print(polygon_scaled.area)
