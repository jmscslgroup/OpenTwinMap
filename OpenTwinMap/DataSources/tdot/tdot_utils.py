import json


class TDOTUtils:
    feet_to_meters = 0.3048
    meters_to_feet = 3.28084
    dem_cell_size = 2.0
    dem_cell_delta = 1.0

    def __init__(self):
        pass

    @staticmethod
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
                result = result + TDOTUtils.getCoordinatePairs(coordinates[i])
            return result
        return []

    @staticmethod
    def getSouthWestCoordinate(coords):
        min_lat = min(coord[1] for coord in coords)
        candidates = [coord for coord in coords if coord[1] == min_lat]
        southwest = min(candidates, key=lambda x: x[0])
        return southwest

    @staticmethod
    def getNorthEastCoordinate(coords):
        max_lat = max(coord[1] for coord in coords)
        candidates = [coord for coord in coords if coord[1] == max_lat]
        northeast = max(candidates, key=lambda x: x[0])
        return northeast

    @staticmethod
    def loadJson(json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
        return data

    @staticmethod
    def writeJson(json_path, json_data):
        with open(json_path, "w+") as f:
            data = json.dump(json_data, f, indent=8)
