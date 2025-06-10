import pandas
import numpy

class DEM:
    path = None
    height = None
    width = None
    y_origin = None
    x_origin = None
    grid_size = None
    nodata_value = None
    csv_data = None

    # dem_y_max - dem_y_min, dem_x_max - dem_x_min, dem_y_min, dem_x_min, 2.0, -999999
    def __init__(self, path, grid_size, nodata_value):
        self.path = path
        self.grid_size = grid_size
        self.nodata_vlaue = nodata_value
        self.csv_data = pandas.read_csv(path, names=["x", "y", "z"], sep=" ").sort_values(by=["y", "x"], ascending=[True, True])
        self.x_origin = self.csv_data.iloc[0].x
        self.y_origin = self.csv_data.iloc[0].y
        self.height = self.csv_data.iloc[-1].y - self.y_origin
        self.width = self.csv_data.iloc[-1].x - self.x_origin

    def altitude(self, x, y):
        last_height_index = int(self.height / self.grid_size)
        last_width_index = width_count = int(self.width / self.grid_size)
        width_count = last_width_index + 1
        float_x = (x - self.x_origin) / self.grid_size
        float_y = (y - self.y_origin) / self.grid_size
        x = int(float_x)
        y = int(float_y)
        bottom_left_x, bottom_left_y = max(0, min(last_width_index, x)), max(0, min(last_height_index, y))
        top_left_x, top_left_y = max(0, min(last_width_index, x)), max(0, min(last_height_index, y + 1))
        bottom_right_x, bottom_right_y = max(0, min(last_width_index, x + 1)), max(0, min(last_height_index, y))
        top_right_x, top_right_y = max(0, min(last_width_index, x + 1)), max(0, min(last_height_index, y + 1))
        bottom_left_index = (bottom_left_y * width_count) + bottom_left_x
        top_left_index = (top_left_y * width_count) + top_left_x
        bottom_right_index = (bottom_right_y * width_count) + bottom_right_x
        top_right_index = (top_right_y * width_count) + top_right_x
        del_x, del_y = (float_x - x), (float_y - y)
        bottom_left_value = (1.0 - del_x) * (1.0 - del_y) * self.csv_data.iloc[bottom_left_index].z
        top_left_value = (1.0 - del_x) * del_y * self.csv_data.iloc[top_left_index].z
        bottom_right_value = del_x * (1.0 - del_y) * self.csv_data.iloc[bottom_right_index].z
        top_right_value = del_x * del_y * self.csv_data.iloc[top_right_index].z

        return bottom_left_value + top_left_value + bottom_right_value + top_right_value