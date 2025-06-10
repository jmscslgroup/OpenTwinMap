import pandas
import numpy
import scipy.interpolate


class DEM:
    path = None
    height = None
    width = None
    y_origin = None
    x_origin = None
    grid_size = None
    step_size = None
    nodata_value = None
    csv_data = None
    grid_x = None
    grid_y = None
    interpolated_grid = None

    def __create_bottom_pad(self):
        entries_in_width = int(self.width / self.grid_size) + 1
        bottom_pad_values = self.csv_data.iloc[0:entries_in_width].z.tolist()
        bottom_pad_points = []
        for i in range(len(bottom_pad_values)):
            bottom_pad_points.append([self.x_origin + (i * self.grid_size), self.y_origin - self.grid_size])
        return bottom_pad_points, bottom_pad_values

    def __create_left_pad(self):
        entries_in_height = int(self.height / self.grid_size) + 1
        entries_in_width = int(self.width / self.grid_size) + 1
        left_pad_values = self.csv_data.iloc[0:int(entries_in_width * self.height):entries_in_width].z.tolist()
        left_pad_values = [left_pad_values[0]] + left_pad_values + [left_pad_values[-1]]
        left_pad_points = []
        for i in range(len(left_pad_values)):
            left_pad_points.append([self.x_origin - self.grid_size, self.y_origin - (self.grid_size / 2.0) + (i * self.grid_size)])
        return left_pad_points, left_pad_values

    def __create_top_pad(self):
        entries_in_height = int(self.height / self.grid_size) + 1
        entries_in_width = int(self.width / self.grid_size) + 1
        top_row_origin = entries_in_width * (entries_in_height - 1)
        top_pad_values = self.csv_data.iloc[top_row_origin:int(top_row_origin + entries_in_width)].z.tolist()
        top_pad_points = []
        for i in range(len(top_pad_values)):
            top_pad_points.append([self.x_origin + (i * self.grid_size), self.y_origin + self.height + self.grid_size])
        return top_pad_points, top_pad_values

    def __create_right_pad(self):
        entries_in_height = int(self.height / self.grid_size) + 1
        entries_in_width = int(self.width / self.grid_size) + 1
        right_pad_values = self.csv_data.iloc[(entries_in_width - 1):int(entries_in_width * self.height):entries_in_width].z.tolist()
        right_pad_values = [right_pad_values[0]] + right_pad_values + [right_pad_values[-1]]
        right_pad_points = []
        for i in range(len(right_pad_values)):
            right_pad_points.append([self.x_origin + self.width + self.grid_size, self.y_origin - (self.grid_size / 2.0) + (i * self.grid_size)])
        return right_pad_points, right_pad_values
    
    def __create_points(self, bottom, left, top, right):
        bottom = numpy.array(bottom)
        left = numpy.array(left)
        top = numpy.array(top)
        right = numpy.array(right)
        original_points = self.csv_data[["x", "y"]].to_numpy()
        new_points = numpy.append(numpy.append(numpy.append(bottom, left, axis=0), top, axis=0), right, axis=0)
        return numpy.append(original_points, new_points, axis=0)

    def __create_values(self, bottom, left, top, right):
        bottom = numpy.array(bottom)
        left = numpy.array(left)
        top = numpy.array(top)
        right = numpy.array(right)
        original_values = self.csv_data["z"].to_numpy()
        new_values = numpy.append(numpy.append(numpy.append(bottom, left, axis=0), top, axis=0), right, axis=0)
        return numpy.append(original_values, new_values, axis=0)

    def __generate_grid(self):
        bottom_pad_points, bottom_pad_values = self.__create_bottom_pad()
        left_pad_points, left_pad_values = self.__create_left_pad()
        top_pad_points, top_pad_values = self.__create_top_pad()
        right_pad_points, right_pad_values = self.__create_right_pad()
        x_bottom_left = self.x_origin - (self.grid_size / 2.0)
        x_top_right = self.x_origin + self.width + (self.grid_size / 2.0) + self.step_size
        y_bottom_left = self.y_origin - (self.grid_size / 2.0)
        y_top_right = self.y_origin + self.height + (self.grid_size / 2.0) + self.step_size
        points = self.__create_points(bottom_pad_points, left_pad_points, top_pad_points, right_pad_points)
        values = self.__create_values(bottom_pad_values, left_pad_values, top_pad_values, right_pad_values)
        self.grid_x, self.grid_y = numpy.mgrid[x_bottom_left:x_top_right:self.step_size, y_bottom_left:y_top_right:self.step_size]
        self.interpolated_grid = scipy.interpolate.griddata(points, values, (self.grid_x, self.grid_y), method='linear').T

    # dem_y_max - dem_y_min, dem_x_max - dem_x_min, dem_y_min, dem_x_min, 2.0, -999999
    def __init__(self, path, grid_size, step_size, nodata_value):
        self.path = path
        self.grid_size = grid_size
        self.step_size = step_size
        self.nodata_vlaue = nodata_value
        self.csv_data = pandas.read_csv(path, names=["x", "y", "z"], sep=" ").sort_values(by=["y", "x"], ascending=[True, True])
        self.x_origin = self.csv_data.iloc[0].x
        self.y_origin = self.csv_data.iloc[0].y
        self.height = self.csv_data.iloc[-1].y - self.y_origin
        self.width = self.csv_data.iloc[-1].x - self.x_origin
        self.__generate_grid()

    def altitude(self, x, y):
        x = round(x, 1)
        y = round(y, 1)
        x_bottom_left = self.x_origin - (self.grid_size / 2.0)
        x_top_right = self.x_origin + self.width + (self.grid_size / 2.0)
        y_bottom_left = self.y_origin - (self.grid_size / 2.0)
        y_top_right = self.y_origin + self.height + (self.grid_size / 2.0)
        x = max(x_bottom_left, min(x_top_right, x)) - self.x_origin
        y = max(y_bottom_left, min(y_top_right, y)) - self.y_origin
        x_index = int(x / self.step_size)
        y_index = int(y / self.step_size)
        return self.interpolated_grid[x_index][y_index]