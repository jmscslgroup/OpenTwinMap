import pandas


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
    def __init__(self, data, grid_size, nodata_value):
        self.grid_size = grid_size
        self.nodata_value = nodata_value
        self.csv_data = data
        self.x_origin = self.csv_data.iloc[0].x
        self.y_origin = self.csv_data.iloc[0].y
        self.x_bottom_left = self.x_origin - (self.grid_size / 2.0)
        self.y_bottom_left = self.y_origin - (self.grid_size / 2.0)
        self.height = self.csv_data.iloc[-1].y - self.y_origin
        self.width = self.csv_data.iloc[-1].x - self.x_origin
        self.last_height_index = int(self.height / self.grid_size)
        self.last_width_index = int(self.width / self.grid_size)
        self.width_count = self.last_width_index + 1
        self.original_x_bottom_left = self.x_bottom_left
        self.original_y_bottom_left = self.y_bottom_left

    @classmethod
    def load_csv_data(cls, path):
        return pandas.read_csv(path, names=["x", "y", "z"], sep=" ").sort_values(
            by=["y", "x"], ascending=[True, True]
        )

    @classmethod
    def from_csv(cls, path, grid_size, nodata_value):
        csv_data = cls.load_csv_data(path)
        return cls(csv_data, grid_size, nodata_value)

    @classmethod
    def from_dems(cls, dems, grid_size, nodata_value):
        dem_series = [dem.csv_data for dem in dems]
        csv_data = pandas.concat(dem_series, ignore_index=True).sort_values(
            by=["y", "x"], ascending=[True, True]
        )
        return cls(csv_data, grid_size, nodata_value)

    @classmethod
    # Default margin of 10 meters
    def clip_dem(cls, dem, bottom_left, top_right, margins=32.8084):
        float_bottom_x, float_bottom_y = (
            bottom_left[0] - dem.x_origin
        ) / dem.grid_size, (bottom_left[1] - dem.y_origin) / dem.grid_size
        float_top_x, float_top_y = (top_right[0] - dem.x_origin) / dem.grid_size, (
            top_right[1] - dem.y_origin
        ) / dem.grid_size
        bottom_x, bottom_y = dem._clip_indices(
            int(float_bottom_x - margins), int(float_bottom_y - margins)
        )
        top_x, top_y = dem._clip_indices(
            int(float_top_x + margins), int(float_top_y + margins)
        )
        indices = dem._compute_dem_indices(bottom_x, bottom_y, top_x, top_y)
        clipped_dem_data = dem.csv_data.iloc[indices].copy()
        result = DEM(clipped_dem_data, dem.grid_size, dem.nodata_value)
        result.original_x_bottom_left = dem.original_x_bottom_left
        result.original_y_bottom_left = dem.original_y_bottom_left
        return result

    def _clip_indices(self, x, y):
        return max(0, min(self.last_width_index, x)), max(
            0, min(self.last_height_index, y)
        )

    def _compute_dem_index(self, x, y):
        return (y * self.width_count) + x

    def _compute_dem_indices(self, bottom_x, bottom_y, top_x, top_y):
        indices = []
        for j in range(bottom_y, top_y + 1):
            for i in range(bottom_x, top_x + 1):
                indices.append(self._compute_dem_index(i, j))
        return indices

    def altitude(self, x, y):
        float_x = (x - self.x_origin) / self.grid_size
        float_y = (y - self.y_origin) / self.grid_size
        x = int(float_x)
        y = int(float_y)
        bottom_left_x, bottom_left_y = self._clip_indices(x, y)
        top_left_x, top_left_y = self._clip_indices(x, y + 1)
        bottom_right_x, bottom_right_y = self._clip_indices(x + 1, y)
        top_right_x, top_right_y = self._clip_indices(x + 1, y + 1)
        bottom_left_index = self._compute_dem_index(bottom_left_x, bottom_left_y)
        top_left_index = self._compute_dem_index(top_left_x, top_left_y)
        bottom_right_index = self._compute_dem_index(bottom_right_x, bottom_right_y)
        top_right_index = self._compute_dem_index(top_right_x, top_right_y)
        del_x, del_y = (float_x - x), (float_y - y)
        bottom_left_value = (
            (1.0 - del_x) * (1.0 - del_y) * self.csv_data.iloc[bottom_left_index].z
        )
        top_left_value = (1.0 - del_x) * del_y * self.csv_data.iloc[top_left_index].z
        bottom_right_value = (
            del_x * (1.0 - del_y) * self.csv_data.iloc[bottom_right_index].z
        )
        top_right_value = del_x * del_y * self.csv_data.iloc[top_right_index].z

        return bottom_left_value + top_left_value + bottom_right_value + top_right_value

    def get_min(self):
        return self.csv_data.z.min()

    def get_max(self):
        return self.csv_data.z.max()
