#include <iostream>
#include <fstream>
#include "json.hpp"
#include "DEM/Utility.hpp"

using namespace std;
using json = nlohmann::json;

// 1000 rows each
// 1500 column values
// Nodata = -999999

int main() {
	ifstream f("./metadata.json");
	json data = json::parse(f);
	//cout << data << '\n';
	for (auto& element: data.items()) {
		string key = element.key();
		json value = element.value();
		if (key == "148110") {
			cout << key << '\n';
			cout << "DEM BIN PATH: " << value["DEM"]["bin_path"] << '\n';
            string bin_path = value["DEM"]["bin_path"];
			double dem_x_min = value["DEM"]["x"]["min"];
			double dem_x_max = value["DEM"]["x"]["max"];
			double dem_y_min = value["DEM"]["y"]["min"];
			double dem_y_max = value["DEM"]["y"]["max"];
			double dem_z_min = value["DEM"]["z"]["min"];
			double dem_z_max = value["DEM"]["z"]["max"];
			cout << "DEM X MIN AND MAX: " << dem_x_min << " " << dem_x_max << '\n';
			cout << "DEM Y MIN AND MAX: " << dem_y_min << " " << dem_y_max << '\n';
			ifstream bin_file(bin_path, ios::binary);
            DEM<double>::Type type = DEM<double>::Type(dem_y_max - dem_y_min, dem_x_max - dem_x_min, dem_y_min, dem_x_min, 2.0, -999999);
            DEM<double> dem = DEM<double>(type, bin_file);
            //cout << dem.altitude(dem_x_min + 2.0, dem_y_min + 2.0) << '\n';
            cout << dem.altitude(1764528.000, 633377.000) << '\n';
            for (double x = dem_x_min - 1.0; x <= (dem_x_max + 1.0); x += 0.1) {
                for (double y = dem_y_min - 1.0; y <= (dem_y_max + 1.0); y += 0.1) {
                    double alt = dem.interpolated_altitude(x, y);
                    if ((alt <= (dem_z_min - 1e-6)) || (alt >= (dem_z_max + 1e-6))) {
                        double alt = dem.interpolated_altitude(x, y, true);
                        cout << "Out of bounds " << dem.altitude(x, y) << ' ' << alt << ' ' << static_cast<size_t>(x) << ' ' << static_cast<size_t>(y) << '\n';
                        return 1;
                    }
                    //cout << x << ' ' << y << ' ' << dem.interpolated_altitude(x, y) << '\n';
                }
            }
            cout << "Pass through successful!" << '\n';
			break;
		}
	}
	return 0;
}
