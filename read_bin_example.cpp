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
		auto value = element.value();
		if (key == "148110") {
			cout << key << '\n';
			cout << "DEM BIN PATH: " << value["DEM"]["bin_path"] << '\n';
            string bin_path = value["DEM"]["bin_path"];
			float dem_x_min = value["DEM"]["x"]["min"];
			float dem_x_max = value["DEM"]["x"]["max"];
			float dem_y_min = value["DEM"]["y"]["min"];
			float dem_y_max = value["DEM"]["y"]["max"];
			cout << "DEM X MIN AND MAX: " << dem_x_min << " " << dem_x_max << '\n';
			cout << "DEM Y MIN AND MAX: " << dem_y_min << " " << dem_y_max << '\n';
			ifstream bin_file(bin_path, ios::binary);
            DEM<float>::Type type = DEM<float>::Type(dem_y_max - dem_y_min, dem_x_max - dem_x_min, dem_y_min, dem_x_min, 2.0, -999999);
            DEM<float> dem = DEM<float>(type, bin_file);
            //cout << dem.altitude(dem_x_min + 2.0, dem_y_min + 2.0) << '\n';
            cout << dem.altitude(1764528.000, 633377.000 ) << '\n';
			break;
		}
	}
	return 0;
}
