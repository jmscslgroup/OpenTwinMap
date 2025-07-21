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
    union {double value; uint8_t bytes[sizeof(double)];} t{};

    auto serialize = [&t](double value) -> double {
        t.value = value;
        std::reverse(t.bytes, t.bytes + sizeof(double));
        return t.value;
    };
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
			double dem_x_min = value["DEM"]["x"]["min"];
			double dem_x_max = value["DEM"]["x"]["max"];
			double dem_y_min = value["DEM"]["y"]["min"];
			double dem_y_max = value["DEM"]["y"]["max"];
			cout << "DEM X MIN AND MAX: " << dem_x_min << " " << dem_x_max << '\n';
			cout << "DEM Y MIN AND MAX: " << dem_y_min << " " << dem_y_max << '\n';
			ifstream bin_file(bin_path, ios::binary);
            for (int i = 0; i < 10; i++) {
                double t_value;
                bin_file.read(reinterpret_cast<char*>(&t_value), sizeof(double));
                t_value = serialize(t_value);
                cout << t_value << '\n';
            }
			break;
		}
	}
	return 0;
}
