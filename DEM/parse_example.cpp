#include <iostream>
#include <fstream>
#include "json.hpp"
using namespace std;
using json = nlohmann::json;

int main() {
	ifstream f("./metadata.json");
	json data = json::parse(f);
	cout << data << '\n';
	for (auto& [key, value] : data.items()) {
		cout << key << '\n';
		cout << "DEM PATH: " << value["DEM"]["path"] << '\n';
		double dem_x_min = value["DEM"]["x"]["min"];
		double dem_x_max = value["DEM"]["x"]["max"];
		double dem_y_min = value["DEM"]["y"]["min"];
		double dem_y_max = value["DEM"]["y"]["max"];
		cout << "DEM X MIN AND MAX: " << dem_x_min << " " << dem_x_max << '\n';
		cout << "DEM Y MIN AND MAX: " << dem_y_min << " " << dem_y_max << '\n';
	}
	return 0;
}
