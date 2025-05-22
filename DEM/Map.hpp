/*
MIT License

Copyright (c) 2023 Pritam Halder

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without
limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

Author : Pritam Halder
Email : pritamhalder.portfolio@gmail.com
*/

#pragma once

#include <bit>
#include <cstdint>
#include <experimental/filesystem>
#include <map>
#include <regex>
#include <string>

#include "DEM.hpp"

using namespace std::experimental;

template <typename T>
class Map {
public:
    using Grid = std::map<Coordinate, std::pair<typename DEM<T>::Type, filesystem::path>>;

    Map() = default;

    Map(const Grid& grid) {
        if (grid.empty()) {
            throw std::runtime_error("map grid is empty\n");
        }

        for (auto m = grid.cbegin(); m != grid.cend(); ++m) {
            filesystem::path filepath = m->second.second;

            if (!filesystem::exists(filepath)) {
                std::string e = "'" + filepath.string() + "' file doesn't exists\n";
                throw std::runtime_error(e);
            }
        }

        this->grid = grid;

        auto first_dem = this->grid.cbegin();
        this->dem = DEM<T>(this->grid[first_dem->first].first, this->grid[first_dem->first].second);
    };


    ~Map() = default;


    const DEM<T>& get_dem() const {
        return this->dem;
    }


    T altitude(float latitude, float longitude) {
        if (!this->dem.bounds.within(latitude, longitude)) {
            if (!this->load(latitude, longitude)) {
                return this->dem.type.nodata;
            }
        }

        return this->dem.altitude(latitude, longitude);
    };


    float interpolated_altitude(float latitude, float longitude) {
        if (!this->dem.bounds.within(latitude, longitude)) {
            if (!this->load(latitude, longitude)) {
                return this->dem.type.nodata;
            }
        }

        return this->dem.interpolated_altitude(latitude, longitude);
    };


    static Grid initialize(const filesystem::path& dem_directory_path, size_t nrows, size_t ncols, float cellsize, T nodata) {
        Map<T>::Grid grid;
        std::regex pattern(R"(([-]?\d{1,2}|90)_([-]?\d{1,3}|180)\.bin)");

        try {
            for (const auto& entry : filesystem::directory_iterator(dem_directory_path)) {
                if (filesystem::is_regular_file(entry.status())) {
                    std::string filename = entry.path().filename().string();
                    std::smatch match;

                    if (std::regex_match(filename, match, pattern)) {
                        float latitude = std::stod(match[1]);
                        float longitude = std::stod(match[2]);

                        if (
                            (latitude >= -90 && latitude <= 90)
                            && (longitude >= -180 && longitude <= 180)
                        ) {
                            typename DEM<T>::Type type(nrows, ncols, latitude, longitude, cellsize, nodata);
                            grid[{latitude, longitude}] = {type, entry.path().string()};
                        }
                    }
                }
            }
        } catch (const filesystem::filesystem_error& e) {
            throw std::runtime_error(std::string(e.what()) + "\n");
        }

        return grid;
    };


private:
    DEM<T> dem;
    Grid grid;


    bool load(float latitude, float longitude) {
        Coordinate grid_coordinate{
            std::floor(latitude),
            std::floor(longitude)
        };

        if (this->grid.count(grid_coordinate) == 0) return false;
        this->dem = DEM<T>(this->grid[grid_coordinate].first, this->grid[grid_coordinate].second);

        return true;
    };
};
