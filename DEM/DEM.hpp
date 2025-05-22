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

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>
#include <system_error>

using namespace std::experimental;

struct Coordinate {
    float latitude;
    float longitude;

    Coordinate()
        : latitude(0),
        longitude(0)
    {};

    Coordinate(float latitude, float longitude)
        : latitude(latitude),
        longitude(longitude)
    {
        if (latitude > 90 || latitude < -90 || longitude > 180 || longitude < -180) {
            std::string e = "invalid coordinates (" + std::to_string(latitude) +  ":" +  std::to_string(longitude) + ")";
            throw std::runtime_error(e);
        }
    };

    Coordinate(const Coordinate& o) = default;
    Coordinate& operator=(const Coordinate& o) = default;
    Coordinate(Coordinate&& o) noexcept = default;
    Coordinate& operator=(Coordinate&& o) noexcept = default;
    ~Coordinate() = default;

    bool operator<(const Coordinate& o) const {
        if (latitude == o.latitude) {
            return longitude < o.longitude;
        }
        return latitude < o.latitude;
    }

    bool operator==(const Coordinate& o) const {
        return latitude == o.latitude && longitude == o.longitude;
    }
};



struct Bounds {
    Coordinate NW;
    Coordinate NE;
    Coordinate SW;
    Coordinate SE;

    Bounds() = default;

    Bounds(const Coordinate& NW, const Coordinate& NE, const Coordinate& SW, const Coordinate& SE)
        : NW(NW),
        NE(NE),
        SW(SW),
        SE(SE)
    {};

    Bounds(const Bounds& o) = default;
    Bounds& operator=(const Bounds& o) = default;
    Bounds(Bounds&& o) noexcept = default;
    Bounds& operator=(Bounds&& o) noexcept = default;
    ~Bounds() = default;

    bool within(float latitude, float longitude) {
        if (
            latitude >= this->SW.latitude
            && latitude < this->NE.latitude
            && longitude >= this->SW.longitude
            && longitude < this->NE.longitude
        ) {
            return true;
        } else {
            return false;
        }
    }
};


/*
template <typename T>
concept dem_datatype =
    std::is_arithmetic_v<T> &&
    !std::is_same_v<T, bool> &&
    !std::is_same_v<T, char> &&
    !std::is_same_v<T, signed char> &&
    !std::is_same_v<T, unsigned char> &&
    !std::is_same_v<T, char16_t> &&
    !std::is_same_v<T, char32_t> &&
    !std::is_same_v<T, wchar_t>;
*/


template <typename T>
class DEM {
private:
    struct Index {
        float row;
        float column;
    };


    int16_t read(const filesystem::path& filepath) {
        union {T value; uint8_t bytes[sizeof(T)];} t{};

        auto serialize = [&t](T value) -> T {
            t.value = value;
            std::reverse(t.bytes, t.bytes + sizeof(T));
            return t.value;
        };

        std::ifstream fp(filepath, std::ios::binary);
        T t_value = 0;

        if (fp.good() && !fp.eof()) {
            size_t column_count = 0;
            std::vector<T> row_data;

            while (fp.read(reinterpret_cast<char*>(&t_value), sizeof(T))) {
                row_data.push_back(serialize(t_value));
                column_count++;

                if (column_count == this->type.ncols) {
                    this->data.push_back(row_data);
                    column_count = 0;
                    row_data.clear();
                }
            }
        } else {
            fp.close();
            return EXIT_FAILURE;
        }

        fp.close();
        return EXIT_SUCCESS;
    };


    Index index(float latitude, float longitude) {
        float dem_latitude_index = 0, dem_longitude_index = 0;

        if (this->bounds.within(latitude, longitude)) {
            if (latitude >= this->bounds.SW.latitude) {
                // Northern Hemisphere
                dem_latitude_index = (this->bounds.NE.latitude - latitude) / this->type.cellsize;
            } else {
                // Southern Hemisphere
                dem_latitude_index = (latitude - this->bounds.SW.latitude) / this->type.cellsize;
            }

            if (longitude >= this->bounds.SW.longitude) {
                // Eastern Hemisphere
                dem_longitude_index = (longitude - this->bounds.SW.longitude) / this->type.cellsize;
            } else {
                // Western Hemisphere
                dem_longitude_index = (this->bounds.NE.longitude - longitude) / this->type.cellsize;
            }
        } else {
            return {
                static_cast<float>(this->type.nodata),
                static_cast<float>(this->type.nodata)
            };
        }

        return {
            dem_latitude_index,
            dem_longitude_index
        };
    };


public:
    struct Type {
        size_t nrows;       // no. of DEM values available in row
        size_t ncols;       // no. of DEM values available in column
        float yllcorner;    // bottom left latitude
        float xllcorner;    // bottom left longitude
        float cellsize;     // distance (in radians) between every DEM values
        T nodata;           // invalid DEM value representation

        Type()
            : nrows(0),
            ncols(0),
            yllcorner(0),
            xllcorner(0),
            cellsize(0),
            nodata(0)
        {};

        Type (size_t nrows, size_t ncols, float yllcorner, float xllcorner, float cellsize, T nodata)
            : nrows(nrows),
            ncols(ncols),
            yllcorner(yllcorner),
            xllcorner(xllcorner),
            cellsize(cellsize),
            nodata(nodata)
         {
            if (nrows == 0 || ncols == 0) {
                throw std::runtime_error("invalid data dimensions, nrows = 0 & ncols = 0");
            }
            if (yllcorner > 90 || yllcorner < -90 || xllcorner > 180 || xllcorner < -180) {
                std::string e = "invalid coordinates (" + std::to_string(yllcorner) +  ":" +  std::to_string(xllcorner) + ")";
                throw std::runtime_error(e);
            }
        };

        Type(const Type& o) = default;
        Type& operator=(const Type& o) = default;
        Type(Type&& o) noexcept = default;
        Type& operator=(Type&& o) noexcept = default;
        ~Type() = default;
    };


    std::vector<std::vector<T>> data;
    Type type;
    Bounds bounds;


    DEM() = default;


    DEM(const Type& type, const filesystem::path& filepath) {
        this->type = type;
        this->bounds = {
            {this->type.yllcorner + (this->type.cellsize * this->type.nrows), this->type.xllcorner},
            {this->type.yllcorner + (this->type.cellsize * this->type.nrows), this->type.xllcorner + (this->type.cellsize * this->type.ncols)},
            {this->type.yllcorner, this->type.xllcorner},
            {this->type.yllcorner, this->type.xllcorner + (this->type.cellsize * this->type.ncols)}
        };

        if (!filesystem::exists(filepath)) {
            std::string e = "DEM file '" + filepath.string() + "' not found";
            throw std::runtime_error(e);
        }

        // read the DEM file (sets: this->data)
        if (this->read(filepath) != EXIT_SUCCESS) {
            std::string e = "failed to read DEM data from '" + filepath.string() + "'";
            throw std::runtime_error(e);
        }
    };


    DEM(const DEM& other) = default;
    DEM& operator=(const DEM& other) = default;
    DEM(DEM&& other) noexcept = default;
    DEM& operator=(DEM&& other) noexcept = default;
    ~DEM() = default;


    T altitude(float latitude, float longitude) {
        Index rc = this->index(latitude, longitude);

        if (rc.row == this->type.nodata || rc.column == this->type.nodata) {
            return this->type.nodata;
        }

        size_t r = static_cast<size_t>(std::round(rc.row));
        size_t c = static_cast<size_t>(std::round(rc.column));

        r = r == this->type.nrows ? r - 1 : r;
        c = c == this->type.ncols ? c - 1 : c;

        T altitude = this->data[r][c];

        return altitude;
    };


    float interpolated_altitude(float latitude, float longitude) {
        Index rc = this->index(latitude, longitude);

        if (rc.row == this->type.nodata || rc.column == this->type.nodata) {
            return this->type.nodata;
        }

        size_t r = static_cast<size_t>(rc.row);
        size_t c = static_cast<size_t>(rc.column);

        float del_latitude = std::min(rc.row, static_cast<float>(this->type.nrows-1)) - r;
        float del_longitude = std::min(rc.column, static_cast<float>(this->type.ncols-1)) - c;

        size_t next_r = (r == this->type.nrows-1) ? r : r + 1;
        size_t next_c = (c == this->type.ncols-1) ? c : c + 1;

        float altitude =   (1-del_latitude) * (1-del_longitude) * this->data[r][c] +
                            del_longitude * (1-del_latitude) * this->data[r][next_c] +
                            (1-del_longitude) * del_latitude * this->data[next_r][c] +
                            del_latitude * del_longitude * this->data[next_r][next_c];

        return altitude;
    };
};
