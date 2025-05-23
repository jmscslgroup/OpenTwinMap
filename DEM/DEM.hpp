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

/*
MIT License
Copyright (c) 2025 Alex Richardson @ Vanderbilt University

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

Author: Alex Richardson @ Vanderbilt University. 
Email: william.a.richardson@vanderbilt.edu
*/

#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>
#include <system_error>


struct Coordinate {
    float x;
    float y;

    Coordinate()
        : x(0),
        y(0)
    {};

    Coordinate(float x, float y)
        : x(x),
        y(y)
    {};

    Coordinate(const Coordinate& o) = default;
    Coordinate& operator=(const Coordinate& o) = default;
    Coordinate(Coordinate&& o) noexcept = default;
    Coordinate& operator=(Coordinate&& o) noexcept = default;
    ~Coordinate() = default;

    bool operator<(const Coordinate& o) const {
        if (y == o.y) {
            return x < o.x;
        }
        return y < o.y;
    }

    bool operator==(const Coordinate& o) const {
        return x == o.x && y == o.y;
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

    bool within(float x, float y) {
        //std::cout << x << ' ' << y << ' ' << this->SW.x << ' ' << this->SW.y << ' ' << this->NW.x << ' ' << this->NW.y << ' ' << this->SE.x << ' ' << this->SE.y << ' ' << this->NE.x << ' ' << this->NE.y << '\n';
        //std::cout << x - this->SW.x << ' ' << y - this->SW.y << ' ' << this->NE.y - y << '\n';
        if (
            y >= this->SW.y
            && y <= this->NE.y
            && x >= this->SW.x
            && x <= this->NE.x
        ) {
            return true;
        } else {
            return false;
        }
    }
};

template <typename T>
class DEM {
private:
    struct Index {
        int32_t row;
        int32_t column;
    };


    int16_t read(std::ifstream& fp) {
        auto serialize = [](T value) -> T {
            union {T value; uint8_t bytes[sizeof(T)];} t{};
            t.value = value;
            std::reverse(t.bytes, t.bytes + sizeof(T));
            return t.value;
        };

        if (fp.good() && !fp.eof()) {
            while (fp.peek() != std::ifstream::traits_type::eof()) {
                T x;
                T y;
                T z;
                fp.read(reinterpret_cast<char*>(&x), sizeof(T));
                fp.read(reinterpret_cast<char*>(&y), sizeof(T));
                fp.read(reinterpret_cast<char*>(&z), sizeof(T));
                x = serialize(x);
                y = serialize(y);
                z = serialize(z);
                Index rc = this->index(x, y);
                size_t r = static_cast<size_t>(rc.row);
                size_t c = static_cast<size_t>(rc.column);
                //std::cout << x << ' ' << y << ' ' << z << ' ' << this->data.size() << ' ' << rc.row << ' ' << rc.column << '\n';
                this->data[r][c] = z;
            }
        } else {
            fp.close();
            return EXIT_FAILURE;
        }

        fp.close();
        return EXIT_SUCCESS;
    };


    Index index(float x, float y) {
        int32_t dem_y_index = 0, dem_x_index = 0;

        if (this->bounds.within(x, y)) {
            dem_y_index = std::floor((y - this->bounds.SW.y) / this->type.cellsize);
            dem_x_index = std::floor((x - this->bounds.SW.x) / this->type.cellsize);
        } else {
            return {
                static_cast<int32_t>(this->type.nodata),
                static_cast<int32_t>(this->type.nodata)
            };
        }

        return {
            dem_y_index,
            dem_x_index
        };
    };


public:
    struct Type {
        size_t nrows; // no. of DEM values available in row
        size_t ncols; // no. of DEM values available in column
        float y_height; // total height of y
        float x_height; // total height of x
        float yllcorner;    // bottom left y
        float xllcorner;    // bottom left x
        float cellsize;     // distance (in feet) between every DEM values
        T nodata;           // invalid DEM value representation

        Type()
            : nrows(0),
            ncols(0),
            y_height(0),
            x_height(0),
            yllcorner(0),
            xllcorner(0),
            cellsize(0),
            nodata(0)
        {};

        Type (float y_height, float x_height, float yllcorner, float xllcorner, float cellsize, T nodata)
            : y_height(y_height),
            x_height(x_height),
            yllcorner(yllcorner),
            xllcorner(xllcorner),
            cellsize(cellsize),
            nodata(nodata)
         {
            nrows = std::ceil(y_height / cellsize) + 1;
            ncols = std::ceil(x_height / cellsize) + 1;
            if (y_height == 0 || x_height == 0) {
                throw std::runtime_error("invalid data dimensions, y_height = 0 & x_height = 0");
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


    DEM(const Type& type, std::ifstream& fp) {
        this->type = type;
        this->bounds = {
            {this->type.xllcorner, this->type.yllcorner + this->type.y_height},
            {this->type.xllcorner + this->type.x_height, this->type.yllcorner + this->type.y_height},
            {this->type.xllcorner, this->type.yllcorner},
            {this->type.xllcorner + this->type.x_height, this->type.yllcorner}
        };

        this->data = std::vector<std::vector<T>>(this->type.nrows, std::vector<T>(this->type.ncols, this->type.nodata));
        // read the DEM file (sets: this->data)
        if (this->read(fp) != EXIT_SUCCESS) {
            std::string e = "failed to read DEM data from fp";
            throw std::runtime_error(e);
        }
    };


    DEM(const DEM& other) = default;
    DEM& operator=(const DEM& other) = default;
    DEM(DEM&& other) noexcept = default;
    DEM& operator=(DEM&& other) noexcept = default;
    ~DEM() = default;


    T altitude(float x, float y) {
        Index rc = this->index(x, y);

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


    float interpolated_altitude(float x, float y) {
        Index rc = this->index(x, y);

        if (rc.row == this->type.nodata || rc.column == this->type.nodata) {
            return this->type.nodata;
        }

        size_t r = static_cast<size_t>(rc.row);
        size_t c = static_cast<size_t>(rc.column);

        float del_y = std::min(rc.row, static_cast<float>(this->type.nrows-1)) - r;
        float del_x = std::min(rc.column, static_cast<float>(this->type.ncols-1)) - c;

        size_t next_r = (r == this->type.nrows-1) ? r : r + 1;
        size_t next_c = (c == this->type.ncols-1) ? c : c + 1;

        float altitude =   (1-del_y) * (1-del_x) * this->data[r][c] +
                            del_x * (1-del_y) * this->data[r][next_c] +
                            (1-del_x) * del_y * this->data[next_r][c] +
                            del_y * del_x * this->data[next_r][next_c];

        return altitude;
    };
};
