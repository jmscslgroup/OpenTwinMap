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
#include <experimental/filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "DEM.hpp"

using namespace std::experimental;


template <typename T>
class Utility {
private:
    static T serialize(T value) {
        static union {T value; uint8_t bytes[sizeof(T)];} t{};
        t.value = value;
        std::reverse(t.bytes, t.bytes + sizeof(t.value));
        return t.value;
    }

    static void dynamic_metadata_skip(std::ifstream& ifp) {
        std::string line;
        std::streampos data_position;

        while (std::getline(ifp, line)) {
            line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));

            if (!line.empty() && std::isdigit(line[0])) {
                data_position = ifp.tellg();
                data_position -= line.size()+1;
                break;
            }
        }

        ifp.clear();
        ifp.seekg(data_position);
    }

    static filesystem::path generate_output_file_path(const filesystem::path& input_path, const std::string& extension) {
        filesystem::path p = input_path;
        return p.replace_extension("." + extension);
    }

public:
    static void create_dem_asc_bin(const filesystem::path& path) {
        if (!filesystem::exists(path)) {
            std::string e = "file '" + path.string() + "' not found";
            throw std::runtime_error(e);
        }

        std::vector<T> dem_data;
        T value = 0;
        std::ifstream ifp(path);
        dynamic_metadata_skip(ifp);

        // read values to `dem_data`
        while (ifp >> value) dem_data.push_back(serialize(value));
        ifp.close();

        // write `dem_data` in binary format
        std::ofstream ofp(generate_output_file_path(path, "bin"), std::ios::binary | std::ios::trunc);
        ofp.write(reinterpret_cast<char*>(dem_data.data()), dem_data.size() * sizeof(T));
        ofp.close();
    };


    static void create_dem_asc_csv(const filesystem::path& path, const typename DEM<T>::Type type) {
        if (!filesystem::exists(path)) {
            std::string e = "file '" + path.string() + "' not found";
            throw std::runtime_error(e);
        }

        std::vector<T> dem_data;
        T value = 0;
        std::ifstream ifp(path);
        dynamic_metadata_skip(ifp);

        // read values to `dem_data`
        while (ifp >> value) dem_data.push_back(value);
        ifp.close();

        // write as per '.csv' format
        std::ofstream ofp(generate_output_file_path(path, "csv"));
        for (size_t i = 0; i < type.nrows * type.ncols; ++i) {
            if (i % type.ncols == type.ncols - 1)
                ofp << dem_data[i] << "\n";
            else
                ofp << dem_data[i] << ",";
        }
        ofp.close();
    };


    static void create_dem_csv_bin(const filesystem::path& path){
        if (!filesystem::exists(path)) {
            std::string e = "file '" + path.string() + "' not found";
            throw std::runtime_error(e);
        }

        std::vector<T> dem_data;
        T value = 0;
        std::ifstream ifp(path);

        // read values to `dem_data`
        std::string line;
        while (std::getline(ifp, line)) {
            std::istringstream iss(line);
            std::string token;
            while (std::getline(iss, token, ','))
                dem_data.push_back(serialize(static_cast<T>(std::stod(token))));
        }
        ifp.close();

        // write `dem_data` in binary format
        std::ofstream ofp(generate_output_file_path(path, "bin"), std::ios::binary | std::ios::trunc);
        ofp.write(reinterpret_cast<char*>(dem_data.data()), dem_data.size() * sizeof(T));
        ofp.close();
    };


    static void create_dem_bin_csv(const filesystem::path& path, const typename DEM<T>::Type type) {
        if (!filesystem::exists(path)) {
            std::string e = "file '" + path.string() + "' not found";
            throw std::runtime_error(e);
        }

        std::vector<T> dem_data;
        T value = 0;
        std::ifstream ifp(path);

        // read from .bin file
        while (ifp.read(reinterpret_cast<char*>(&value), sizeof(T))) dem_data.push_back(serialize(value));
        ifp.close();

        // write as per '.csv' format
        std::ofstream ofp(generate_output_file_path(path, "csv"));
        for (size_t i = 0; i < type.nrows * type.ncols; ++i) {
            if (i % type.ncols == type.ncols - 1)
                ofp << dem_data[i] << "\n";
            else
                ofp << dem_data[i] << ",";
        }
        ofp.close();
    };
};
