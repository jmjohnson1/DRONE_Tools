#pragma once

#include "Eigen/Dense"
#include <vector>
#include <fstream>

using namespace Eigen;

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

/**
 * @brief Writes the contents of an Eigen matrix to a CSV file
 * @param name: filename to write to
 * @param mat: matrix to write
*/
template<typename Derived>
void write_csv(std::string name, const Eigen::MatrixBase<Derived>& mat) {
	std::ofstream file(name.c_str());
	if (file.is_open()) {
		file << mat.format(CSVFormat);
	}
	file.close();
}

/**
* @brief Reads CSV file and stores it in an Eigen matrix
* @param path: Path to the CSV file
*/
template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<float> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}
