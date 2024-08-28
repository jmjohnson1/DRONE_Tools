#pragma once

#include <exception>
#include <fstream>
#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace NavData {
typedef struct NavDataType {
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> accelData;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gyroData;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> positionMeasurements;
  Eigen::Vector<uint64_t, Eigen::Dynamic> systemTime_us;
  Eigen::Vector<uint64_t, Eigen::Dynamic> measSeqNum;

  NavDataType(const uint64_t numberDataPoints)
      : accelData(numberDataPoints, 3),
        gyroData(numberDataPoints, 3),
        positionMeasurements(numberDataPoints, 3),
        systemTime_us(numberDataPoints),
        measSeqNum(numberDataPoints) {}
} NavDataType;

NavDataType LoadDatafile(const std::string& path);

// Exceptions for invalid matrix size
class not_square : public std::exception {
  virtual const char* what() const throw() { return "Matrix must be square"; }
};

class not_2d : public std::exception {
  virtual const char* what() const throw() {
    return "Matrix must be 2 dimensional";
  }
};

// DANGER: This function relies on a sort of hack relating to templates
// and the output argument. This is mentioned in the official Eigen
// documentation, but care should be taken if modifying this function.
// More info can be found here:
// https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigentuxfamilyTypes.html

/* @brief: Takes in a square matrix and returns the upper triangular part in a
 * row-major vector
 * @param inputMatrix: input matrix
 * @param-out outVec_: vector that will be written to
 */
template <typename DerivedSrc, typename DerivedDst>
void GetUpperTriangle(const Eigen::MatrixBase<DerivedSrc>& inputMatrix,
                      Eigen::MatrixBase<DerivedDst> const& outVec_) {
  // First check that the input is a 2d matrix
  try {
    if (inputMatrix.rows() < 2 || inputMatrix.cols() < 2) {
      throw not_2d();
    }
    if (inputMatrix.rows() != inputMatrix.cols()) {
      throw not_square();
    }
  } catch (std::exception& e) {
    std::cerr << "Error in GetUpperTriangle() - " << e.what() << std::endl;
  }

  const size_t matSize = inputMatrix.rows();
  const size_t numElements = (matSize * matSize - matSize) / 2 + matSize;

	// Check if outVec is a row or column vector
	bool rowVector = false;
	if (outVec_.cols() > 1) {
		rowVector = true;
	}

  // Cast away the const on outVec_. Yes, really.
  Eigen::MatrixBase<DerivedDst>& outVec =
      const_cast<Eigen::MatrixBase<DerivedDst>&>(outVec_);
	if (rowVector == true) {
		outVec.derived().resize(1, numElements);
	}
	else {
		outVec.derived().resize(numElements, 1);
	}

  size_t iterator = 0;
	if (rowVector == true) {
		for (size_t row = 0; row < matSize; row++) {
			for (size_t col = row; col < matSize; col++) {
				outVec(0, iterator) = inputMatrix(row, col);
				iterator++;
			}
		}
	} else {
		for (size_t row = 0; row < matSize; row++) {
			for (size_t col = row; col < matSize; col++) {
				outVec(iterator, 0) = inputMatrix(row, col);
				iterator++;
			}
		}
	}
}

template <typename T> 
void Vec2CSV(const std::string filepath, const std::vector<T>& vec) {
	std::ofstream outfile;
	outfile.open(filepath, std::ofstream::out);
	outfile << vec[0]; 
	for (size_t i = 1; i < vec.size(); i++) {
		outfile << "\n" << vec[i];
	}
}

}  // namespace NavData
