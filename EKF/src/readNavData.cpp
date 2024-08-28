#include "readNavData.h"

#include <fstream>
#include <vector>

NavData::NavDataType NavData::LoadDatafile(const std::string &path) {
  // Read the file
  std::ifstream dataIn;
  dataIn.open(path);
  std::string line;
  std::vector<std::string> values;
  uint64_t numRows = 0;
  // First while collects stores each line into "line"
  // Second one gets the contents up to ','
  while (std::getline(dataIn, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(cell);
    }
    numRows++;
  }

  // Map the data to the right arrays
  NavDataType navData(numRows);
  uint8_t numColumns = values.size() / numRows;

  for (uint64_t row = 0; row < numRows; row++) {
    // Index of the first element of the row
    uint64_t firstIdx = row * numColumns;
    // The first three columns are accel data
    navData.accelData(row, 0) = std::stof(values[firstIdx]);
    navData.accelData(row, 1) = std::stof(values[firstIdx + 1]);
    navData.accelData(row, 2) = std::stof(values[firstIdx + 2]);
    // Next three are gyro data
    navData.gyroData(row, 0) = std::stof(values[firstIdx + 3]);
    navData.gyroData(row, 1) = std::stof(values[firstIdx + 4]);
    navData.gyroData(row, 2) = std::stof(values[firstIdx + 5]);
    // Position measurement
    navData.positionMeasurements(row, 0) = std::stof(values[firstIdx + 6]);
    navData.positionMeasurements(row, 1) = std::stof(values[firstIdx + 7]);
    navData.positionMeasurements(row, 2) = std::stof(values[firstIdx + 8]);
    // System time
    navData.systemTime_us(row) = std::stoul(values[firstIdx + 9]);
    // Measurement sequence number
    navData.measSeqNum(row) = std::stoul(values[firstIdx + 10]);
  }

  return navData;
}
