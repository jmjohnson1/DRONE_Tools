#include <iostream>
#include <vector>
#include <string>

#include "EKF.h"
#include "eigencsv.h"
#include "readNavData.h"

#ifdef DEBUG
#define DEBUG_PRINT(MSG) std::cout << "DEBUG: " << (MSG) << std::endl;
#else
#define DEBUG_PRINT(MSG)
#endif

int main(int argc, char *argv[]) {
  // User must input path to data
  if (argc < 2) {
    std::cerr << "Missing path to data file" << std::endl;
    return 1;
  }
	if (argc < 3) {
		std::cerr << "Missing path to output directory" << std::endl;
		return 1;
	}
  std::string datafile = argv[1];
  std::cout << "Datafile path: " << datafile << std::endl;
	std::string outDir = argv[2];
	if (outDir.compare("/") != 0) {
		outDir.push_back('/');
	}
	std::cout << "Output directory: " << outDir << std::endl;

  // Read from datafile
  NavData::NavDataType navData = NavData::LoadDatafile(datafile);
  const size_t navDataSize = navData.systemTime_us.rows();
  std::cout << "NavDataSize: " << navDataSize << std::endl;

  // Storage variables
  // stateEstimate includes the 15 states
  Eigen::MatrixXf stateEstimate(navDataSize, 15);
  // Covariance estimate includes upper triangular portion of covariance matrix.
  // Mapping is row major
  Eigen::MatrixXf covarianceEstimate(navDataSize, 120);
	std::vector<bool> measurementAvailable(navDataSize);

  // Create EKF object. Configure will populate various matrices, so make sure
  // to set any noise parameters before calling.
  EKF ekf;
  ekf.Configure();

  // Initialize with first instance of data
  ekf.Initialize(navData.gyroData.row(0), navData.accelData.row(0),
                 navData.positionMeasurements.row(0).cast<double>());

  NavData::GetUpperTriangle(ekf.Get_CovFull(), covarianceEstimate.row(0));
	stateEstimate.row(0) = ekf.Get_State();
	measurementAvailable[0] = false;

  // Run through the full data set
  for (size_t i = 1; i < navDataSize; i++) {
    ekf.Update(navData.systemTime_us(i), navData.measurementSequenceNumber(i),
               navData.gyroData.row(i), navData.accelData.row(i),
               navData.positionMeasurements.row(i).cast<double>());
		stateEstimate.row(i) = ekf.Get_State();
		NavData::GetUpperTriangle(ekf.Get_CovFull(), covarianceEstimate.row(i));
		measurementAvailable[i] = navData.measurementSequenceNumber(i) > navData.measurementSequenceNumber(i - 1);
  }

	// Print run data to CSV files
	write_csv(outDir + "stateEstimate.csv", stateEstimate);
	write_csv(outDir + "covariance.csv", covarianceEstimate);
	write_csv(outDir + "systemTime_us.csv", navData.systemTime_us);
	NavData::Vec2CSV(outDir + "measurementAvail.csv", measurementAvailable);

  return 0;
}
