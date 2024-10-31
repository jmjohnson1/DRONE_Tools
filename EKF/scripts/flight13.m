clear; close all; clc;

ekfInputsDir = '../../EKFInputs/';
ekfOutputsDir = '../../EKFOutputs/';
ekfExe = '../build/runEKF';
dataDir = '../../Data_Aug_3_24/';

% Get the flight13 data
quadDataFile = fullfile(dataDir, 'flight_data13.csv');
quadMocapFile = fullfile(dataDir, 'mocap', 'mocap_8_3_24_b.csv');

% Prepare the flight data using the MPU6050
outFile = fullfile(ekfInputsDir, 'flight_data13_processed_mpu6050.csv');
WriteNavData(quadDataFile, outFile, 'mpu6050');


% Run the EKF with the MPU6050
status = system([ekfExe, ' ', outFile, ' ', ekfOutputsDir]);
if status ~= 0
    error("EKF execution failed");
end

EvalEKFOutput(ekfOutputsDir, quadMocapFile, quadDataFile);

% Prepare the flight data using the BMI088
outFile = fullfile(ekfInputsDir, 'flight_data13_processed_bmi088.csv');
WriteNavData(quadDataFile, outFile, 'bmi088');

% Run the EKF with the BMI088
status = system([ekfExe, ' ', outFile, ' ', ekfOutputsDir]);
if status ~= 0
    error("EKF execution failed");
end

EvalEKFOutput(ekfOutputsDir, quadMocapFile, quadDataFile);