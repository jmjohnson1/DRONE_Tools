function EvalEKFOutput(ekfOutputDir, mocapDataFile)
	% TOOD: make docs

	ekfOutputDir = char(ekfOutputDir);
	mocapDataFile = char(mocapDataFile);
	
	% Load in CSV data from EKF run
	% Ensure directory has trailing '/'
	if ~strcmp(ekfOutputDir(end), '/')
		ekfOutputDir = [ekfOutputDir, '/'];
	end
	stateEstimate = readmatrix(fullpath([ekfOutputDir, 'stateEstimate.csv']));
	covarianceFlat = readmatrix(fullpath([ekfOutputDir, 'covariance.csv']));
	systemTime = readmatrix(fullpath([ekfOutputdir, 'systemTime_us.csv']));
	measAvail = readmatrix(fullpath([ekfOutputDir, 'measurementAvail.csv']));

	% Map the covariance to (15, 15, n)
	covarianceEstimate = mapCovariance(covarianceFlat, 15); 

	% Load mocap data
	mocapData = readtable(mocapDataFile);

	% Time sync mocap + ekf

	% Compute position errors, RMSE, NIS, etc.

	% Plot pos, vel, att estimates

	% Plot pos error with covariance bounds

	% Plot biases (with covariance bounds?)

end

function cov = MapCovariance(flatCov, numStates) 
	expectedWidth = (numStates^2 - numStates)/2 + numStates;
	if width(flatCov) ~= expectedWidth
		error("Flat covariance does not match expected width");
	end
	cov = zeros(numStates, numStates, height(flatCov));
	for timeIdx = 1:height(flatCov)
		iterator = 1;
		tempMat = zeros(numStates, numStates);
		% Upper triangle of covariance is stored in flatCov in row-major order
		for row = 1:numStates
			for col = row:numStates
				tempMat(row, col) = flatCov(timeIdx, iterator);
				iterator = iterator + 1;
			end
		end
		diagElems = diag(tempMat);
		cov(:, :, timeIdx) = tempMat + tempMat' - diag(diagElems);
	end
end
