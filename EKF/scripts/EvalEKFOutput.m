function EvalEKFOutput(ekfOutputDir, mocapDataFile, quadDataFile)
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

  systemTime_s = systemTime*1e-6;

	% Map the covariance to (15, 15, n)
	covarianceEstimate = mapCovariance(covarianceFlat, 15); 

	% Load mocap data
	mocapData = readtable(mocapDataFile);

	% Time sync mocap + ekf
  fd = readtable(quadDataFile);
  offset = fd.quadTime - fd.timeSinceBoot(1) - fd.mocapTime;
	offset = rmoutliers(offset);
	offset = mean(offset);
  minTime = systemTime(1)*1e-6;
  maxTime = systemTime(end)*1e-6;

  [mocapTime, mocapX, mocapY, mocapZ, mocapQ] = ...
		processMocapRigid(mocapfile, 1, offset);
	mocapPos = [mocapX, mocapY, mocapZ];
	% Fix any non-unique points in the mocap times
	[mocapTime, iu] = unique(mocapTime);
	mocapPos = mocapPos(iu, :);
	mocapQ = mocapQ(iu, :);
	
	mocap_idx = find(mocapTime >= minTime & mocapTime <= maxTime);
	mocapTime_trunc = mocapTime(mocap_idx);
	mocapPos_trunc = mocapPos(mocap_idx, :);
	
	% Interpolate mocap_pos to align it with estimates
	mocapPos_trunc_xi = interp1(mocapTime_trunc, mocapPos_trunc(:,1), systemTime_s);
	mocapPos_trunc_yi = interp1(mocapTime_trunc, mocapPos_trunc(:,2), systemTime_s);
	mocapPos_trunc_zi = interp1(mocapTime_trunc, mocapPos_trunc(:,3), systemTime_s);
	
	% Compute position errors, RMSE, NIS, etc.
  errorX = mocapPos_trunc_xi - stateEstimate(:, 1);
	errorY = mocapPos_trunc_yi - stateEstimate(:, 2);
	errorZ = mocapPos_trunc_zi - stateEstimate(:, 3);
	
	rmseX = rms(errorX, "omitnan");
	rmseY = rms(errorY, "omitnan");
	rmseZ = rms(errorZ, "omitnan");

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
