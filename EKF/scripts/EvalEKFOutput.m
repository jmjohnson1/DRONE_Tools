function EvalEKFOutput(ekfOutputDir, mocapDataFile, quadDataFile)
  % TOOD: make docs

  ekfOutputDir = char(ekfOutputDir);
  mocapDataFile = char(mocapDataFile);
  
  % Load in CSV data from EKF run
  % Ensure directory has trailing '/'
  if ~strcmp(ekfOutputDir(end), '/')
    ekfOutputDir = [ekfOutputDir, '/'];
  end
  stateEstimate  = readmatrix(fullfile([ekfOutputDir, 'stateEstimate.csv']), Delimiter=',');
  covarianceFlat = readmatrix(fullfile([ekfOutputDir, 'covariance.csv']), Delimiter=',');
  systemTime     = readmatrix(fullfile([ekfOutputDir, 'systemTime_us.csv']), Delimiter=',');
  measAvail      = readmatrix(fullfile([ekfOutputDir, 'measurementAvail.csv']), Delimiter=',');
  
  systemTime_s = systemTime*1e-6;

  % Map the covariance to (15, 15, n)
  covarianceEstimate = MapCovariance(covarianceFlat, 15); 

  % Load mocap data
  mocapData = readtable(mocapDataFile);

  % Load flight data
  fd = readtable(quadDataFile);
  fdEuler_madgwick = [fd.euler_madgwick0, ...
                      fd.euler_madgwick1, ...
                      fd.euler_madgwick2]*180/pi;
  % Time sync mocap + ekf
  offset = fd.quadTime - fd.timeSinceBoot(1) - fd.mocapTime;
  offset = rmoutliers(offset);
  offset = mean(offset);

  minTime = systemTime(1);
  maxTime = systemTime(end);

  mocapTime = mocapData.time_us + offset;
  % Fix any non-unique points in the mocap times
  [mocapTime, iu] = unique(mocapTime);
  mocapData = mocapData(iu, :);
  mocapIdx = find(mocapTime >= minTime & mocapTime <= maxTime);
  mocapData = mocapData(mocapIdx, :);
  mocapTime = mocapTime(mocapIdx);


  % Interpolate mocap_pos to align it with estimates
  mocapX = interp1(mocapTime, mocapData.x, systemTime);
  mocapY = interp1(mocapTime, mocapData.y, systemTime);
  mocapZ = interp1(mocapTime, mocapData.z, systemTime);
  mocapPos = [mocapX, mocapY, mocapZ];
  
  % Compute position errors, RMSE, NIS, etc.
  errorX = mocapX - stateEstimate(:, 1);
  errorY = mocapY - stateEstimate(:, 2);
  errorZ = mocapZ - stateEstimate(:, 3);
  posErrors = [errorX, errorY, errorZ];

  positionSigma = zeros(length(systemTime), 3);
  for i = 1:3
      positionSigma(:, i) = sqrt(reshape(covarianceEstimate(i, i, :), [length(systemTime), 1]));
  end
  
  percentBounded = sum(posErrors < positionSigma)/length(systemTime)
  
  rmseX = rms(errorX, "omitnan")
  rmseY = rms(errorY, "omitnan")
  rmseZ = rms(errorZ, "omitnan")

  % Plot options
  fdColor    = 'b';
  mocapColor = 'r';
  colors     = ["#32a852"
                "#3262a8"
                "#a83248"
                "#a88132"];
  lw = 1;

  % Plot pos, vel, att estimates
  figure(Name="Position")
  ylabels = ["X [m]", "Y [m]", "Z [m]"];
  for i = 1:3
    ax(i) = subplot(3, 1, i);
    plot(systemTime_s, stateEstimate(:, i), LineStyle='-', Color=colors(1), DisplayName="Estimate");
    hold on
    plot(systemTime_s, mocapPos(:, i), LineStyle='--', Color=colors(2), DisplayName="MoCap");
    hold off
    ylabel(ylabels(i))
    grid on
  end
  sgtitle("Position Estimate")
  legend(ax(1))

  figure(Name="Velocity")
  ylabels = ["V_x [m]", "V_y [m]", "V_z [m]"];
  for i = 1:3
    ax(i) = subplot(3, 1, i);
    plot(systemTime_s, stateEstimate(:, i+3), LineStyle='-', Color=colors(1));
    ylabel(ylabels(i))
    grid on
  end
  sgtitle("Velocity Estimate")
  legend(ax(1))

  figure(Name="Attitude")
  ylabels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"];
  for i = 1:3
    s(i) = subplot(3, 1, i);
    plot(systemTime_s, fdEuler_madgwick(:, i), Color=colors(1), LineWidth=lw, ...
      DisplayName="Madgwick estimate");
    hold on
    plot(systemTime_s, stateEstimate(:, i+6)*180/pi, Color=colors(3), LineWidth=lw, ...
      DisplayName="EKF estimate");
    hold off
    ylabel(ylabels(i))
  end
  sgtitle("Attitude Estimate")
  legend();
  xlabel("time (s)");
  linkaxes(s, 'x')


  % Plot pos error with covariance bounds
  figure(Name="Position Error")
  ylabels = ["X [m]", "Y [m]", "Z [m]"];
  for i = 1:3
    ax(i) = subplot(3, 1, i);
    plot(systemTime_s, posErrors(:, i), LineStyle='-', Color=colors(1), DisplayName="Estimate");
    hold on
    plot(systemTime_s, positionSigma(:, i), LineStyle='--', Color=colors(3), DisplayName="1 \sigma")
    plot(systemTime_s, -positionSigma(:, i), LineStyle='--', Color=colors(3), DisplayName="1 \sigma")
    hold off
    ylabel(ylabels(i))
    grid on
  end
  sgtitle("Position Estimate Error")
  

  % Plot biases (with covariance bounds?)
  figure(Name="Accelerometer Biases")
  ylabels = ["b_{a1} [m/s^2]", "b_{a2} [m/s^2]", "b_{a3} [m/s^2]"];
  for i = 1:3
    ax(i) = subplot(3, 1, i);
    plot(systemTime_s, stateEstimate(:, i+9), LineStyle='-', Color=colors(1), DisplayName="Estimate");
    ylabel(ylabels(i))
    grid on
  end
  sgtitle("Accelerometer Biases")

  figure(Name="Gyro Biases")
  ylabels = ["b_{g1} [m/s^2]", "b_{g2} [m/s^2]", "b_{g3} [m/s^2]"];
  for i = 1:3
    ax(i) = subplot(3, 1, i);
    plot(systemTime_s, stateEstimate(:, i+12), LineStyle='-', Color=colors(1), DisplayName="Estimate");
    ylabel(ylabels(i))
    grid on
  end
  sgtitle("Gyro Biases")

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
