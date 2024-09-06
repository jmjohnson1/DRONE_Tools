function WriteNavData(inputPath, outputDir, imuType, filtering)
	arguments
		inputPath char
		outputDir char
		imuType char = 'mpu6050';
    filtering char = 'original';
  end

  outputDir = char(outputDir);
  imuType = char(imuType);
  if ~strcmp(outputDir(end), '/')
    outputDir = [outputDir, '/'];
  end

	dataTable = readtable(inputPath);	
	systemTime_us = dataTable.timeSinceBoot - dataTable.timeSinceBoot(1);
	mocapSequenceNumber = dataTable.numMocapUpdates;
  if strcmp(filtering, 'original')
	  if strcmp(imuType, 'mpu6050') 
		  accel = [dataTable.Acc1, dataTable.Acc2, dataTable.Acc3];
		  gyro = [dataTable.Gyro1, dataTable.Gyro2, dataTable.Gyro3];
	  elseif strcmp(imuType, 'bmi088')
		  accel = [dataTable.AccBMI1, dataTable.AccBMI2, dataTable.AccBMI3];
		  gyro = [dataTable.GyroBMI1, dataTable.GyroBMI2, dataTable.GyroBMI3];
    elseif strcmp(imuType, 'combined')
		  accel = ([dataTable.AccBMI1, dataTable.AccBMI2, dataTable.AccBMI3] ...
                + [dataTable.Acc1, dataTable.Acc2, dataTable.Acc3])/2;
		  gyro = ([dataTable.GyroBMI1, dataTable.GyroBMI2, dataTable.GyroBMI3] ...
                + [dataTable.Gyro1, dataTable.Gyro2, dataTable.Gyro3])/2;
	  else
		  error("Invalid imuType")
    end
  else
    if strcmp(imuType, 'mpu6050') 
		  accel = [dataTable.Acc1Raw, dataTable.Acc2Raw, dataTable.Acc3Raw];
		  gyro = [dataTable.Gyro1Raw, dataTable.Gyro2Raw, dataTable.Gyro3Raw];
	  elseif strcmp(imuType, 'bmi088')
		  accel = [dataTable.AccBMI1Raw, dataTable.AccBMI2Raw, dataTable.AccBMI3Raw];
		  gyro = [dataTable.GyroBMI1Raw, dataTable.GyroBMI2Raw, dataTable.GyroBMI3Raw];
    elseif strcmp(imuType, 'combined')
		  accel = ([dataTable.AccBMI1Raw, dataTable.AccBMI2Raw, dataTable.AccBMI3Raw] ...
                + [dataTable.Acc1Raw, dataTable.Acc2Raw, dataTable.Acc3Raw])/2;
		  gyro = ([dataTable.GyroBMI1Raw, dataTable.GyroBMI2Raw, dataTable.GyroBMI3Raw] ...
                + [dataTable.Gyro1Raw, dataTable.Gyro2Raw, dataTable.Gyro3Raw])/2;
	  else
		  error("Invalid imuType")
    end
  end
	mocapPositions = [dataTable.mocapPositionNED_0, dataTable.mocapPositionNED_1, dataTable.mocapPositionNED_2];

  tmp = split(inputPath, '/');
  inFileName = char(tmp(end));
  outfile = [outputDir, inFileName(1:end-4), '_processed_', imuType, '.csv'];
	% Write to CSV file
  mocapPositions = circshift(mocapPositions, -2);
  mocapSequenceNumber = circshift(mocapSequenceNumber, -2);
	writematrix([accel, gyro, mocapPositions, systemTime_us, mocapSequenceNumber], outfile)

	% Plot of each set of values
	figure(Name="IMU")
	ax(1) = subplot(2, 1, 1);
		plot(systemTime_us, accel(:, 1), Marker='.', DisplayName="X")
		hold on
		% plot(systemTime_us, accel(:, 2), Marker='.', DisplayName="Y")
		plot(systemTime_us, accel(:, 3), Marker='.', DisplayName="Z")
		hold off
	ax(2) = subplot(2, 1, 2);
		plot(systemTime_us, gyro(:, 1), Marker='.', DisplayName="X")
		hold on
		plot(systemTime_us, gyro(:, 2), Marker='.', DisplayName="Y")
		plot(systemTime_us, gyro(:, 3), Marker='.', DisplayName="Z")
		hold off
	grid(ax, 'on')
	linkaxes(ax, 'x')
	xlabel(ax(2), "System time [us]")
	ylabel(ax(1), "Accel [m/s^2]")
	ylabel(ax(2), "Gyro [rad/s]")
	legend(ax(1))

	figure(Name="Position")
	updateIdx = find(diff(mocapSequenceNumber) > 0) + 1;
	ax(1) = subplot(3, 1, 1);
		plot(ax(1), systemTime_us, dataTable.positionNED_0, Marker='.')
		hold on
		plot(ax(1), systemTime_us(updateIdx), mocapPositions(updateIdx, 1), Marker='o')
		hold off
	ax(2) = subplot(3, 1, 2);
		plot(ax(2), systemTime_us, dataTable.positionNED_1, Marker='.')
		hold on
		plot(ax(2), systemTime_us(updateIdx), mocapPositions(updateIdx, 2), Marker='o')
		hold off
	ax(3) = subplot(3, 1, 3);
		plot(ax(3), systemTime_us, dataTable.positionNED_2, Marker='.')
		hold on
		plot(ax(3), systemTime_us(updateIdx), mocapPositions(updateIdx, 3), Marker='o')
		hold off

end
