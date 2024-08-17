clear; close all;
%============
% User Inputs
%============

% Filenames that contain flight data and motion capture system logs
quadfile  = "../flightdata/FlightData_8_3_24/flight_data13.csv";
mocapfile = "";

% You can use this to select a subset of the flight time
minTime = 0;  % [s]
maxTime = 10000;  % [s]

% Plot options
fdColor    = 'b';
mocapColor = 'r';
colors     = ["#32a852"
							"#3262a8"
							"#a83248"
							"#a88132"];
lw = 1;

plot_position      = true;
plot_attitude      = true;
plot_velocity      = true;
plot_biases        = true;
plot_posError      = false;
plot_imu           = true;
plot_motors        = true;
plot_controlInputs = true;

%===================
% Import flight data
%===================
fd = readtable(quadfile, 'NumHeaderLines', 0);
fd.time_us = fd.timeSinceBoot - fd.timeSinceBoot(1);
fd.time_s = fd.time_us*1e-06;

% Truncate the set of flight data to the desired time window
fd_idx           = find(fd.time_s >= minTime & fd.time_s <= maxTime);
time_trunc       = fd.time_s(fd_idx);
fdPos_trunc      = [fd.positionNED_0(fd_idx), ...
	                  fd.positionNED_1(fd_idx), ...
	                  fd.positionNED_2(fd_idx)];
fdPos_setpoint   = [fd.positionSetpointNED_0(fd_idx), ...
	                  fd.positionSetpointNED_1(fd_idx), ...
	                  fd.positionSetpointNED_2(fd_idx)];
fdVel_trunc      = [fd.velocityNED_0(fd_idx), ...
	                  fd.velocityNED_1(fd_idx), ...
	                  fd.velocityNED_2(fd_idx)];
fdVel_setpoint   = [fd.velocitySetpointNED_0(fd_idx), ...
	                  fd.velocitySetpointNED_1(fd_idx), ...
	                  fd.velocitySetpointNED_2(fd_idx)];
fdEuler_madgwick = [fd.euler_madgwick0(fd_idx), ...
	                  fd.euler_madgwick1(fd_idx), ...
	                  fd.euler_madgwick2(fd_idx)]*180/pi;
fdEuler_ekf      = [fd.euler_ekf0(fd_idx), ...
	                  fd.euler_ekf1(fd_idx), ...
	                  fd.euler_ekf2(fd_idx)]*180/pi;
fdEuler_setpoint = [fd.euler_setpoint0(fd_idx), ...
	                  fd.euler_setpoint1(fd_idx), ...
	                  fd.euler_setpoint2(fd_idx)]*180/pi;

% If mocapTime remains zero for the whole flight, then there were no position updates
posUpdatesAvailable = any(fd.mocapTime > 0);

%===========================
% Import motion capture data
%===========================
% If a motion capture file is specified, it will be time-shifted to align with
% the flight data
if ~strcmp(mocapfile, '')
	offset = fd.quadTime - fd.timeSinceBoot(1) - fd.mocapTime;
	offset = rmoutliers(offset);
	offset = mean(offset);
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
	mocapPos_trunc_xi = interp1(mocapTime_trunc, mocapPos_trunc(:,1), time_trunc);
	mocapPos_trunc_yi = interp1(mocapTime_trunc, mocapPos_trunc(:,2), time_trunc);
	mocapPos_trunc_zi = interp1(mocapTime_trunc, mocapPos_trunc(:,3), time_trunc);
	
	errorX = mocapPos_trunc_xi - fdPos_trunc(:, 1);
	errorY = mocapPos_trunc_yi - fdPos_trunc(:, 2);
	errorZ = mocapPos_trunc_zi - fdPos_trunc(:, 3);
	
	rmseX = rms(errorX, "omitnan");
	rmseY = rms(errorY, "omitnan");
	rmseZ = rms(errorZ, "omitnan");
	
	% Euler angles [roll, pitch, yaw]
	mocapRPY = quat2eul(mocapQ);
	mocapRPY = flip(mocapRPY, 2)*180/pi;
else
	mocapTime = [];
	mocapPos = [];
	mocapRPY = [];
	mocapTime_trunc = [];
end

%=========
% Plotting
%=========

if plot_position == true
	figure(Name="Position")
	labels = ["X (North) (m)", "Y (East) (m)", "Z (Down) (m)"];
	for i = 1:3
		s(i) = subplot(3, 1, i);
		if posUpdatesAvailable
			plot(time_trunc, fdPos_trunc(:, i), Color=colors(1), LineWidth=lw, ...
				DisplayName="EKF estimate");
		end
		hold on
		plot(time_trunc, fdPos_setpoint(:, i), Color=colors(4), LineWidth=lw, ...
			DisplayName="Setpoint", LineStyle="--");
		hold off
		ylabel(labels(i))
	end
	
	if ~isempty(mocapTime_trunc)
		for i = 1:3
			hold(s(i), 'on')
			plot(s(i), mocapTime_trunc, mocapPos_trunc(:, i), ...
				Color=colors(2), LineWidth=lw, DisplayName="Mocap measurement");
			hold(s(i), 'off');
		end
	end
	legend(Location="southeast")
	xlabel(s(3), "time (s)");
	linkaxes(s, 'x')
	
end

if plot_attitude == true
	figure(Name="Attitude")
	labels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"];
	for i = 1:3
		s(i) = subplot(3, 1, i);
		plot(time_trunc, fdEuler_madgwick(:, i), Color=colors(1), LineWidth=lw, ...
			DisplayName="Madgwick estimate");
		hold on
		if posUpdatesAvailable
			plot(time_trunc, fdEuler_ekf(:, i), Color=colors(3), LineWidth=lw, ...
				DisplayName="EKF estimate");
		end
		plot(time_trunc, fdEuler_setpoint(:, i), Color=colors(4), LineWidth=lw, ...
			DisplayName="Setpoint", LineStyle="--");
		hold off
		ylabel(labels(i))
	end
	
	if ~isempty(mocapTime_trunc)
		for i = 1:3
			hold(s(i), 'on')
			plot(s(i), mocapTime_trunc, mocapRPY(mocap_idx, i), ...
				Color=colors(2), LineWidth=lw, DisplayName="Mocap measurement");
			hold(s(i), 'off');
		end
	end
	legend();
	xlabel("time (s)");
	linkaxes(s, 'x')
	
end


if plot_velocity == true
	figure(Name="Velocity")
	labels = ["V_x (North) (m/s)", "V_y (East) (m/s)", "V_z (Down) (m/s)"];
	for i = 1:3
		s(i) = subplot(3, 1, i);
		if posUpdatesAvailable
			plot(time_trunc, fdVel_trunc(:, i), Color=colors(1), LineWidth=lw, ...
				DisplayName="EKF estimate");
		end
		hold on
		plot(time_trunc, fdVel_setpoint(:, i), Color=colors(4), LineWidth=lw, ...
			DisplayName="Setpoint", LineStyle="--");
		hold off
		ylabel(labels(i))
	end
	legend(Location="southeast")
	xlabel(s(3), "time (s)");
	linkaxes(s, 'x')
end

if plot_posError == true && ~isempty(mocapPos_trunc)
	figure()
	s1 = subplot(311);
	plot(time_trunc, errorX, Color=colors(1), LineWidth=lw);
	ylabel("X Error (m)")
	grid on;
	
	s2 = subplot(312);
	plot(time_trunc, errorY, Color=colors(1), LineWidth=lw);
	ylabel("Y Error (m)")
	grid on;
	
	s3 = subplot(313);
	plot(time_trunc, errorZ, Color=colors(1), LineWidth=lw);
	ylabel("Z Error (m)")
	grid on;
	
	xlabel("time (s)");
	linkaxes([s1, s2, s3], 'x')
end

if plot_imu == true
	figure(Name="Accelerometer")
	ax(1) = subplot(311);
	scatter(time_trunc, fd.Acc1Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.AccBMI1Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Acc1(fd_idx), Color=colors(1))
	plot(time_trunc, fd.AccBMI1(fd_idx), Color=colors(2))
	hold off
	ylabel("fx (m/s^2)")
	grid on;
	
	ax(1) = subplot(312);
	scatter(time_trunc, fd.Acc2Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.AccBMI2Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Acc2(fd_idx), Color=colors(1))
	plot(time_trunc, fd.AccBMI2(fd_idx), Color=colors(2))
	hold off
	ylabel("fy (m/s^2)")
	grid on;
	
	
	ax(1) = subplot(313);
	scatter(time_trunc, fd.Acc3Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.AccBMI3Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Acc3(fd_idx), Color=colors(1))
	plot(time_trunc, fd.AccBMI3(fd_idx), Color=colors(2))
	hold off
	ylabel("fz (m/s^2)")
	grid on;
	
	xlabel("time (s)");
	linkaxes(ax, 'x');
	
	figure(Name="Gyroscope")
	ax(1) = subplot(311);
	scatter(time_trunc, fd.Gyro1Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.GyroBMI1Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Gyro1(fd_idx), Color=colors(1))
	plot(time_trunc, fd.GyroBMI1(fd_idx), Color=colors(2))
	hold off
	ylabel("gx (rad/s)")
	grid on;
	
	ax(1) = subplot(312);
	scatter(time_trunc, fd.Gyro2Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.GyroBMI2Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Gyro2(fd_idx), Color=colors(1))
	plot(time_trunc, fd.GyroBMI2(fd_idx), Color=colors(2))
	hold off
	ylabel("gy (rad/s)")
	grid on;
	
	
	ax(1) = subplot(313);
	scatter(time_trunc, fd.Gyro3Raw(fd_idx), Color=colors(1), marker='.')
	hold on
	scatter(time_trunc, fd.GyroBMI3Raw(fd_idx), Color=colors(2), marker='.')
	plot(time_trunc, fd.Gyro3(fd_idx), Color=colors(1))
	plot(time_trunc, fd.GyroBMI3(fd_idx), Color=colors(2))
	hold off
	ylabel("gz (rad/s)")
	grid on;
	
	xlabel("time (s)");
	linkaxes(ax, 'x');
end

if plot_motors == true
	figure()
	s1 = subplot(221);
	plot(time_trunc, fd.w0(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("Angular Rate (rad/s)")
	xlabel("time (s)");
	grid on;
	
	s2 = subplot(222);
	plot(time_trunc, fd.w1(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("Angular Rate (rad/s)")
	xlabel("time (s)");
	grid on;
	
	s3 = subplot(224);
	plot(time_trunc, fd.w2(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("Angular Rate (rad/s)")
	xlabel("time (s)");
	grid on;
	
	s4 = subplot(223);
	plot(time_trunc, fd.w3(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("Angular Rate (rad/s)")
	xlabel("time (s)");
	grid on;
	
	linkaxes([s1, s2, s3, s4], 'x')
	set([s1, s2, s3, s4], 'ylim', [0, 1500])
end

if plot_controlInputs == true
	figure()
	s1 = subplot(411);
	plot(time_trunc, fd.u0(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("Thrust (N)")
	grid on;
	
	s2 = subplot(412);
	plot(time_trunc, fd.u1(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("\tau_x (Nm)")
	grid on;
	
	s3 = subplot(413);
	plot(time_trunc, fd.u2(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("\tau_y (Nm)")
	grid on;
	
	s4 = subplot(414);
	plot(time_trunc, fd.u3(fd_idx), Color=colors(1), LineWidth=lw);
	ylabel("\tau_z (Nm)")
	xlabel("time (s)");
	grid on;
	
	linkaxes([s1, s2, s3, s4], 'x')
end
