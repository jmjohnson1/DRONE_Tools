function [t, x, y, z, q] = processMocapRigid(filepath, scaling, timeOffset, lowpassFreq)
    if nargin < 3
        timeOffset = 0;
    end
    if nargin < 4
        lowpassFreq = 0;
        doLowpass = false;
    else
        doLowpass = true;
    end
    table = readtable(filepath, 'NumHeaderLines', 0);

    % t = (table.time - table.time(1) + timeOffset*1e-3)*1e-03*scaling ; % seconds
    %t = (table.time + timeOffset*1e-3)*1e-03*scaling ; % seconds
    t = (table.time_us + timeOffset)*1e-6;

    x = table.x;
    y = table.y;
    z = table.z;
    qw = table.qw;
    q1 = table.qx;
    q2 = table.qy;
    q3 = table.qz;
    

    % get rid of zeros
    % zerosIndex = find(x==0 & y==0 & z==0);
    % t(zerosIndex) = [];
    % x(zerosIndex) = [];
    % y(zerosIndex) = [];
    % z(zerosIndex) = [];
    % qw(zerosIndex) = [];
    % q1(zerosIndex) = [];
    % q2(zerosIndex) = [];
    % q3(zerosIndex) = [];

    if doLowpass
        % filter the data
        x = lowpass(x, lowpassFreq, dataRate);
        y = lowpass(y, lowpassFreq, dataRate);
        z = lowpass(z, lowpassFreq, dataRate);
        qw = lowpass(qw, lowpassFreq, dataRate);
        q1 = lowpass(q1, lowpassFreq, dataRate);
        q2 = lowpass(q2, lowpassFreq, dataRate);
        q3 = lowpass(q3, lowpassFreq, dataRate);
    end

    q = quaternion(qw, q1, q2, q3);

    % % Filter out bad times
    % res = sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2);
    % idx = find(res > 1000);
    % oldLength = length(idx);
    % while length(idx)>2% FIXME
    %     for i = (idx' + 1)
    %         if x(i) == 0
    %             x(i) = x(i - 1);
    %             y(i) = y(i - 1);
    %             z(i) = z(i - 1);
    %             q(i) = q(i - 1);
    %         end
    %     end
    %     res = sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2);
    %     oldLength = length(idx);
    %     idx = find(res > 1000);
    % 
    % end
    % 

    % % prepare coordinate transformation
    % pos = [x, y, z];
    % Cna = angle2dcm(0, pi, pi/2);
    % % Cna = angle2dcm(0, 0, pi);
    % Cba = quat2dcm(q);
    % Cnb = pagemtimes(Cna,pagetranspose(Cba));
    % Cbn = pagetranspose(Cnb);
    % q = dcm2quat(Cbn);
    % q = quaternion(q(:,1), q(:,2), q(:,3), q(:,4));
    % 
    % % Transform to NED
    % for i = 1:length(x) 
    %     pos(i, :) = Cna*pos(i,:)';
    % end
    % x = pos(:,1);
    % y = pos(:,2);
    % z = pos(:,3);
end