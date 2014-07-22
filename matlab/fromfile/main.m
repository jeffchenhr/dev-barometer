function [ dout, D, packetstarts ] = main( filename )
% on chip selections:
% assume FS_SEL=3 and AFS_SEL=3:
% 2048 counts/g
% 16.4 counts/(deg/s)
gyro_fac = 2000 / 2^15;
mag_fac  = 1;
acc_fac  = 16 / 2^15;

% would you like some graphical output (slow)?
withgraphics = 0;

addpath('../graphics');
addpath('../geom');


if (~isa(filename, 'char'))
    error('Please supply a valid filename.');
end

D = [];
fid = fopen(filename);
while (~feof(fid))
    D = [D; fread(fid, 8192, '*uint8')];
end
fclose(fid);
N = numel(D)

packet_size = 60; % +6 for vicon
pos = 1;
packetstarts = [0 0];
lastlength = 0;
lastpos = 0;
while (pos + packet_size < N)
    while ((pos + packet_size < N) && ((D(pos) ~= 32) || (D(pos+1) ~= 32) || (D(pos+2) ~= 32) || (D(pos+3) ~= 32)))
        pos = pos + 1;
    end
    
    length = pos - lastpos;
    if ((lastlength ~= packet_size) && (length + lastlength == packet_size))
        % last and current add up to perfect packet
        lastpos = pos;
        lastlength = packet_size;
        packetstarts(end) = pos;
    else
        if ((length ~= packet_size) && (packetstarts(end) - packetstarts(end-1) ~= packet_size))
            % combine bad packets into one large bad
            lastpos = pos;
            lastlength = pos - packetstarts(end-1);
            packetstarts(end) = pos;
            
        else
            lastpos = pos;
            lastlength = length;
            packetstarts = [packetstarts pos];
        end
    end
    
    pos = pos + 1;
end

% it is normal that the first and last packet are bad.
% if you have many bad packets in the middle your data connection was bad
% or something else is/was wrong...
badpackets = find(diff(packetstarts) ~= packet_size)
goodpackets = packetstarts(diff(packetstarts) == packet_size);

if (withgraphics)
    ViewSize = 3;
    ViewHeight = 3;

    vp.c1l = 0.05;
    vp.c1w = 0.45;
    vp.c2l = 0.55;
    vp.c2w = 0.4;

    figure;
    subplot('Position', [vp.c1l 0.55 vp.c1w 0.4]);
    hqs = paintQuadshot([0 0 0], eye(3));
    view(3); grid on;
    %xlabel('North (X)'); ylabel('East (Y)'); zlabel('Down (Z)');
    %set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
    xlabel('East (X)'); ylabel('North (Y)'); zlabel('Up (Z)');
    set(gca, 'XLim', [-ViewSize ViewSize], 'YLim', [-ViewSize ViewSize], 'ZLim', [-ViewHeight ViewHeight]); 
    hqstitle = title('uninit');

    subplot('Position', [vp.c1l 0.05 vp.c1w 0.4]);
    hqs_goal = paintQuadshot([0 0 0], eye(3));
    view(3); grid on;
    %xlabel('X'); ylabel('Y'); zlabel('Z');
    %xlabel('North (X)'); ylabel('East (Y)'); zlabel('Down (Z)');
    xlabel('East (X)'); ylabel('North (Y)'); zlabel('Up (Z)');
    %set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
    set(gca, 'XLim', [-ViewSize ViewSize], 'YLim', [-ViewSize ViewSize], 'ZLim', [-ViewHeight ViewHeight]); 
    hqs_goal_title = title('uninit');

    lim_gyro = 2^12 * gyro_fac;
    lim_mag  = 2^10 * mag_fac;
    lim_acc  = 2^12 * acc_fac;
    lim_actuator = 2000;

    subplot('Position', [vp.c2l 0.05 vp.c2w 0.15]);
    hgyro = bar([-1 0 1], [0 0 0]);
    hgyro_title = title('GYRO [-32768..32767]');
    set(gca, 'YLim', [-lim_gyro lim_gyro]);

    subplot('Position', [vp.c2l 0.3 vp.c2w 0.15]);
    hacc = bar([-1 0 1], [0 0 0]);
    hacc_title = title('ACC');
    set(gca, 'YLim', [-lim_acc lim_acc]);

    subplot('Position', [vp.c2l 0.55 vp.c2w 0.15]);
    hmag = bar([-1 0 1], [0 0 0]);
    hmag_title = title('MAG [-32768..32767]');
    set(gca, 'YLim', [-lim_mag lim_mag]);

    subplot('Position', [vp.c2l 0.8 vp.c2w 0.15]);
    hactuator = bar(0:5, [0 0 0 0 0 0]);
    hactuator_title = title('ACTUATORS');
    set(gca, 'YLim', [0 lim_actuator]);
    set(gca, 'XTick', 0:5, 'XTickLabel', { 'A1 (LT)', 'A2 (RT)', 'B1 (LB)', 'B2 (RB)', 'A_LEFT', 'A_RIGHT' });
end

lasttick = 0;
N = numel(goodpackets);
dout.tick = zeros(N,1);

dout.mag  = zeros(N,3);
dout.gyro = zeros(N,3);
dout.acc  = zeros(N,3);

dout.actuators = zeros(N,6);
dout.body_quat = zeros(N,4);
dout.goal_quat = zeros(N,4);
dout.body_pos  = zeros(N,3);

dout.status    = zeros(N,1);
dout.motorson  = zeros(N,1);


for i = 1:N
    pos = goodpackets(i);
    packet = D(pos+4:pos+packet_size-1);
    data = zeros(28,1);
    for j = 1:28
        data(j) = 256 * int32(packet(2*j)) + int32(packet(2*j-1));
        if ((j ~= 1) && (j ~= 28) && (data(j) > 32767))
            data(j) = data(j) - 65536;
        end
    end
    
    tick      = data(1);
    mag       = data(2:4) * mag_fac;
    gyro      = data(5:7) * gyro_fac;
    acc       = data(8:10) * acc_fac;
    actuators = data(11:16);
    qis       = data(17:20) / 32768;
    qgoal     = data(21:24) / 32768;
    pos       = data(25:27) / 1000;
    status    = data(28); % contains info about radio connection, imu and motor initialization, plus debug vars

    
    rstatus = status & 3;
    aligner = bitand(bitshift(status, -2), 3);
    ahrs    = bitand(bitshift(status, -4), 1);
    arming  = bitand(bitshift(status, -5), 7);
    motors  = bitand(bitshift(status, -8), 1);
    wrl     = bitand(bitshift(status, -9), 127);    

    
    dout.tick(i)   = tick;
    dout.mag(i,:)  = mag;
    dout.gyro(i,:) = gyro;
    dout.acc(i,:)  = acc;

    dout.actuators(i,:) = actuators;
    dout.body_quat(i,:) = qis;
    dout.goal_quat(i,:) = qgoal;
    dout.body_pos(i,:)  = pos;

    dout.status(i)    = status;    
    dout.motorson(i)  = motors;
    
    if (withgraphics)
        paintQuadshot([0 0 0], RMAT_OF_QUAT(qis), hqs);
        paintQuadshot([0 0 0], RMAT_OF_QUAT(qgoal), hqs_goal);
        set(hqstitle, 'String', sprintf('T=%6.3f', tick/1000));
        set(hqs_goal_title, 'String', sprintf('R%d A%d/%d M%d/%d W%3d', rstatus, aligner, ahrs, arming, motors, wrl));

        set(hmag, 'YData', mag);
        set(hgyro, 'YData', gyro);
        set(hacc, 'YData', acc);
        set(hactuator, 'YData', actuators);

        set(hmag_title, 'String', sprintf('MAG [uT]: x=%6.3f y=%6.3f z=%6.3f', mag(1), mag(2), actuators(3)));
        set(hgyro_title, 'String', sprintf('GYRO [deg/s]: x=%6.3f y=%6.3f z=%6.3f', gyro(1), gyro(2), gyro(3)));
        set(hacc_title, 'String', sprintf('ACC [g]: x=%6.3f y=%6.3f z=%6.3f', acc(1), acc(2), acc(3)));
        set(hactuator_title, 'String', sprintf('ACTUATORS [%4d %4d %4d %4d %4d %4d]', actuators(1), actuators(2), actuators(3), actuators(4), actuators(5), actuators(6)));

        drawnow;
    end
    
    % check if our ticks are 10ms apart, if not we missed a packet
    % (except for first of course)
    % NOTE: ticks overflow at 2^16
    if (tick ~= mod(lasttick + 10, 65536))
        fprintf('missed some packets: %5d --> %5d\n', lasttick, tick);
    end
    lasttick = tick;
    
end


end

function rm = RMAT_OF_QUAT(q)    

v2qi2_m1 = 2*q(1)^2 - 1;
v2qiqx   = 2*q(1)*q(2);
v2qiqy   = 2*q(1)*q(3);
v2qiqz   = 2*q(1)*q(4);

v2qxqy   = 2*q(2)*q(3);
v2qxqz   = 2*q(2)*q(4);
v2qyqz   = 2*q(3)*q(4);

rm = [2*q(2)^2+v2qi2_m1  v2qxqy-v2qiqz  v2qxqz+v2qiqy; ...
      v2qxqy+v2qiqz  2*q(3)^2+v2qi2_m1  v2qyqz-v2qiqx; ...
      v2qxqz-v2qiqy  v2qyqz+v2qiqx  2*q(4)^2+v2qi2_m1];
end

