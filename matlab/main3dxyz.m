  function main3dxyz
% assume FS_SEL=3 and AFS_SEL=3:
% 2048 counts/g
% 16.4 counts/(deg/s)

hconfig_window = create_config_window();
%return; 

gyro_fac = 2000 / 2^15;
mag_fac  = 1;
acc_fac  = 16 / 2^15;

global data;
data.kill = 0;

addpath('graphics');
addpath('geom');

ViewSize = 2;
ViewHeight = 2;

vp.c1l = 0.05;
vp.c1w = 0.35;
vp.c2l = 0.5;
vp.c2w = 0.2;
vp.c3l = 0.75;
vp.c3w = 0.2;

figure; set(gcf,'CloseRequestFcn', @my_closereq);

use_ned = 0;

for icol = 1:2
    for irow = 1:3
        subplot('Position', [0.025+(icol-1)*0.22 0.03+(3-irow)*0.32 .2 .27]);
        %hquat{2*(irow-1)+icol} = paintQuadshot([0 0 0], eye(3));
        hquat{2*(irow-1)+icol} = paintXYZ([0 0 0], eye(3));
        view(3); grid on;
        if (use_ned)
            xlabel('North (X)'); ylabel('East (Y)'); zlabel('Down (Z)');
            set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
        else
            xlabel('East (X)'); ylabel('North (Y)'); zlabel('Up (Z)');
        end
        set(gca, 'XLim', [-ViewSize ViewSize], 'YLim', [-ViewSize ViewSize], 'ZLim', [-ViewHeight ViewHeight]); 
        hquat_title{2*(irow-1)+icol} = title('uninit');
    end
end

lim_gyro = 2^12 * gyro_fac;
lim_mag  = 2^10 * mag_fac;
lim_acc  = 2^12 * acc_fac;
lim_rates = 2*pi;

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
hrates = bar([-1 0 1], [0 0 0]);
hrates_title = title('RATES');
set(gca, 'YLim', [-lim_rates lim_rates]);

lim_radio = 10000;
lim_cmd = 0.25*9600;
lim_actuator = 2000;
% i2c scaling for ailerons:
lim_actuator = 200;

subplot('Position', [vp.c3l 0.05 vp.c3w 0.15]);
hradio = bar(0:5, [0 0 0 0 0 0]);
hradio_title = title('RADIO');
set(gca, 'YLim', [-lim_radio lim_radio]);
set(gca, 'XTick', 0:5, 'XTickLabel', { 'THROTTLE', 'ROLL', 'PITCH', 'YAW', 'GEAR', 'FLAP' });

subplot('Position', [vp.c3l 0.3 vp.c3w 0.15]);
hcmd = bar(0:3, [0 0 0 0]); % roll pitch yaw thrust
hcmd_title = title('CMD');
set(gca, 'YLim', [-lim_cmd lim_cmd]);
set(gca, 'XTick', 0:3, 'XTickLabel', { 'ROLL', 'PITCH', 'YAW', 'THRUST' });

subplot('Position', [vp.c3l 0.55 vp.c3w 0.15]);
hactuator = bar(0:5, [0 0 0 0 0 0]);
hactuator_title = title('ACTUATORS');
set(gca, 'YLim', [0 lim_actuator]);
set(gca, 'XTick', 0:5, 'XTickLabel', { 'A1 (LT)', 'A2 (RT)', 'B1 (LB)', 'B2 (RB)', 'A_LEFT', 'A_RIGHT' });

subplot('Position', [vp.c3l 0.8 vp.c3w 0.15]);
%hrates = bar([-1 0 1], [0 0 0]);
hstatus_title = title('Status');
hgear = rectangle('Position', [1,0,1,1], 'Curvature', [1,1], 'FaceColor', 'w');
text(1.5, 0.5, 'GEAR', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
helevdr = rectangle('Position', [2,0,1,1], 'Curvature', [1,1], 'FaceColor', 'w');
text(2.5, 0.5, 'ELEV D/R', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );

hflap = rectangle('Position', [3,0,1,1], 'Curvature', [1,1], 'FaceColor', 'w');
text(3.5, 0.5, 'FLAP', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
haildr = rectangle('Position', [4,0,1,1], 'Curvature', [1,1], 'FaceColor', 'w');
text(4.5, 0.5, 'AIL D/R', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );


hahrs_led = rectangle('Position', [1,1,1,1], 'Curvature', [1,1], 'FaceColor', 'r');
hahrstext = text(1.5, 1.5, 'AHRS', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
haligner_led = rectangle('Position', [2,1,1,1], 'Curvature', [1,1], 'FaceColor', 'r');
halignertext = text(2.5, 1.5, 'ALIGNER', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
hradio_led = rectangle('Position', [3,1,1,1], 'Curvature', [1,1], 'FaceColor', 'r');
hradiotext = text(3.5, 1.5, 'RADIO', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
hvicon_led = rectangle('Position', [4,1,1,1], 'Curvature', [1,1], 'FaceColor', 'r');
hvicontext = text(4.5, 1.5, 'VICON', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );
hbattery_led = rectangle('Position', [5,1,1,1], 'Curvature', [1,1], 'FaceColor', 'r');
hbatterytext = text(5.5, 1.5, 'BAT', 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Middle' );

set(gca, 'XLim', [0 6], 'YLim', [0 3]);
%set(gca, 'YLim', [-lim_rates lim_rates]);



if (0)
    subplot('Position', [0.05 0.05 0.4 0.15]);
    hquat = bar([-1.5 -0.5 0.5 1.5], [0 0 0 0]);
    hquat_title = title('Quaternion');
    %set(gca, 'YLim', [-1 1]);
    lim_cmd = 2^11;
    set(gca, 'YLim', [-lim_cmd lim_cmd]);
end

set(hquat_title{2}, 'String', 'GOAL QUAT'); %'ned\_to\_body\_orientation\_goal\_quat\_i');
set(hquat_title{3}, 'String', 'IMU QUAT'); %'ahrs\_impl.ltp\_to\_imu\_quat');
set(hquat_title{4}, 'String', 'BODY 2 IMU QUAT'); %'imu.body\_to\_imu\_quat');
set(hquat_title{5}, 'String', '???'); %'enu\_to\_body\_orientation\_vicon\_quat\_i');


% /dev/tty.usbserial-A9KRBL95
% /dev/tty.usbserial-FTEZOHBA
% /dev/tty.usbserial-FTEZKC1L
%serial_write_mac('open', '/dev/tty.usbserial-FTEZKC1L', 921600); %115200

% /dev/tty.usbserial-A9KRBL95
% /dev/tty.usbserial-FTEZOHBA
% /dev/tty.usbserial-FTEZKC1L
% /dev/tty.usbserial-A96PDJVB
% FTDPDIK4
% FTG7RQ0R
serial_lisam('open', '/dev/tty.usbserial-FTG7RQ0R', 921600); %115200

pause(0.1);
msg_tick = 0;
while (~data.kill)
    [lisam, N] = serial_lisam('read_new');
    if (N > 0)        
        status = serial_lisam('status');

        paintXYZ([0 0 0], RMAT_OF_QUAT(lisam.body_quat), hquat{1});
        paintXYZ([0 0 0], RMAT_OF_QUAT(lisam.goal_quat), hquat{2});
        set(hquat_title{2}, 'String', sprintf('q=[%7.4f %7.4f %7.4f %7.4f]\np=[%7.4f %7.4f %7.4f]', lisam.goal_quat(1), lisam.goal_quat(2), lisam.goal_quat(3), lisam.goal_quat(4), lisam.vicon_pos(1), lisam.vicon_pos(2), lisam.vicon_pos(3)));
        
        R = RMAT_OF_QUAT(lisam.body_quat);
        R(3,3)
        
        status = serial_write_mac('status');
        set(hquat_title{1}, 'String', sprintf('%6.3f (%d bytes, %d pkt/update)\nq=[%7.4f %7.4f %7.4f %7.4f]', lisam.tick, status(6), N,   lisam.body_quat(1), lisam.body_quat(2), lisam.body_quat(3), lisam.body_quat(4)));
        
        %paintQuadshot([0 0 0], RMAT_OF_QUAT(D(38:41)), hquat{3});
        %paintQuadshot([0 0 0], RMAT_OF_QUAT(D(42:45)), hquat{4});
        paintXYZ([0 0 0], RMAT_OF_QUAT(lisam.imu_quat), hquat{3});
        paintXYZ([0 0 0], RMAT_OF_QUAT(lisam.body_to_imu_quat), hquat{4});
        paintXYZ([0 0 0], RMAT_OF_QUAT(lisam.vicon_quat), hquat{5});
        %set(hquat_title{5}, 'String', sprintf('VICON q=[%7.4f %7.4f %7.4f %7.4f] x=%7.3f y=%7.3f z=%7.3f', D(46), D(47), D(48), D(49), D(58), D(59), D(60)));
        %paintXYZ([0 0 0], RMAT_OF_QUAT(D(50:53)), hquat{6});
        %set(hquat_title{6}, 'String', sprintf('VICON qvic=[%7.4f %7.4f %7.4f %7.4f]  qacc=[%7.4f %7.4f %7.4f %7.4f] setpoint: x=%7.3f y=%7.3f z=%7.3f', D(50), D(51), D(52), D(53),  D(54), D(55), D(56), D(57),  D(61), D(62), D(63)));
        %[norm(D(46:49)) norm(D(46:49)./2)]
        
        lGyro = lisam.gyro * gyro_fac;
        lAcc  = lisam.acc  * acc_fac;
        lMag  = lisam.mag  * mag_fac;
        
        % i2c scaling for ailerons:
        lisam.actuators(5:6) = (lisam.actuators(5:6) - 1000) * 0.2;
        
        set(hgyro, 'YData', lGyro);
        set(hacc, 'YData', lAcc);
        set(hmag, 'YData', lMag);
        set(hgyro_title, 'String', sprintf('GYRO [deg/s]: x=%6.3f y=%6.3f z=%6.3f', lGyro(1), lGyro(2), lGyro(3)));
        set(hacc_title, 'String', sprintf('ACC [g]: x=%6.3f y=%6.3f z=%6.3f', lAcc(1), lAcc(2), lAcc(3)));
        set(hmag_title, 'String', sprintf('MAG [uT]: x=%6.3f y=%6.3f z=%6.3f', lMag(1), lMag(2), lMag(3)));
        set(hactuator, 'YData', lisam.actuators);

        set(hrates, 'YData', lisam.rates);
        set(hradio, 'YData', lisam.radio);
        set(hcmd, 'YData', lisam.commands);
        
        set(hrates_title, 'String', sprintf('RATES: p=%6.3f q=%6.2f r=%6.3f', lisam.rates(1), lisam.rates(2), lisam.rates(3)));
        set(hcmd_title, 'String', sprintf('CMD: [%5d %5d %5d %5d]', lisam.commands(1), lisam.commands(2), lisam.commands(3), lisam.commands(4)));
        set(hactuator_title, 'String', sprintf('ACTUATORS [%4d %4d %4d %4d %4d %4d]', lisam.actuators(1), lisam.actuators(2), lisam.actuators(3), lisam.actuators(4), lisam.actuators(5), lisam.actuators(6)));
        set(hradio_title, 'String', sprintf('RADIO [%6d %6d %6d %6d %6d %6d]', lisam.radio(1), lisam.radio(2), lisam.radio(3), lisam.radio(4), lisam.radio(5), lisam.radio(6)));

        set(hstatus_title, 'String', sprintf('AHRS: %d, ALIGNER: %d, ARMING: %d, RADIO: %d, VICON: %d', lisam.ahrs, lisam.aligner, lisam.arming, lisam.radio_on, lisam.vicon_on));
        
        display_config_values(hconfig_window, lisam.pid, lisam.pos_pid, lisam.motor_coef, lisam.pos_cmd_limits, lisam.ailevon_params);
                
        if (lisam.posctrl_on == 1)
            set(hgear, 'FaceColor', 'g');
        else
            set(hgear, 'FaceColor', 'w');            
        end
        if (lisam.sw_elevdr == 1)
            set(hflap, 'FaceColor', 'g');
        else
            set(hflap, 'FaceColor', 'w');            
        end
        if (lisam.sw_flap == 1)
            set(haildr, 'FaceColor', 'g');
        else
            set(haildr, 'FaceColor', 'w');            
        end
        if (abs(lisam.radio(6)) < 7500)
            set(helevdr, 'FaceColor', 'g');
        else
            set(helevdr, 'FaceColor', 'w');            
        end
        
        if (lisam.ahrs == 1)
            set(hahrs_led, 'FaceColor', 'g');
        else
            set(hahrs_led, 'FaceColor', 'r');            
        end
        if (lisam.aligner == 2)
            set(haligner_led, 'FaceColor', 'g');
        else
            set(haligner_led, 'FaceColor', 'r');            
        end
        if (lisam.radio_on == 0)
            set(hradio_led, 'FaceColor', 'g');
        else
            set(hradio_led, 'FaceColor', 'r');            
        end
        if (lisam.vicon_on < 15)
            set(hvicon_led, 'FaceColor', 'g');
        else
            set(hvicon_led, 'FaceColor', 'r');            
        end
        set(hbatterytext, 'String', sprintf('%4.2f V', lisam.battery_voltage));
        if (lisam.battery_voltage > 10.9)
            set(hbattery_led, 'FaceColor', 'g');
        else
            set(hbattery_led, 'FaceColor', 'r');            
        end

        drawnow; msg_tick = msg_tick + 1;
    end
    pause(0.01);
end
fprintf('EXITING\n');
pause(0.01);
serial_lisam('close');
delete(gcf);
if (ishandle(hconfig_window.fig))
    delete(hconfig_window.fig);
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


function rm = RMAT_OF_QUAT_(q)    
% implemented EXACTLY as in Paparazzi    
    v2qi2_m1	= 2*q(1)^2 - 1;
	rm(1)	   	= 2*q(2)^2;
	rm(5)	   	= 2*q(3)^2;
	rm(9)	   	= 2*q(4)^2;

    v2qiqx   = 2*q(1)*q(2);
    v2qiqy   = 2*q(1)*q(3);
    v2qiqz   = 2*q(1)*q(4);

    rm(2)    = 2*q(2)*q(3);
    rm(3)    = 2*q(2)*q(4);
    rm(6)    = 2*q(3)*q(4);

    rm(1) = rm(1) + v2qi2_m1;
    rm(4) = rm(2) - v2qiqz;
    rm(7) = rm(3) + v2qiqy;
    
    rm(8) = rm(6) - v2qiqx;
    rm(5) = rm(5) + v2qi2_m1;
    rm(2) = rm(2) + v2qiqz;
    
    rm(3) = rm(3) - v2qiqy;
    rm(6) = rm(6) + v2qiqx;
    rm(9) = rm(9) + v2qi2_m1;
end

function my_closereq(src,evnt)
global data;
data.kill = 1;
fprintf('Closing figure\n');
%serial_write_mac('close');
%delete(gcf);
end

function h = create_config_window()
h.fig = figure;

h.pid_labels = { 'p.x', 'i.x', 'd.x', 'dd.x', 'p.y', 'i.y', 'd.y', 'dd.y', 'p.z', 'i.z', 'd.z', 'dd.z' };
h.mcoef_labels = { 'pitch A1', 'roll A1', 'yaw A1', 'thrust A1', ...
    'pitch A2', 'roll A2', 'yaw A2', 'thrust A2', ...
    'pitch B1', 'roll B1', 'yaw B1', 'thrust B1', ...
    'pitch B2', 'roll B2', 'yaw B2', 'thrust B2' };

h.pos_pid_labels = { 'p.x', 'i.x', 'd.x', 'dd.x', 'p.y', 'i.y', 'd.y', 'dd.y', 'p.z', 'i.z', 'd.z', 'dd.z' };
h.pos_cmd_limits_labels = { 'roll_max', 'pitch_max', 'yaw_max', 'thrust_max' };
h.ailevon_params_labels = { 'Ld', 'Lu', 'Rd', 'Ru' };

for i = 0:11
    col = floor(i / 4);
    row = mod(i, 4);
    h.pid{i+1} = uicontrol('Style', 'edit', 'Position', [10+100*col 160 - 50*row 100 30], 'FontSize', 16, 'String', 0);
    h.pidis{i+1} = uicontrol('Style', 'text', 'String', h.pid_labels{i+1} , 'Position', [10+100*col 190 - 50*row 100 15], 'FontSize', 12);
end
h.set_pid = uicontrol('Style', 'pushbutton', 'Position', [10 210 300 30], 'String', 'SET PID', 'FontSize', 16, 'Callback', @(x,y)(config_set_pid(x,y,h)));

for i = 0:11
    col = floor(i / 4) + 3;
    row = mod(i, 4);
    h.pos_pid{i+1} = uicontrol('Style', 'edit', 'Position', [10+100*col 160 - 50*row 100 30], 'FontSize', 16, 'String', 0);
    h.pos_pidis{i+1} = uicontrol('Style', 'text', 'String', h.pos_pid_labels{i+1} , 'Position', [10+100*col 190 - 50*row 100 15], 'FontSize', 12);
end
h.set_pos_pid = uicontrol('Style', 'pushbutton', 'Position', [310 210 300 30], 'String', 'SET POS PID', 'FontSize', 16, 'Callback', @(x,y)(config_set_pos_pid(x,y,h)));

for i = 0:15
    col = floor(i / 4) + 6;
    row = mod(i, 4);
    h.mcoef{i+1} = uicontrol('Style', 'edit', 'Position', [10+100*col 160 - 50*row 100 30], 'FontSize', 16, 'String', 0);
    h.mcoefis{i+1} = uicontrol('Style', 'text', 'String', h.mcoef_labels{i+1} , 'Position', [10+100*col 190 - 50*row 100 15], 'FontSize', 12);
end
h.set_mcoef = uicontrol('Style', 'pushbutton', 'Position', [610 210 400 30], 'String', 'SET MCOEF', 'FontSize', 16, 'Callback', @(x,y)(config_set_mcoef(x,y,h)));

for i = 0:3
    col = floor(i / 4) + 10;
    row = mod(i, 4);
    h.pos_cmd_limits{i+1} = uicontrol('Style', 'edit', 'Position', [10+100*col 160 - 50*row 100 30], 'FontSize', 16, 'String', 0);
    h.pos_cmd_limitsis{i+1} = uicontrol('Style', 'text', 'String', h.pos_cmd_limits_labels{i+1} , 'Position', [10+100*col 190 - 50*row 100 15], 'FontSize', 12);
end
h.set_pos_cmd_limits = uicontrol('Style', 'pushbutton', 'Position', [1010 210 100 30], 'String', 'SET CMD LIMITS', 'FontSize', 16, 'Callback', @(x,y)(config_set_pos_cmd_limits(x,y,h)));

for i = 0:3
    col = floor(i / 4) + 11;
    row = mod(i, 4);
    h.ailevon_params{i+1} = uicontrol('Style', 'edit', 'Position', [10+100*col 160 - 50*row 100 30], 'FontSize', 16, 'String', 0);
    h.ailevon_paramsis{i+1} = uicontrol('Style', 'text', 'String', h.ailevon_params_labels{i+1} , 'Position', [10+100*col 190 - 50*row 100 15], 'FontSize', 12);
end
h.set_ailevon_params = uicontrol('Style', 'pushbutton', 'Position', [1110 210 100 30], 'String', 'SET AILEVONS', 'FontSize', 16, 'Callback', @(x,y)(config_set_ailevon_params(x,y,h)));

%h.get = uicontrol('Style', 'pushbutton', 'Position', [310 10 100 30], 'String', 'GET', 'FontSize', 16, 'Callback', @config_get_pid);
end

function config_set_pid(hObject, hEvent, h)
vals = zeros(4,3);
for i = 0:11
    col = floor(i / 4);
    row = mod(i, 4);
    v = str2num(get(h.pid{i+1}, 'String'));
    if ((numel(v) ~= 1))
        return;
    else
        vals(row+1,col+1) = v;
    end
end
crc32 = serial_lisam('send_pid', vals);
end

function config_set_pos_pid(hObject, hEvent, h)
vals = zeros(4,3);
for i = 0:11
    col = floor(i / 4);
    row = mod(i, 4);
    v = str2num(get(h.pos_pid{i+1}, 'String'));
    if ((numel(v) ~= 1))
        return;
    else
        vals(row+1,col+1) = v;
    end
end
crc32 = serial_lisam('send_pos_pid', vals);
end

function config_set_mcoef(hObject, hEvent, h)
vals = zeros(4,4);
for i = 0:15
    col = floor(i / 4);
    row = mod(i, 4);
    v = str2num(get(h.mcoef{i+1}, 'String'));
    if ((numel(v) ~= 1))
        return;
    else
        vals(row+1,col+1) = v;
    end
end
size(vals)
crc32 = serial_lisam('send_mcoef', vals);
end

function config_set_pos_cmd_limits(hObject, hEvent, h)
vals = zeros(4,1);
for i = 0:3
    col = floor(i / 4);
    row = mod(i, 4);
    v = str2num(get(h.pos_cmd_limits{i+1}, 'String'));
    if ((numel(v) ~= 1))
        return;
    else
        vals(row+1,col+1) = v;
    end
end
crc32 = serial_lisam('send_pos_cmd_limits', vals);
end

function config_set_ailevon_params(hObject, hEvent, h)
vals = zeros(4,1);
for i = 0:3
    col = floor(i / 4);
    row = mod(i, 4);
    v = str2num(get(h.ailevon_params{i+1}, 'String'));
    if ((numel(v) ~= 1))
        return;
    else
        vals(row+1,col+1) = v;
    end
end
crc32 = serial_lisam('send_ailevon_params', vals);
end

function display_config_values(h, pid_vals, pos_pid_vals, mcoef_vals, pos_cmd_limits, ailevon_params)

if (~ishandle(h.fig))
    return;
end
display_pid_values(h, pid_vals);
display_pos_pid_values(h, pos_pid_vals);
display_mcoef_values(h, mcoef_vals);
display_pos_cmd_limits_values(h, pos_cmd_limits);
display_ailevon_params_values(h, ailevon_params);

end

function display_pid_values(h, values)
for i = 0:11
    col = floor(i / 4);
    row = mod(i, 4);
    set(h.pidis{i+1}, 'String', sprintf('%s = %d', h.pid_labels{i+1}, (values(row+1,col+1))));
    v = str2num(get(h.pid{i+1}, 'String'));
    if ((numel(v) ~= 1) || (values(row+1,col+1) ~= v))
        set(h.pidis{i+1}, 'BackgroundColor', 'r');
    else
        set(h.pidis{i+1}, 'BackgroundColor', 'g');
    end
end
end

function display_pos_pid_values(h, values)
for i = 0:11
    col = floor(i / 4);
    row = mod(i, 4);
    %set(h.pid{i+1}, 'String', int2str(vals(row+1,col+1)));
    set(h.pos_pidis{i+1}, 'String', sprintf('%s = %d', h.pos_pid_labels{i+1}, (values(row+1,col+1))));
    v = str2num(get(h.pos_pid{i+1}, 'String'));
    if ((numel(v) ~= 1) || (values(row+1,col+1) ~= v))
        set(h.pos_pidis{i+1}, 'BackgroundColor', 'r');
    else
        set(h.pos_pidis{i+1}, 'BackgroundColor', 'g');
    end
end
end

function display_mcoef_values(h, values)
for i = 0:15
    col = floor(i / 4);
    row = mod(i, 4);
    %set(h.pid{i+1}, 'String', int2str(vals(row+1,col+1)));
    set(h.mcoefis{i+1}, 'String', sprintf('%s = %d', h.mcoef_labels{i+1}, (values(row+1,col+1))));
    v = str2num(get(h.mcoef{i+1}, 'String'));
    if ((numel(v) ~= 1) || (values(row+1,col+1) ~= v))
        set(h.mcoefis{i+1}, 'BackgroundColor', 'r');
    else
        set(h.mcoefis{i+1}, 'BackgroundColor', 'g');
    end
end
end

function display_pos_cmd_limits_values(h, values)
for i = 0:3
    col = floor(i / 4);
    row = mod(i, 4);
    %set(h.pid{i+1}, 'String', int2str(vals(row+1,col+1)));
    set(h.pos_cmd_limitsis{i+1}, 'String', sprintf('%s = %d', h.pos_cmd_limits_labels{i+1}, (values(row+1,col+1))));
    v = str2num(get(h.pos_cmd_limits{i+1}, 'String'));
    if ((numel(v) ~= 1) || (values(row+1,col+1) ~= v))
        set(h.pos_cmd_limitsis{i+1}, 'BackgroundColor', 'r');
    else
        set(h.pos_cmd_limitsis{i+1}, 'BackgroundColor', 'g');
    end
end
end

function display_ailevon_params_values(h, values)
for i = 0:3
    col = floor(i / 4);
    row = mod(i, 4);
    %set(h.pid{i+1}, 'String', int2str(vals(row+1,col+1)));
    set(h.ailevon_paramsis{i+1}, 'String', sprintf('%s = %d', h.ailevon_params_labels{i+1}, (values(row+1,col+1))));
    v = str2num(get(h.ailevon_params{i+1}, 'String'));
    if ((numel(v) ~= 1) || (values(row+1,col+1) ~= v))
        set(h.ailevon_paramsis{i+1}, 'BackgroundColor', 'r');
    else
        set(h.ailevon_paramsis{i+1}, 'BackgroundColor', 'g');
    end
end
end

