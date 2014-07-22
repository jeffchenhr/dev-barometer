function main3d
% assume FS_SEL=3 and AFS_SEL=3:
% 2048 counts/g
% 16.4 counts/(deg/s)

gyro_fac = 2000 / 2^15;
mag_fac  = 1;
acc_fac  = 16 / 2^15;

global data;
data.kill = 0;

addpath('graphics');
addpath('geom');

ViewSize = 3;
ViewHeight = 3;

vp.c1l = 0.05;
vp.c1w = 0.35;
vp.c2l = 0.5;
vp.c2w = 0.2;
vp.c3l = 0.75;
vp.c3w = 0.2;

figure; set(gcf,'CloseRequestFcn', @my_closereq);
subplot('Position', [vp.c1l 0.55 vp.c1w 0.4]);
hqs = paintQuadshot([0 0 0], eye(3));
view(3); grid on;
xlabel('North (X)'); ylabel('East (Y)'); zlabel('Down (Z)');
set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
set(gca, 'XLim', [-ViewSize ViewSize], 'YLim', [-ViewSize ViewSize], 'ZLim', [-ViewHeight ViewHeight]); 
hqstitle = title('uninit');

subplot('Position', [vp.c1l 0.05 vp.c1w 0.4]);
hqs_goal = paintQuadshot([0 0 0], eye(3));
view(3); grid on;
%xlabel('X'); ylabel('Y'); zlabel('Z');
xlabel('North (X)'); ylabel('East (Y)'); zlabel('Down (Z)');
set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
set(gca, 'XLim', [-ViewSize ViewSize], 'YLim', [-ViewSize ViewSize], 'ZLim', [-ViewHeight ViewHeight]); 
hqs_goal_title = title('uninit');




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

% /dev/tty.usbserial-A9KRBL95
% /dev/tty.usbserial-FTEZOHBA
serial_write_mac('open', '/dev/tty.usbserial-FTEZOHBA', 115200)
pause(0.1);
while (~data.kill)
    [Dnew, Snew] = serial_write_mac('readIMUall');
    if (size(Dnew,2) > 0)
        D = Dnew(:,end);
        S = Snew(:,end);
        
        TD = Dnew(1,end) - Dnew(1,1);
        %fprintf('periodic: %6.4f  mag: %6.4f  gyro: %6.4f  acc: %6.4f\n', (Dnew(23,end) - Dnew(23,1)) / TD, (Dnew(24,end) - Dnew(24,1)) / TD, (Dnew(25,end) - Dnew(25,1)) / TD, (Dnew(26,end) - Dnew(26,1)) / TD);
        fprintf('periodic: %6.4f  mag: %6.4f  gyro: %6.4f  wrl: %d\n', (Dnew(23,end) - Dnew(23,1)) / TD, (Dnew(24,end) - Dnew(24,1)) / TD, (Dnew(25,end) - Dnew(25,1)) / TD, Dnew(26,end));
        
        R = RMAT_OF_QUAT(D(30:33));
        paintQuadshot([0 0 0], R, hqs); %RotationMatrixY(pi)*

        R = RMAT_OF_QUAT(D(34:37));
        paintQuadshot([0 0 0], R, hqs_goal); % * RotationMatrixY(pi/2)
        
        status = serial_write_mac('status');
        set(hqstitle, 'String', sprintf('%6.3f (%d bytes, %d pkt/update)', D(1), status(6), size(Dnew,2)));

        D(5:7)  = D(5:7)  * gyro_fac;
        D(8:10) = D(8:10) * acc_fac;
        D(2:4)  = D(2:4)  * mag_fac;
        
        set(hgyro, 'YData', D(5:7));
        set(hacc, 'YData', D(8:10));
        set(hmag, 'YData', D(2:4));
        set(hrates, 'YData', D(27:29));
        set(hgyro_title, 'String', sprintf('GYRO [deg/s]: x=%6.3f y=%6.3f z=%6.3f', D(5), D(6), D(7)));
        set(hacc_title, 'String', sprintf('ACC [g]: x=%6.3f y=%6.3f z=%6.3f', D(8), D(9), D(10)));
        set(hmag_title, 'String', sprintf('MAG [uT]: x=%6.3f y=%6.3f z=%6.3f', D(2), D(3), D(4)));
        set(hrates_title, 'String', sprintf('RATES: p=%6.3f q=%6.2f r=%6.3f', D(27), D(28), D(29)));

        set(hradio, 'YData', D(17:22));
        set(hcmd, 'YData', D(23:26));
        set(hcmd_title, 'String', sprintf('CMD: %d', D(26)));
        set(hactuator, 'YData', D(11:16));
        
        if (D(21) > 0)
            set(hgear, 'FaceColor', 'g');
        else
            set(hgear, 'FaceColor', 'w');            
        end
        if (D(22) > 0)
            set(hflap, 'FaceColor', 'g');
        else
            set(hflap, 'FaceColor', 'w');            
        end
        if (abs(D(21)) < 7500)
            set(haildr, 'FaceColor', 'g');
        else
            set(haildr, 'FaceColor', 'w');            
        end
        if (abs(D(22)) < 7500)
            set(helevdr, 'FaceColor', 'g');
        else
            set(helevdr, 'FaceColor', 'w');            
        end
        
        set(hstatus_title, 'String', sprintf('AHRS: %d, ALIGNER: %d, ARMING: %d, RADIO: %d [%d/%d/%d/%d], [%d/%d/%d]', S(1), S(2), S(8), S(3), S(4), S(5), S(6), S(7),  D(38), D(39), D(40)));
        set(hactuator_title, 'String', sprintf('ACTUATORS [%4d %4d %4d %4d %4d %4d]', D(11), D(12), D(13), D(14), D(15), D(16)));
        set(hradio_title, 'String', sprintf('RADIO [%6d %6d %6d %6d %6d %6d]', D(17), D(18), D(19), D(20), D(21), D(22)));

        if (S(1) == 1)
            set(hahrs_led, 'FaceColor', 'g');
        else
            set(hahrs_led, 'FaceColor', 'r');            
        end
        if (S(2) == 2)
            set(haligner_led, 'FaceColor', 'g');
        else
            set(haligner_led, 'FaceColor', 'r');            
        end
        if (S(3) == 0)
            set(hradio_led, 'FaceColor', 'g');
        else
            set(hradio_led, 'FaceColor', 'r');            
        end
        
        %set(hquat, 'YData', D(23:26));
        %set(hquat, 'YData', D(30:33));
        
        drawnow;
    end
    pause(0.01);
end
fprintf('EXITING\n');
pause(0.01);
serial_write_mac('close');
delete(gcf);

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
