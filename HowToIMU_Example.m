% BNO055 IMU
% Tutorial - https://www.youtube.com/watch?v=0Oz1YxQ42X0

clear all
clear a
clc

%% initialization and calibration
% arduino interface
a = arduino;
% IMU interface
imu = bno055(a);
% calibration
status = readCalibrationStatus(imu);

%% reading imu data global
% IMU read data
imu_read = read(imu);
% reading data in matrix
imu_matrix = imu_read{:,:};
% average matrix columns
imu_mean =  mean(imu_matrix);

% final table with variables 
imu_table = array2table(imu_mean, 'VariableNames',{'Acc x','Acc y','Acc z','Gyro x','Gyro y','Gyro z','Mag x','Mag y','Mag z','Ort z','Ort y','Ort x'});

%%%%% acc (m/s)
acc_x = imu_mean(:,1);
acc_y = imu_mean(:,2);
acc_z = imu_mean(:,3);

%%%%% gryo (rad/s)
gyro_x_rad = imu_mean(:,4);
gyro_y_rad = imu_mean(:,5);
gyro_z_rad = imu_mean(:,6);
% gryo (deg/s)
gyro_x_deg = (gyro_x_rad)*(180/pi);
gyro_y_deg = (gyro_y_rad)*(180/pi);
gyro_z_deg = (gyro_z_rad)*(180/pi);

%%%%% mag (microTelsa)
mag_x = imu_mean(:,7);
mag_y = imu_mean(:,8);
mag_z = imu_mean(:,9);

%%%%% ort (rad)
ort_x_rad = imu_mean(:,12);
ort_y_rad = imu_mean(:,11);
ort_z_rad = imu_mean(:,10);
% ort (deg)
ort_x_deg = (ort_x_rad)*(180/pi);
ort_y_deg = (ort_y_rad)*(180/pi);
ort_z_deg = (ort_z_rad)*(180/pi);

% final matrix in (x,y,z) degrees
ort_matrix = [ort_x_deg, ort_y_deg, ort_z_deg];

%% Stream live ACC data
%{
acc_count = 0;
acc_data = [];

while acc_count < 200  %numer of samples - 10 samples per second, 20 seconds
    acc_count = acc_count + 1;

    % home base (reading matrix data and avg 10 samples)
    imu_read = read(imu);
    imu_matrix = imu_read{:,:};
    imu_mean =  mean(imu_matrix);
    
    % acc data
    acc_x = imu_mean(:,1);
    acc_y = imu_mean(:,2);
    acc_z = imu_mean(:,3);

    % data 
    acc_data = [acc_data; [acc_count, acc_x, acc_y, acc_z]];
    raw_acc_data = [acc_x, acc_y, acc_z];
    disp(raw_acc_data);

    if acc_count == 200
        disp('Run time end');
    end

end

acc_data;
% convert to data table
acc_table = array2table(acc_data, 'VariableNames',{'Time (0.1s)','Acc x','Acc y','Acc z'});
%}

%% Ploting live imu data

tiledlayout(3,1);
nexttile;

%%%%%%%%%%%%%%%%%% GRAPH Acceleration (x,y,z)
% X
acc_x_line = animatedline;
axis([1 100 -10 10]) % range of graph

acc_x_line.LineStyle = '-';
acc_x_line.LineWidth = 2;
acc_x_line.Color = '[0.99 0.01 0.01]'; % red
acc_x_line_title = title('Accelerometer','FontSize',11);
% labels
acc_x_line_xlabel = xlabel('Time (sec)');
acc_x_line_xlabel.FontSize = 9;
acc_x_line_ylabel = ylabel('Acceleration (m/s)');
acc_x_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Y
acc_y_line = animatedline;
axis([1 100 -10 10]) % range of graph

acc_y_line.LineStyle = '-';
acc_y_line.LineWidth = 2;
acc_y_line.Color = '[0.99 0.35 0.01]'; % orange
acc_y_line_title = title('Accelerometer','FontSize',11);
% labels
acc_y_line_xlabel = xlabel('Time (sec)');
acc_y_line_xlabel.FontSize = 9;
acc_y_line_ylabel = ylabel('Acceleration (m/s)');
acc_y_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Z
acc_z_line = animatedline;
axis([1 100 -10 10]) % range of graph

acc_z_line.LineStyle = '-';
acc_z_line.LineWidth = 2;
acc_z_line.Color = '[0.99 0.70 0.01]'; % yellow
acc_z_line_title = title('Accelerometer','FontSize',11);
% labels
acc_z_line_xlabel = xlabel('Time (sec)');
acc_z_line_xlabel.FontSize = 9;
acc_z_line_ylabel = ylabel('Acceleration (m/s)');
acc_z_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% legend
acc_legend = legend([acc_x_line acc_y_line acc_z_line], {'x axis','y axis','z axis'});
acc_legend.FontSize = 12;
acc_legend.Location = 'northeastoutside';
acc_legend.NumColumns = 1;
acc_legend.TextColor  = 'w';
acc_legend.Title.String = 'Accelerometer Data';
acc_legend.Title.Color = 'w';
acc_legend.Title.FontSize = 11;
hold off;

nexttile;

%%%%%%%%%%%%%%%%%% GRAPH Gyroscope (x,y,z)
% X
gyro_x_line = animatedline;
axis([1 100 -10 10]) % range of graph

gyro_x_line.LineStyle = '-';
gyro_x_line.LineWidth = 2;
gyro_x_line.Color = '[0.01 0.01 0.99]'; % blue
gyro_x_line_title = title('Gyroscope','FontSize',11);
% labels
gyro_x_line_xlabel = xlabel('Time (sec)');
gyro_x_line_xlabel.FontSize = 9;
gyro_x_line_ylabel = ylabel('Angular velocity (rad/s)');
gyro_x_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Y
gyro_y_line = animatedline;
axis([1 100 -10 10]) % range of graph

gyro_y_line.LineStyle = '-';
gyro_y_line.LineWidth = 2;
gyro_y_line.Color = '[0.01 0.31 0.99]'; % blue ish
gyro_y_line_title = title('Gyroscope','FontSize',11);
% labels
gyro_y_line_xlabel = xlabel('Time (sec)');
gyro_y_line_xlabel.FontSize = 9;
gyro_y_line_ylabel = ylabel('Angular velocity (rad/s)');
gyro_y_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Z
gyro_z_line = animatedline;
axis([1 100 -10 10]) % range of graph

gyro_z_line.LineStyle = '-';
gyro_z_line.LineWidth = 2;
gyro_z_line.Color = '[0.01 0.7 0.99]'; % bright blue
gyro_z_line_title = title('Gyroscope','FontSize',11);
% labels
gyro_z_line_xlabel = xlabel('Time (sec)');
gyro_z_line_xlabel.FontSize = 9;
gyro_z_line_ylabel = ylabel('Angular velocity (rad/s)');
gyro_z_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% legend
gyro_legend = legend([gyro_x_line gyro_y_line gyro_z_line], {'x axis','y axis','z axis'});
gyro_legend.FontSize = 12;
gyro_legend.Location = 'northeastoutside';
gyro_legend.NumColumns = 1;
gyro_legend.TextColor  = 'w';
gyro_legend.Title.String = 'Gyroscope Data';
gyro_legend.Title.Color = 'w';
gyro_legend.Title.FontSize = 11;
hold off;

nexttile;

%%%%%%%%%%%%%%%%%% GRAPH Magnetometer (x,y,z)
% X
mag_x_line = animatedline;
axis([1 100 -120 120]) % range of graph

mag_x_line.LineStyle = '-';
mag_x_line.LineWidth = 2;
mag_x_line.Color = '[0.01 0.99 0.01]'; % green
mag_x_line_title = title('Magnetometer','FontSize',11);
% labels
mag_x_line_xlabel = xlabel('Time (sec)');
mag_x_line_xlabel.FontSize = 9;
mag_x_line_ylabel = ylabel('Magnetic Field (micro T)');
mag_x_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Y
mag_y_line = animatedline;
axis([1 100 -120 120]) % range of graph

mag_y_line.LineStyle = '-';
mag_y_line.LineWidth = 2;
mag_y_line.Color = '[0.01 0.99 0.35]'; % green ish
mag_y_line_title = title('Magnetometer','FontSize',11);
% labels
mag_y_line_xlabel = xlabel('Time (sec)');
mag_y_line_xlabel.FontSize = 9;
mag_y_line_ylabel = ylabel('Magnetic Field (micro T)');
mag_y_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% Z
mag_z_line = animatedline;
axis([1 100 -120 120]) % range of graph

mag_z_line.LineStyle = '-';
mag_z_line.LineWidth = 2;
mag_z_line.Color = '[0.01 0.99 0.70]'; % bright green
mag_z_line_title = title('Magnetometer','FontSize',11);
% labels
mag_z_line_xlabel = xlabel('Time (sec)');
mag_z_line_xlabel.FontSize = 9;
mag_z_line_ylabel = ylabel('Magnetic Field (micro T)');
mag_z_line_ylabel.FontSize = 9;
% settings
set(gca,'Color','[0.45 0.45 0.5]');
grid on;
hold on;

% legend
mag_legend = legend([mag_x_line mag_y_line mag_z_line], {'x axis','y axis','z axis'});
mag_legend.FontSize = 12;
mag_legend.Location = 'northeastoutside';
mag_legend.NumColumns = 1;
mag_legend.TextColor  = 'w';
mag_legend.Title.String = 'Magnetometer Data';
mag_legend.Title.Color = 'w';
mag_legend.Title.FontSize = 11;
hold off;

%% Stream IMU data loop

imu_count = 0;
imu_data = [];
imu_data_rad = [];

while imu_count < 100  %numer of samples - 10 samples per second, 10 seconds
    imu_count = imu_count + 1;

    % home base (reading matrix data and avg 10 samples)
    imu_read = read(imu);
    imu_matrix = imu_read{:,:};
    imu_mean =  mean(imu_matrix);
    
    % acc data
    acc_x = imu_mean(:,1);
    acc_y = imu_mean(:,2);
    acc_z = imu_mean(:,3);

    % gryo (rad/s)
    gyro_x_rad = imu_mean(:,4);
    gyro_y_rad = imu_mean(:,5);
    gyro_z_rad = imu_mean(:,6);
    % gryo (deg/s)
    gyro_x_deg = (gyro_x_rad)*(180/pi);
    gyro_y_deg = (gyro_y_rad)*(180/pi);
    gyro_z_deg = (gyro_z_rad)*(180/pi);
    
    % mag (microTelsa)
    mag_x = imu_mean(:,7);
    mag_y = imu_mean(:,8);
    mag_z = imu_mean(:,9);

    % ort (rad)
    ort_x_rad = imu_mean(:,12);
    ort_y_rad = imu_mean(:,11);
    ort_z_rad = imu_mean(:,10);
    % ort (deg)
    ort_x_deg = (ort_x_rad)*(180/pi);
    ort_y_deg = (ort_y_rad)*(180/pi);
    ort_z_deg = (ort_z_rad)*(180/pi);

    % data 
    imu_data = [imu_data; [imu_count, acc_x, acc_y, acc_z, gyro_x_rad, gyro_y_rad, gyro_z_rad, mag_x, mag_y, mag_z, ort_x_deg, ort_y_deg, ort_z_deg]];
    imu_data_rad = [imu_data_rad; [imu_count, acc_x, acc_y, acc_z, gyro_x_rad, gyro_y_rad, gyro_z_rad, mag_x, mag_y, mag_z, ort_x_rad, ort_y_rad, ort_z_rad]];
    raw_imu_data = [acc_x, acc_y, acc_z, gyro_x_rad, gyro_y_deg, gyro_z_deg, mag_x, mag_y, mag_z, ort_x_deg, ort_y_deg, ort_z_deg];
    disp(raw_imu_data);

    if imu_count == 100
        disp('Run time end');
    end

    % drawing data points
    addpoints(acc_x_line, imu_count, acc_x);
    addpoints(acc_y_line, imu_count, acc_y);
    addpoints(acc_z_line, imu_count, acc_z);

    addpoints(gyro_x_line, imu_count, gyro_x_rad);
    addpoints(gyro_y_line, imu_count, gyro_y_rad);
    addpoints(gyro_z_line, imu_count, gyro_z_rad);

    addpoints(mag_x_line, imu_count, mag_x);
    addpoints(mag_y_line, imu_count, mag_y);
    addpoints(mag_z_line, imu_count, mag_z);
    
    drawnow;

end

imu_data;
% convert to data table
imu_table = array2table(imu_data, 'VariableNames',{'Time (0.1s)','Acc x','Acc y','Acc z','Gyro x','Gyro y','Gyro z','Mag x','Mag y','Mag z','Ort z','Ort y','Ort x'});

imu_data_acc = imu_data(:,2:4);
imu_data_gyro = imu_data(:,5:7);

tsa = timeseries(imu_data_acc);
tsg = timeseries(imu_data_gyro);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% issues %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% not reading mag due to matlab issues
% systems reads uncalibrated - likely due to newer version of matlab
% subtracting gravity vector earth frame before integration