%----------------------------------------------------------------
% Template created for the course SD2231 by Mikael Nybacka 2013
% Following file is the start file for the state estimation using
% Uncented Kalman Filter (UKF).
%----------------------------------------------------------------
% clear all;
% close all;
% clc;
addpath('scripts')
addpath('logged_data')
disp(' ');

% Set global variables so that they can be accessed from other matlab
% functions and files
global lf lr Cf Cr mass Iz vbox_file_name YOURBETA

%----------------------------
% LOAD DATA FROM VBOX SYSTEM
%----------------------------
%vbox_file_name='logged_data/Lunda_test_140411/Stand_Still_no2.VBO'; %stand still logging, engine running
% vbox_file_name='logged_data/Lunda_test_140411/Circle_left_R13m_no2.VBO'; %circle test left, roughly 13m in radius
% vbox_file_name='logged_data/Lunda_test_140411/Slalom_35kph.VBO'; %slalom entry to the left @ first cone, 35kph
vbox_file_name='logged_data/Lunda_test_140411/Step_Steer_left_80kph.VBO'; %Step steer to the left in 80kph
% vbox_file_name='logged_data/Lunda_test_140411/SWD_80kph.VBO'; %Sine with dwell, first turn to the right, 80kph

vboload
%  Channel 1  = satellites
%  Channel 2  = time
%  Channel 3  = latitude
%  Channel 4  = longitude
%  Channel 5  = velocity kmh
%  Channel 6  = heading
%  Channel 7  = height
%  Channel 8  = vertical velocity kmh
%  Channel 9  = steerang
%  Channel 10 = vxcorr
%  Channel 11 = slipcorr
%  Channel 12 = event 1 time
%  Channel 13 = rms_hpos
%  Channel 14 = rms_vpos
%  Channel 15 = rms_hvel
%  Channel 16 = rms_vvel
%  Channel 17 = latitude_raw
%  Channel 18 = longitude_raw
%  Channel 19 = speed_raw
%  Channel 20 = heading_raw
%  Channel 21 = height_raw
%  Channel 22 = vertical_velocity_raw
%  Channel 23 = true_head
%  Channel 24 = slip_angle 
%  Channel 25 = pitch_ang. 
%  Channel 26 = lat._vel.
%  Channel 27 = yaw_rate
%  Channel 28 = roll_angle 
%  Channel 29 = lng._vel.
%  Channel 30 = slip_cog
%  Channel 31 = slip_fl
%  Channel 32 = slip_fr
%  Channel 33 = slip_rl
%  Channel 34 = slip_rr
%  Channel 35 = yawrate
%  Channel 36 = x_accel
%  Channel 37 = y_accel
%  Channel 38 = temp
%  Channel 39 = pitchrate
%  Channel 40 = rollrate
%  Channel 41 = z_accel

%-----------------------------------
% SET VEHICLE DATA FOR THE VOLVO V40
%-----------------------------------
Rt=0.312;           % Tyre radius (m)
lf=0.41*2.55;       % Distance from CoG to front axis (m)
lr=2.55-lf;         % Distance from CoG to rear axis (m)
L=lf+lr;            % Wheel base (m)
h=0.2*L;            % Hight from ground to CoG (m)
mass=1435-80;       % Mass (kg)
Iz=2380;            % Yaw inertia (kg-m2)
tw=1.565;           % Track width (m)
Ratio=17;           % Steering gear ratio
Cf=70000;          % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
Cr=70000;          % Lateral stiffness rear axle (N/rad) [FREE TO TUNE]
Lx_relax=0.05;      % Longitudinal relaxation lenth of tyre (m)
Ly_relax=0.15;      % Lateral relaxation lenth of tyre (m)
Roll_res=0.01;      % Rolling resistance of tyre
rollGrad=5*(pi/180);% Rollangle rad per g (rad/g)
rx=0.4;             % Distance from IMU to CoG x-axle (m)
ry=0;               % Distance from IMU to CoG y-axle (m)
rz=0;               % Distance from IMU to CoG z-axle (m)

%--------------------------------------
% SET ENVIRONEMNTAL PARAMETERS FOR TEST
%--------------------------------------
Mu=0.95;             % Coefficient of friction
g=9.81;             % Gravity constant (m/s^2)


%--------------------------------------------
% SET VARIABLES DATA FROM DATA READ FROM FILE
%--------------------------------------------
trim_start=1;
trim_end=length(vbo.channels(1, 2).data);

Time=(vbo.channels(1, 2).data(trim_start:trim_end,1) - vbo.channels(1, 2).data(1,1));
yawRate_VBOX = vbo.channels(1, 35).data(trim_start:trim_end,1).*(-pi/180); %signal is inverted hence (-)
vx_VBOX = vbo.channels(1, 5).data(trim_start:trim_end,1)./3.6;
vy_VBOX = vbo.channels(1, 26).data(trim_start:trim_end,1)./3.6;
ax_VBOX = vbo.channels(1, 36).data(trim_start:trim_end,1).*g;
ay_VBOX = vbo.channels(1, 37).data(trim_start:trim_end,1).*g;
Beta_VBOX = vbo.channels(1, 30).data(trim_start:trim_end,1).*(pi/180);
SWA_VBOX=vbo.channels(1, 9).data(trim_start:trim_end,1).*(pi/180)/Ratio;

% Taking away spikes in the data
for i=1:length(Time)
    if (i>1)
        if (abs(SWA_VBOX(i,1)-SWA_VBOX(i-1))>1 || abs(SWA_VBOX(i,1))>7)
            SWA_VBOX(i,1)=SWA_VBOX(i-1);
        end
    end
end
n = length(Time);
dt = Time(2)-Time(1);

%----------------------------------------------
% SET MEASUREMENT AND PROCESS NOICE COVARIANCES
%----------------------------------------------
% Use as starting value 0.1 for each of the states in Q matrix
Q= eye(3)* 0.1;
%Q= [0.1 0 0; 0 0.1 0; 0 0 0.1];

% Use as starting value 0.01 for each of the measurements in R matrix
R= eye(3)*0.01;

%--------------------------------------------------
% SET INITIAL STATE AND STATE ESTIMATION COVARIANCE
%--------------------------------------------------
x_0=[vx_VBOX(1); 0; yawRate_VBOX(1)];
M = x_0;
P = Q; 
Y = [vx_VBOX'; ay_VBOX'; yawRate_VBOX'];

% M = [vx_VBOX(1); vy_VBOX(1); yawRate_VBOX(1)];
%P = P_0;
% M = eye(size(i));
% P = eye(size(i));
% Y = eye(size(i));

%-----------------------
% INITIALISING VARIABLES
%-----------------------

%Parameters that might be needed in the measurement and state functions are added to predictParam
predictParam.dt=dt; 

% Handles to state and measurement model functions.
state_func_UKF = @Vehicle_state_eq;
meas_func_UKF = @Vehicle_measure_eq;

a= state_func_UKF;
h= meas_func_UKF;
param.dt = dt;
%-----------------------
% FILTERING LOOP FOR UKF 
%-----------------------
disp(' ');
disp('Filtering the signal with UKF...');

for i = 2:n

 param.delta = SWA_VBOX(i);
    % ad your predict and update functions, see the scripts ukf_predict1.m
    % and ukf_update1.m
    
    %ukf_predict
    [M(:,i),P] = ukf_predict1(M(:,i-1),P,a,Q,param);%,alpha,beta,kappa,mat)
  
    % ukf_update
    [M(:,i),P,K,MU,S,LH] = ukf_update1(M(:,i),P,Y(:,i),h,R,param);%,alpha,beta,kappa,mat)
    
    if i==round(n/4)
        disp(' ');
        disp('1/4 of the filtering done...');
        disp(' ');
    end
    if i==round(n/2)
        disp(' ');
        disp('1/2 of the filtering done...');
        disp(' ');
    end
    if i==round(n*(3/4))
        disp(' ');
        disp('3/4 of the filtering done... Stay tuned for the results...');
        disp(' ');
    end
end

%----------------------------------------
% CALCULATE THE SLIP ANGLE OF THE VEHICLE
%----------------------------------------
YOURBETA = M(2,:)./M(1,:);

%---------------------------------------------------------
% CALCULATE THE ERROR VALUES FOR THE ESTIMATE OF SLIP ANGLE
%---------------------------------------------------------
Beta_VBOX_smooth=smooth(Beta_VBOX,0.01,'rlowess'); 
[e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(YOURBETA',Beta_VBOX_smooth);
disp(' ');
fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
fprintf('The Max error of Beta estimation is: %d \n',e_beta_max);


%-----------------
% PLOT THE RESULTS
%-----------------

beta_ukf=YOURBETA';
% plot(Time, beta_ukf);
% hold on
% plot(Time, SWA_VBOX,'r');
% hold off


