%----------------------------------------------------------------
% Template created for the course SD2231 by Mikael Nybacka 2013
% Following file is the start file for the Simulink implementation 
% of the integration, model-based, and washout filter.
%----------------------------------------------------------------
% clear all;
% close all;
% clc;
addpath('scripts')
addpath('logged_data')
disp(' ');

% Set global variables so that they can be accessed from other matlab
% functions and files
global vbox_file_name YOURBETA

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
%global lf lr L mass Cf Cr Ratio
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
SWA_VBOX=vbo.channels(1, 9).data(trim_start:trim_end,1).*(pi/180);

% Taking away spikes in the data
for i=1:length(Time)
    if (i>1)
        if (abs(SWA_VBOX(i,1)-SWA_VBOX(i-1))>1 || abs(SWA_VBOX(i,1))>7)
            SWA_VBOX(i,1)=SWA_VBOX(i-1);
        end
    end
end

rollGrad_VBOX = vbo.channels(1, 28).data(trim_start:trim_end,1)./g;
rollGradDD = 1-rollGrad_VBOX;
T = 0.47; 

%        <TASK 3>


% suv = zeros(1000,5);
% i=1
% for T=0:0.01:5
%    sim('beta_inte.slx')
% %      suv(i) = simout;
%     suv2(i)=mean(simout);
%     suv3(i)=mode(simout);
% %     suv4(i)=
%     i=i+1
% end


%       </TASK 3>


%        < TASK 4>
%                                 
%       Beta_VBOX_smooth=smooth(Beta_VBOX,0.01,'rlowess'); 
% [e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(bicycle.signals.values',Beta_VBOX_smooth);
% [e_beta_mean1,e_beta_max1,time_at_max1,error1] = errorCalc(integration.signals.values',Beta_VBOX_smooth);
% [e_beta_mean2,e_beta_max2,time_at_max2,error2] = errorCalc(washout.signals.values',Beta_VBOX_smooth);
% disp(' ');
% fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
% fprintf('The Max error of Beta estimation is: %d \n',e_beta_max) 

%        < /TASK 4>


%%PLOT ALL RESULTS(TASK 9)[run UKF_start.m first, then
%%Init_for_washout_filter.m, then simulink file, then again
%%Init_for_washout_filter.m to plot the graphs successfully]

%[clear workspace after each test to plot correct graph];
Sideslip_trim_integ = Sideslip_Integ(1:size(Beta_VBOX),1);
Sideslip_trim_Washout = Sideslip_Washout(1:size(Beta_VBOX),1);
Sideslip_trim_Bicycle = Sideslip_Bicycle(1:size(Beta_VBOX),1);

plot(Time,Sideslip_trim_integ,'r')
hold on 
plot(Time,Sideslip_trim_Washout,'m')
hold on
plot(Time,beta_ukf,'k')
hold on
plot(Time,Beta_VBOX,'g')
hold on
plot(Time,Sideslip_trim_Bicycle,'b')
hold off


