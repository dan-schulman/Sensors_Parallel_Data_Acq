cam = readtable('ME499_Lab4_Part2_Camera_ShakingArm_w_Fused.csv');
rawArd = readtable('ME499_Lab4_Part2_ShakingArm_Fused.csv');

cam = cam(~any(ismissing(cam),2),:);
rawArd = rawArd(~any(ismissing(rawArd),2),:);

cam = cam{:,:};
rawArd = rawArd{:,:};

ard_sec = rawArd(:,1)/1000;
ax = rawArd(:,2);
ay = rawArd(:,3);
gz = rawArd(:,4);
enc_theta = rawArd(:,5);

cam_sec = cam(:,1);
cam_theta = cam(:,4);



%% Sensor Fusion Accelerometer + Gyro
freq_cutoff = 3;
sample_freq = 1/0.02;
theta_gyro =  cumtrapz(ard_sec,gz);

theta_accel = (180/pi()).*atan2(ax,ay);

w0 = 2*pi()*freq_cutoff;
HLP = tf([w0],[1,w0]);
GLP = c2d(HLP,1/sample_freq);
theta_accel_filt = lsim(GLP,theta_accel,ard_sec,0);
HHP = tf([1,0],[1,w0]);
GHP = c2d(HHP,1/sample_freq);
theta_gyro_filt = lsim(GHP,theta_accel,[],0);

theta_accel_gyro = theta_accel_filt + theta_gyro_filt;

%% Sensor Fusion Camera + Gyro
cam_sample_freq = 30;
theta_cam_filt = lowpass(cam_theta,freq_cutoff,cam_sample_freq);

theta_cam_filt_tt = timetable(datetime(0,0,0,0,0,cam_sec),theta_cam_filt);
theta_gyro_filt_tt = timetable(datetime(0,0,0,0,0,ard_sec),theta_gyro_filt);
theta_cam_filt_tt = retime(theta_cam_filt_tt,'regular','spline','TimeStep',duration(0,0,0,20));
theta_gyro_filt_tt = retime(theta_gyro_filt_tt,'regular','spline','TimeStep',duration(0,0,0,20));
tt_out = synchronize(theta_cam_filt_tt,theta_gyro_filt_tt);

tt_out = rmmissing(tt_out); % Remove NaN due to different start/end
resampled_time = seconds(tt_out.Time-datetime(0,0,0,0,0,0));
theta_cam_gyro = tt_out.theta_cam_filt+tt_out.theta_gyro_filt;

%% Plot
figure(1)
stairs(resampled_time,theta_cam_gyro);
hold on
stairs(ard_sec,theta_accel_gyro);
hold on
%stairs(ard_sec,theta_accel);
hold on
%stairs(ard_sec,theta_gyro);
hold on
%stairs(cam_sec,cam_theta);
hold on
stairs(ard_sec,enc_theta);
legend('Cam+Gyro','Accel+Gyro','enc');
%legend('\theta_{c+g}','\theta_{a+g}','\theta_{a}','\theta_{g}','\theta_{c}','\theta_{e}');
