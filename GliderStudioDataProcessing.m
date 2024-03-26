% Plot Data for Glider Studio

close all
clear all
clc

GliderName = 'V3_';

mass = 0.02;

filename = 'Q1-standing_01.csv';
data = readmatrix(filename);


% Find first frame 
k = find(data(:,1)> 0,1);

% Discard rows before frame 1
data = data(k:end, :);

% May want to replace this with something determining if x position before varies significantly from rest of trajectory
% Find where X is not 0 (Important to keep markers hidden for this?)
k = find(data(:,3) ~= 0,1);

% Discard rows before start of flight
data = data(k:end, :);

% Column order = Frame, Time (s), X Rotation, Y Rotation, Z Rotation, X
%               Position, Y Position, Z Position

frame = data(:, 1);
time = data(:, 2);
X_Rotation = data(:, 3);
Y_Rotation = data(:, 4);
Z_Rotation = data(:, 5);
X_Position = data(:, 6);
Y_Position = data(:, 7);
Z_Position = data(:, 8);


% Transform to normal coordinate system
Z = Y_Position; % Should be -Y but to keep height above 0 as positive, Z refers to magnitude
Y = -Z_Position;
X = - X_Position;
yaw = Y_Rotation;       % About new z axis
pitch = - Z_Rotation;   % About new y axis
roll = -X_Rotation;     % About new x axis

% Velocity Estimation
%   Estimate with dx/dt ~ x2 - x1 / t2 - t1 with end BC of velocity = 0 at end of flight
Velocity_X = [(X(2:end) - X(1:end-1))./(time(2:end) - time(1:end-1)); 0];
Velocity_Y = [(Y(2:end) - Y(1:end-1))./(time(2:end) - time(1:end-1)); 0]; 
Velocity_Z = [(Z(2:end) - Z(1:end-1))./(time(2:end) - time(1:end-1)); 0]; 
VelocityMagnitude = sqrt( Velocity_X.^2 + Velocity_Y.^2 + Velocity_Z.^2 );


% Acceleration Estimation
%   Estimate with d^2x/dt^2 at x2 ~ x3 - 2x2 + x1 / (dt^2) with end and start BC of acceleration = 0 at end of flight
dt = time(2) - time(1); % Assume constant dt
Acceleration_X = [0 ; (X(3:end) - 2*X(2:end-1) + X(1:end-2))/(dt^2); 0];
Acceleration_Y = [0 ; (Y(3:end) - 2*Y(2:end-1) + Y(1:end-2))/(dt^2); 0]; 
Acceleration_Z = [0 ; (Z(3:end) - 2*Z(2:end-1) + Z(1:end-2))/(dt^2); 0];
AccelerationMagnitude = sqrt( Acceleration_X.^2 + Acceleration_Y.^2 + Acceleration_Z.^2 );

% Net Force 
F_net = mass*AccelerationMagnitude;


f = figure;
subplot(3,2,1);
plot(X, Z)
xlabel("X")
ylabel("Z")

subplot(3,2,2); 
plot(X, roll)
xlabel("X")
ylabel("Roll")

subplot(2,2,3); 
plot(X, pitch)
xlabel("X")
ylabel("Pitch")

subplot(2,2,4); 
plot(X, yaw)
xlabel("X")
ylabel("Yaw")

figurename = [GliderName, filename(1:(end-4)), '_Position_Orientation_vs_X'];
print(f, figurename, '-dpng', '-r0')

f = figure;
subplot(2,1,1);
plot(X, VelocityMagnitude)
xlabel("X")
ylabel("Velocity")

subplot(2,1,2);
plot(X, F_net)
xlabel("X")
ylabel("Net Force")

figurename = [GliderName, filename(1:(end-4)), '_Velocity_and_NetForce_vs_X'];
print(f, figurename, '-dpng', '-r0')

f = figure;
subplot(3,2,1);
plot(time, X)
xlabel("Time")
ylabel("X")

subplot(3,2,2);
plot(time, Y)
xlabel("Time")
ylabel("Y")

subplot(3,2,3);
plot(time, Z)
xlabel("Time")
ylabel("Z")

subplot(3,2,4);
plot(time, roll)
xlabel("Time")
ylabel("Roll")

subplot(3,2,5); 
plot(time, pitch)
xlabel("Time")
ylabel("Pitch")

subplot(3,2,6); 
plot(time, yaw)
xlabel("Time")
ylabel("Yaw")

figurename = [GliderName, filename(1:(end-4)), '_Position_Orientation_vs_Time'];
print(f, figurename, '-dpng', '-r0')

f = figure;
subplot(2,1,1);
plot(time, VelocityMagnitude)
xlabel("Time")
ylabel("Velocity")

subplot(2,1,2);
plot(time, F_net)
xlabel("Time")
ylabel("Net Force")

figurename = [GliderName, filename(1:(end-4)), '_Velocity_and_NetForce_vs_Time'];
print(f, figurename, '-dpng', '-r0')