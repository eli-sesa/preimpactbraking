clear all; close all; clc;

# This script aims to address the conversation with Tom about prescribing a simple 
# displacement input to eliminate mechanical vibration. (Tom idea)

pkg load control

vInitial = 60 / 3.6; % m/s
vFinal = 30 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 200; % kg

kRope = 13402517.3; # Wire
##kRope = 6935401.117; # Polymer 
kRope /= lengthRope;

tStep = 1/10000;
tFinal = 10;

fMax = 10000;

# State vector [x0', x0]'
# Input vector [xi]

A = [0, -kRope/mVehicle;
    1, 0];

# Let our input be spring position
B = [kRope/mVehicle; 0];

C = eye(2)
D = 0

T = 0:tStep:tFinal;
iSwitch = round(length(T)/2)

velVec = zeros(size(T));
velVec(1:iSwitch) = vInitial;
velVec(iSwitch+1:end) = vFinal;

U = cumtrapz(T, velVec);

U(iSwitch+1:end) += (9.81 * mVehicle)/kRope; # Apply force necessary to decelerate by 1 G w spring


names = {'vehicleVelocity','vehiclePosition'}
G = ss(A, B, C, D)
G.stname = names
G.outputname = names
G.inputname = {'xi'}

x0 = [vInitial, 0];
lsim(G, U, T, x0);

wn = (1/(2*pi)) * sqrt(kRope/mVehicle)# "anitresonant Frequency" in Mass - Spring - Mass system.


##subplot(211)
##plot(T, velVec)
##subplot(212)
##plot(T, U)