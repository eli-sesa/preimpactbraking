clear all; close all; clc;
pkg load control

vInitial = 60 / 3.6; % m/s
vFinal = 30 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 200; % kg

R = mVehicle / mDrive;


kRope = 13402517.3; # Wire
kRope = 6935401.117; # Polymer 
kRope /= lengthRope;

tStep = 1/10000;
tFinal = 6

fMax = 10000;


A = zeros(4);
A([1:2],[3:4]) = [-kRope/mDrive, kRope/mDrive;
                 kRope/mVehicle, -kRope/mVehicle];    
A([3:4],[1:2]) = eye(2);

B = zeros(4,1);
# Let input be in term of 10,000 N.  This keeps the input magnitue low for automatic axis for plotting.  G proper
B(1) = -fMax/mDrive;

C = zeros(1, 4);
C(3) = 1;
C = eye(4)
D = 0;       

G = ss(A, B, C, D)
names = {'V_d', 'V_v','X_d','X_v'};
G.stname = names
G.outputname = names

T = 0:tStep:tFinal;
U = zeros(size(T));
U = 2 * ones(size(T'));
##U = 50 * exp(-T/.3);

wn = sqrt(kRope/mVehicle * (1+R)) # Natural Frequency rad/s
wn /= (2*pi) # hz

wa = sqrt(kRope/mVehicle)# "anitresonant Frequency" rad/s
wa /= (2*pi) # hz

x0 = [vInitial, vInitial, 0, 0];
lsim(G, U, T, x0);


Q = eye(4);
R = .0001;
