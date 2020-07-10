clear all; close all; clc;
pkg load control

vInitial = 60 / 3.6; % m/s
vFinal = 30 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 600; % kg


kRope = 13402517.3; # Wire
kRope = 6935401.117; # Polymer 
##kRope /= 100

A = zeros(4);
A([1:2],[3:4]) = [-kRope/mDrive, kRope/mDrive;
                 kRope/mVehicle, -kRope/mVehicle];    
A([3:4],[1:2]) = eye(2);

B = zeros(4,1);
B(1) = 1/mDrive;
C = eye(4);

D = 0;       

G = ss(A, B, C, D)


Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
R = .00001;
     
K = lqr(A,B,Q,R)

Gcl = feedback(G*K, eye(4));
##Gcl2 = ss(A-B*K, B, C, D)
F = 1000;

T = 0:.01:10;
U = ones(size(T'))*[vFinal, vFinal 0 0];

x0 = [vInitial, vInitial, 0, 0];

[y,t,x] = lsim(Gcl, U, T, x0);
lsim(Gcl, U, T, x0);
##sys = ss((A-B*K), B, C, D)