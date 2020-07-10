clear all; close all; clc;

nRopeElements = 1;
# Overall Model will have position for each rope element and the drive and vehicle elements
nDof = 2 + nRopeElements ;

vInitial = 60 / 3.6; % m/s
vFinal = 30 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 230; % kg
mRope = .98 * lengthRope / nRopeElements; % kg
kRope = pi/4*(.0254 * (9/16))^2 * 207e9 / (lengthRope / nRopeElements); 

fDrive = 9547.902542; % N # Drive Force.  Probably wrap in function
preTension = 10675.2; % N


A = zeros(2*nDof);
# Order reduction to system of first order diff eq
A(nDof+1:2*nDof, 1:nDof) = eye(nDof);


# Create matrix entities for the inner rope elements
for i = 2:nRopeElements
    A(i, [nDof+i-1:nDof+i+1]) = kRope/mRope * [1, -2, 1];
endfor

#EOM for the drive element
A(1, [nDof+1:nDof+2]) = kRope/mDrive * [-1, 1];
#EOM for the vehicle element
A(nDof-1,[2*nDof-1:2*nDof])    = kRope/mVehicle * [-1, 1];


yInitial = zeros(2*nDof, 1);
# All elements initially moving at same velocity
yInitial(1:nDof) = vInitial;
##deltaX = preTension / kRope;
##for i = 1:nDof
##    yInitial(i+nDof) = i * deltaX
##endfor



F = zeros(2*nDof, 1);
# Apply force to the drive element and watch the strecth happen
F(1) = 0 * -fDrive;
##f(nDof) = preTension

eom = @(t,y) A*y + F 

[tSol, ySol] = ode15s(eom, [0,.4], yInitial)

plot(tSol, ySol);

xDrive = ySol(:, nDof+1);
vDrive = ySol(:,1);
xVehicle = ySol(:,2*nDof);
vVehicle = ySol(:, nDof+1);

plot(tSol, xDrive)
hold on 
plot(tSol, xVehicle)
legend('drive','vehicle')
title('position')

figure()
plot(tSol, vDrive)
hold on 
plot(tSol, vVehicle)
legend('drive','vehicle')
title('velocity')