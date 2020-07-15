clear all; close all; clc;

pkg load control  # Ocatave specific

### Model Parameters
vInitial = 30 / 3.6; % m/s
vFinal = 0 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 600; % kg

##kRope = 13402517.3; # Wire
kRope = 6935401.117; # Polymer 
kRope /= lengthRope; # Account for length of rope

#damping coefficent

wn = 1/(2*pi) * sqrt(kRope/mVehicle); # Simple sqrt(k/m)
R = (mVehicle/mDrive);
wn = 1/(2*pi) * sqrt(kRope/mVehicle * (1+R)); # from motor analogies

zeta = 10;
bDamp = zeta * (2*mDrive*wn); # Shen System Dynamics, Equation 3.16.  I think this may only hold true for sisio 1dof system

tStep = 1/10000;
tFinal = 6;
tOffset = .5;

fMax = 13982.6087; #Barrier Dynamic Model, Cell 

### State Space Model
A = zeros(4);

A([1:2],[3:4]) = [-kRope/mDrive, kRope/mDrive;
                 kRope/mVehicle, -kRope/mVehicle];    
                 
A([1:2],[1:2]) =  [-bDamp/mDrive, bDamp/mDrive;
                 bDamp/mVehicle, -bDamp/mVehicle];
                 
A([3:4],[1:2]) = eye(2);

B = zeros(4,1);
B(1) = -1/mDrive;

# Assume we can see full state or get it with estimator
C = eye(4);
D = 0;       

#Compose Model
G = ss(A, B, C, D);
names = {'V_d', 'V_v','X_d','X_v'};
G.stname = names;
G.outputname = names;

#Compute the energy to take out of the system via Impulse-Momentum Equation.
energyOut = (mVehicle + mDrive) * (vInitial - vFinal);

#Time to remove energy at constant force
tImpulse = energyOut / (fMax);
iImpulse = round(tImpulse / tStep); #index


T = 0:tStep:tFinal;
U = zeros(size(T));

uOffset = round(tOffset / tStep);
# Apply Maximum Force "on-off"
U(uOffset + [1:iImpulse]) = fMax;
U = cfc_filter(U, 1); #Filter FOrce to give softer lead in

# Initial Conditions
x0 = [vInitial, vInitial, 0, 0];

#Simulate using Octave linear simulator for state space systems
##lsim(G, U, T, x0);

[Y, T, X] = lsim(G, U, T, x0);


### Plot results 

figure()

subplot(311)
plot(T, Y(:, 1))
hold on
plot(T, Y(:, 2))
plot(T, vFinal * ones(size(T)), '--k')
ylabel('Velocity [m/s]')

title('Mass-Spring-Mass Simulation Results')

subplot(312)
plot(T, Y(:, 3))
hold on
plot(T, Y(:, 4))
ylabel('Position [m]')

legend('Drive', 'Vehicle')

subplot(313)
plot(T, U)
xlabel('Time [s]')
ylabel('Drive Force (Input) [N]')
ylim([0, 1.1*max(U)])


figure()
plot(T(2:end), 1/9.81*diff(Y(:,1))/tStep)
hold on 
plot(T(2:end), 1/9.81*diff(Y(:,2))/tStep)
legend('Drive', 'Vehicle')
title('Coarse Approximation Acceleration')