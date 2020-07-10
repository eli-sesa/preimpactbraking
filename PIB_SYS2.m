clear all; close all; clc;
pkg load control

figure()
vInitial = 60 / 3.6; % m/s
vFinal = 30 / 3.6; % m/s
lengthRope = 120; % M
mVehicle = 3500; % kg
mDrive = 600; % kg

##kRope = 13402517.3; # Wire
kRope = 6935401.117; # Polymer 
kRope /= lengthRope;

tStep = 1/10000;
tStep = .01
tFinal = 6;

fMax = 13982.6087; #Barrier Dynamic Model, Cell 


A = zeros(4);
A([1:2],[3:4]) = [-kRope/mDrive, kRope/mDrive;
                 kRope/mVehicle, -kRope/mVehicle];    
A([3:4],[1:2]) = eye(2);

B = zeros(4,1);
# Let input be in term of 10,000 N.  This keeps the input magnitue low for automatic axis for plotting.  G proper
B(1) = -1/mDrive;

C = eye(4);
D = 0;       

G = ss(A, B, C, D)
names = {'V_d', 'V_v','X_d','X_v'};
G.stname = names
G.outputname = names

#Compute the energy to take out of the system
energyOut = 1/2* (mVehicle + mDrive) * (vInitial ^ 2 - vFinal ^ 2); # this is wrong.  Gives 
energyOut = (mVehicle + mDrive) * (vInitial - vFinal)
#Time to remove energy at constant force
tImpulse = energyOut / (fMax);
iImpulse = round(tImpulse / tStep);


T = 0:tStep:tFinal;
##U = 2 * ones(size(T'));
##U = 50 * exp(-T/.3);
uOffset = round(.5 / tStep)
U = zeros(size(T));
U(uOffset + [1:iImpulse]) = fMax;


##plot(T,U)

x0 = [vInitial, vInitial, 0, 0];
lsim(G, U, T, x0);

 [Y, T, X] = lsim(G, U, T, x0);

 figure()
subplot(211)
plot(T, Y(:, 1))
hold on
plot(T, Y(:, 2))
plot(T, vFinal * ones(size(T)))
ylabel('Velocity [m/s]')

subplot(212)
plot(T, Y(:, 3))
hold on
plot(T, Y(:, 4))
ylabel('Position [m]')
xlabel('Time [s]')
legend('Drive', 'Vehicle')

##step(G, 10)
##
##wn =  300
##wn2 = 400
##zeta = sqrt(2)/2;
##
##
##p(1) = -zeta*wn + i*sqrt(1-zeta^2)
##p(2) = -zeta*wn + i*sqrt(1-zeta^2)
##p(4) = -zeta*wn2 + i*sqrt(1-zeta^2)
##p(3) = -zeta*wn2 + i*sqrt(1-zeta^2)
##
##K = place(A, B, p)
##
##Gcl = ss(A-B*K, B, C, D)
##step(Gcl)
##
##
##T = 0:.001:1;
##U = ones(size(T'))*[vFinal, vFinal 0 0];
##U = 2500 * ones(size(T'));
##
##[y, t, x] = lsim(G, U, T, x0);
##x0 = [vInitial, vInitial, 0, 0];