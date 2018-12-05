clc
clear all
close all

%% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb im p_3in p_9in Kp Ki

Fb= 25000; %Friction value 

Rw = 0.5; %Ohms
Lw = 0.015; %Henrys
Tm = 1.718; %Webers
M = 2200; %kg, vehicle mass
bt = 0.05; %Nms/rad
R = 0.2; %m, wheel radius
Gr = 5; %:1 gear ratio
Cr = 0.006; %rolling resistance coefficient
g = 9.81; %m/s^2 
Cd = 0.32; %drag coefficient
rho = 1.21; %kg/m^3, air density
Af = 2.05; %m^2 vehicle frontal area
n = 0.0001; 
vi = 20; %m/s initial vehicle speed
im = 0; %i200nitial currrent in recharging circuit
k = (Tm^2)/Lw; %stiffness, simplification of I/GY to C
Kp = 165; %proportional gain
Ki = 500; %integral gain



%% Run ODE45 Solver 

% define initial conditions
initial = [0 0 0 0];

% setup time array
tspan = [0:0.05:300];

% call ODE solver
[t, s] = ode45(@lab5_eqns,tspan,initial);

% pre-allocate additional output vectors
ext = zeros(length(t),4);
ds = zeros(length(t),4);

% obtain actual function outputs
for i = 1:length(t)    
    [ds(i,:) ext(i,:)] = lab5_eqns(t(i), s(i,:));
end

vref = zeros(length(t),1);
for i = 1:length(t)
    vref(i) = LA92Oracle(t(i));
end

%% Bode plot

A = [(-Rw/Lw) -(Tm*Gr)/(R*M); (Tm*Gr)/(R*Lw) -(bt*Gr^2)/(M*R^2)];
B = [1;0];
C = [0 1/M];
D = [0];

s = tf('s');
I = eye(2);

TF = C*inv(s*I - A)*B + D;
figure
bode(TF)

%% Plot Figures

figure('Name','Actual and Reference Velocity Profiles','NumberTitle','off','Color','white')
plot(t,ds(:,4),'r',t,ds(:,3),'--k'), grid on
title('Actual and Reference Velocity Profiles')
legend('Actual Vehicle Velocity','LA92 Reference Velocity','Location','Best')
axis([32,54,4,8])
ylabel('Response (m/s)')
xlabel('Time (s)')

figure('Name','Actual Velocity Profile','NumberTitle','off','Color','white')
plot(t,ds(:,4),'k'), grid on
title('Actual Velocity Profile')
legend('Actual Velocity')
ylabel('Speed (m/s)')
xlabel('Time (s)')
