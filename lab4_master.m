clc
clear all
close all

%% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb im p_3in p_9in 

Fb= 1000; %Friction value -N-!!!!! WhOOMP

Rw = 0.3; %Ohms
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
n = 0.001; 
vi = 20; %m/s initial vehicle speed
im = 0; %initial currrent in recharging circuit

p_3in = Lw*im; %initial inductor momentum
p_9in = M*vi; %initial car momentum
uinss = im*Rw+Tm*Gr/R*vi;


%% Run ODE45 Solver 

% define initial conditions
initial = [p_3in p_9in]; 

% setup time array
tspan = [0:.025:3]; %defined in lab handout

% call ODE solver
[t, s] = ode45(@lab4_eqns,tspan,initial);

% pre-allocate additional output vectors
ext = zeros(length(t),2);
ds = zeros(length(t),2);

% obtain actual function outputs
for i = 1:length(t)    
    [ds(i,:) ext(i,:)] = lab4_eqns(t(i), s(i,:));
end
%ext[p_3, p_9]

%% current

current=-ext(:,1)/Lw;

%% Plot Figures

figure('Name','velocity','NumberTitle','off','Color','white')
plot(t,ext(:,2)/M), grid on
title('Suspension Displacement, Forward Configuration')
legend('Front Wheel','Back Wheel')
ylabel('Amplitude (m)')
xlabel('time (s)')


figure('Name','current','NumberTitle','off','Color','white')
plot(t,current,'k'), grid on
title('Pitch Angular Velocity, Forward Configuration')
legend('Pitch angular velocity')
ylabel('Angular velocity (rad/s)')
xlabel('time (s)')
% 
% figure('Name','Heave Velocity','NumberTitle','off','Color','white')
% plot(t,s(:,2)./mcr,'k'), grid on
% title('Heave Velocity, Forward Configuration')
% legend('Heave velocity')
% ylabel('Heave velocity (m/s)')
% xlabel('time (s)')


