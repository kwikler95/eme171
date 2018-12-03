clc
clear all
close all

%% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb im p_3in p_9in Kp Ki

Fb= 25000; %Friction value 

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
n = 0.0001; 
vi = 20; %m/s initial vehicle speed
im = 0; %initial currrent in recharging circuit
k = (Tm^2)/Lw; %stiffness, simplification of I/GY to C
Kp = 200; %proportional gain
Ki = 30; %integral gain
vref=1;

%p_3in = Lw*im; %initial inductor momentum
e11_in = 0.5*rho*Af*Cd*20*20;
p_3in = (e11_in*R/Gr + bt*(Gr/R)*(20))*(Lw/Tm);
p_9in = M*vi; %initial car momentum
uinss = im*Rw+Tm*(Gr/R)*20;

%% Run ODE45 Solver 

% define initial conditions
%initial = [p_3in p_9in]; 
initial = [0 0 0 0];

% setup time array
%tspan = [0:.025:3]; %defined in lab handout
tspan = [0:0.01:2];

% call ODE solver
[t, s] = ode45(@lab5_eqns,tspan,initial);

% pre-allocate additional output vectors
ext = zeros(length(t),4);
ds = zeros(length(t),4);

% obtain actual function outputs
for i = 1:length(t)    
    [ds(i,:) ext(i,:)] = lab5_eqns(t(i), s(i,:));
end

%% current and nat frequency

current=ext(:,1)/Lw;

%% Plot Figures

figure('Name','velocity','NumberTitle','off','Color','white')
plot(t,s(:,2)/M,'k'), grid on
title('Vehicle Speed vs Time')
legend('Vehicle Speed')
ylabel('Speed (m/s)')
xlabel('Time (s)')

figure('Name','current','NumberTitle','off','Color','white')
plot(t,current,'k'), grid on
title('Current vs Time')
legend('Current')
ylabel('Current (Amps)')
xlabel('Time (s)')
 
figure('Name','Step Response','NumberTitle','off','Color','white')
plot(t,ds(:,3),'--k',t,-s(:,2)/M,'m'), grid on
title('Step Response')
legend('Step Input','Output Response')
ylabel('Response')
xlabel('Time (s)')

min(current)
wn = sqrt(k/Lw)