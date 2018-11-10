% state derivative function
function [ds, ext] = lab4_eqns(t,s)

% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb 

% renaming variables for clarity
p_3 = s(1);
p_9 = s(2); 

% define u_in and Fb time
T1=0.5;
T2=2;

% defining uin and Fb
if t < T1
	u_in = uinss; %initial voltage
    fb=0;
elseif t >= T1 && t <= T2
	u_in = uinss.*(1-2/3.*(t-0.5)); %ramp down
    fb=Fb;
else t > T2
	u_in = 0; %zero voltage
    fb=0; % zero voltage
end


% friction brake input


% input specs
% SE1 = mcr*g;
% SE2 = mtr*g;
% SE3 = mtf*g;

% non linear efforts
e11 = 0.5*rho*Af*Cd*v^2;
e12 = M*g*Cr*(v/(abs(v)+n));

% eqns of motion 
p_dot3 = u_in - (p_3/Lw)*Rw - Tm*(Gr/R)*(p_9/M);
p_dot9 = (Gr/R)*(-bt*(Gr/R)*(p_9/M) + Tm*(p_3/Lw)) - e12 - e11 - fb;


% defining extra variables for output
ext(1) = u_in; % front wheel input velocity
ext(2) = vri; % rear wheel input velocity

% stacking up derivs
ds = [p_dotj; p_dotcr; q_dotsf; q_dotsr; p_dottf; p_dottr; q_dottf; q_dottr];



