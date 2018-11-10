% state derivative function
function [ds, ext] = lab4_eqns(t,s)

% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi

% renaming variables for clarity
p_3 = s(1);
p_9 = s(2);

% define u_in and v vs time
u_in = linspace(0.5,2,100);
v = linspace();

% friction brake input


% input specs
SE1 = mcr*g;
SE2 = mtr*g;
SE3 = mtf*g;

% non linear efforts
e11 = 0.5*rho*Af*Cd*v^2;
e12 = M*g*Cr*(v/(abs(v)+n));

% eqns of motion 
p_dot3 = u_in - (p_3/Lw)*Rw - Tm*(Gr/R)*(p_9/M);
p_dot9 = (Gr/R)*(-bt*(Gr/R)*(p_9/M) + Tm*(p_3/Lw)) - e12 - e11 - Fb;


% defining extra variables for output
ext(1) = vfi; % front wheel input velocity
ext(2) = vri; % rear wheel input velocity

% stacking up derivs
ds = [p_dotj; p_dotcr; q_dotsf; q_dotsr; p_dottf; p_dottr; q_dottf; q_dottr];



