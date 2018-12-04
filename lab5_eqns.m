% state derivative function
function [ds, ext] = lab5_eqns(t,s);

% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb im p_in Kp Ki

% renaming variables for clarity
p_3 = s(1); 
p_9 = s(2); 
d_ref = s(3);
d = s(4);

% define u_in and Fb time
T1 = 0.1;

if t >= T1;
    %vref = 1;
    vref = LA92Oracle(t);
else
    vref = 0;
end

u_in = Kp*(vref-(p_9/M)) + Ki*(d_ref - d);

% non linear efforts
v=p_9/M; %car velocity
sign_v = v/(abs(v)+n);
e11 = (0.5/(M^2))*rho*Af*Cd*p_9*abs(p_9);
e12 = M*g*Cr*sign_v;

% eqns of motion 
p_dot3 = u_in - (p_3/Lw)*Rw - Tm*(Gr/R)*(p_9/M);
p_dot9 = (Gr/R)*(-bt*(Gr/R)*(p_9/M) + Tm*(p_3/Lw)) - e12 - e11;
d_ref_dot = vref;
d_dot = p_9/M;

% defining extra variables for output
ext(1) = p_3; % 
ext(2) = p_9; % 
ext(3) = d_ref;
ext(4) = d;

% stacking up derivs
ds = [p_dot3; p_dot9; d_ref_dot; d_dot]; 