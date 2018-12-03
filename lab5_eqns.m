% state derivative function
function [ds, ext] = lab5_eqns(t,s);

% input system parameters
global Rw Lw Tm M bt R Gr Cr g Cd rho Af n vi uinss Fb im p_in Kp Ki

% renaming variables for clarity
p_3 = s(1); %momentum to inductor
p_9 = s(2); %momentum to car mass
d_ref = s(3);
d = s(4);

% define u_in and Fb time
T1=0.5;
T2=2;

%defining uin and Fb and v
if t <= T1;
	u_in = uinss; %initial voltage
    fb=0;
    vref = 0;
elseif t >= T1 && t <= T2;
	%u_in = uinss.*(1-(2/3).*(t-0.5)); %ramp down
    u_in = Kp*(1-(p_9/M)) + Ki*(d_ref - d);
    fb=Fb;
    vref = 1;
else  t > T2;
	u_in = 0; %zero voltage
    fb=0; % zero voltage
    vref = 1; 
end


% friction brake input

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