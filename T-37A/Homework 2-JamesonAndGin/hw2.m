m = 0;
g = -9.81;
Ftx = 0;
Fax = 0;
Fty = 0;
Fay = 0;
Ftz = 0;
Faz = 0;
Ft = [Ftx; Fty; Ftz];
Fa = [Fax; Fay; Faz]; 
Forces = Ft+Fa; 

theta = 0;
psi = 0;
phi = 0;
P=1;
Q=1;
W=1;
R=1;
V=1;
U=1;

u_dot=0;
v_dot=0;
w_dot=0;

velocity = 100;
t = 10;

%modelname = 'T37_SimulinkModel';
%sim(modelname)

%moments
Mx = 0;
My = 0;
Mz = 0;

M = [Mx; My; Mz];

qr = 0;
qi=0;
qj=0;
qk=0;



modelname = 'T37_SimulinkModelhw2.slx';
sim(modelname)

% Homework 2
