m = 0;
g = -9.81;
Ftx = 0;
Fax = 0;
Fty = 0;
Fay = 0;
Ftz = 0;
Faz = 0;
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

modelname = 'T37_SimulinkModel';
sim(modelname)

