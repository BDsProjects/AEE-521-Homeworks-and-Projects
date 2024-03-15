mass = 0;
alpha1 = 2.6;
g = -32.2;
A_ft = 41.5;
XHV_ft = 8.5;
b_ft = 38.7;
XWHr_ft = 28.0;
bH_ft = 16.4;
X1_ft = 46.4;
bV_ft = 6.2;
yAI_ft = 8.5;
cbar_ft = 16.0;
yAO_ft = 13.5;
cbar_Aileron_ft = 2.9;
yRI_ft = 0.0;
cR_ft = 2.6;
yRF_ft = 5.1;
cWing_at_Aileron_ft = 11.9;
yV_ft = 5.2;
cr_ft = 18.0;
ZRS_ft = 7.4;
crH_ft = 7.6;
z1_ft = 6.8;
crV_ft = 16.4;
z2_ft = 6.9;
cT_ft = 4.7;
ZH_ft = 2.1;
cTH_ft = 2.2;
ZHS_ft = 3.2;
cTV_ft = 3.9;
zmax_ft = 7.0;
d_ft = 5.9;
ZW_ft = 1.9;
lb_ft = 58.9;
ZWHr_ft = 4.0;
lcg_ft = 29.8;
GammaH_deg = -27;
GammaH_rad = deg2rad(GammaH_deg);
r1_ft = 6.2;
GammaW_deg = 4.0;
GammaW_rad = deg2rad(GammaW_deg);
S_ft2 = 530;
epsilonH_deg_assumed = 2;
SBS_ft = 327.5;
epsilonW_deg_assumed = 2;
SfAVG_ft2 = 24.6;
LambdaLE_deg = 48.5;
SP_V_ft2 = 14.7;
LambdaLEH_deg = 43.0;
wmax_ft = 8.9;
LambdaLEV_deg = 63;
LambdaLEV_rad = deg2rad(LambdaLEV_deg);
XACR_ft = 15.1;
eta_H = 0.9;
eta_V = 0.9;
tau_E = 0.5;
alphadot = 0;
Mach = 0.9;

x_AC_H = 23.6;
xbar_AC_H = 1.479;
t = horzcat(0.0, 0.1, 70.0);

c_D_u = 0;
a = 968;
uv = a*Mach; %velocity?
V_P_1 = 0; %what is this 
c_D_i = -0.10; %Given by Krishna
c_D_alphadot = 0;
c_D_q = 0;
c_D_delta_E = 0;
i_H = 0; %needs to be set as an array based of time 
delta_stabilitator = 0;
deltaE = 0;
deltaR = 0;
deltaA = 0;
alpha1_rad = deg2rad(alpha1);
V_P_1 = uv/(cos(alpha1_rad));
u1 = V_P_1 * cosd(alpha1);
q = 0; %pitching rate

alt = 35000;
Mach = 0.9;
qbar1 = 283.2;
alpha1 = 2.6;
Length = 63.75; %(ft)
Height = 16.5;
xbar_CG = 0.29;
qbar1 = 283.2;
e=.95;
%C_D_alpha should be .2322
Weight = 39000; %lbs
c_L_1 = .26;
c_D_1 = .030;
c_m_1 = 0;
c_T_X1 = .030;
c_m_T1 = 0;
mass = Weight/32.2;
c_L_u=((Mach^2)/(1-Mach^2))*c_L_1;

LambdaLE_rad = deg2rad(LambdaLE_deg);

lambda = cT_ft/cr_ft;

%S = ((b_ft/2)*(cr_ft))*(1+lambda)
AR = (b_ft^2)/S_ft2;
xMAC = cbar_ft;


xMAC = ((b_ft/6)*((1+(2*lambda))/((1+lambda))*(tand(LambdaLE_deg))));

LambdaLE_half_deg = tand(LambdaLE_deg) - (((4*.5)*(1-lambda))/(AR*(1+lambda)));
LambdaLE_quarter_deg = tand(LambdaLE_deg)  - (((4*.25)*(1-lambda))/(AR*(1+lambda))); 

LambdaLE_half_rad = atand(LambdaLE_half_deg) * (pi/180);
LambdaLE_quarter_rad = atand(LambdaLE_quarter_deg) * (pi/180);

%Modeling Downwash Coefficients

K_AR = (1/AR)-(1/(1+(AR^1.7)));
K_lambda = (10-(3*lambda))/7;
Z_WH = ZWHr_ft + (crH_ft/4) - (cR_ft/4); %need to change 
m = (2*Z_WH)/b_ft;
X_WH = XWHr_ft + (crH_ft/4) - (cr_ft/4);
r = (2*X_WH)/b_ft;
K_mr = (1-(m/2))/(r^.33);
K_mr = .819;
d_epsilon_dalpha_mach0 = 4.44 * (K_AR*K_lambda*K_mr * (sqrt(cos(LambdaLE_quarter_rad))))^1.19;
kw = 1+((AR*1.87-(.000233*LambdaLE_rad))/100);
c_L_alpha_W_mach0 = (2*pi*AR)/(2+sqrt(((((AR^2)*(1-0^2))/(kw^2))*(1+((tan(LambdaLE_half_rad)^2)/(1-0^2))))+4));
c_L_alpha_W_mach = (2*pi*AR)/(2+sqrt(((((AR^2)*(1-Mach^2))/(kw^2))*(1+((tan(LambdaLE_half_rad)^2)/(1-Mach^2))))+4));
d_epsilon_dalpha_mach = d_epsilon_dalpha_mach0 * (c_L_alpha_W_mach/c_L_alpha_W_mach0);            

%Horizontal Tail Parameters

lambdaH = cTH_ft/crH_ft  ;
S_H = (bH_ft/2)*(crH_ft)*(1+lambdaH);
ARH = (bH_ft^2)/S_H;
cbarH = .66667*crH_ft*(((1+lambdaH+lambdaH^2))/(1+lambdaH));

LambdaLEH_half_deg = tand(LambdaLEH_deg) - (((4*.5)*(1-lambdaH))/(ARH*(1+lambdaH)));
LambdaLEH_quarter_deg = tand(LambdaLEH_deg)  - (((4*.25)*(1-lambdaH))/(ARH*(1+lambdaH))); 

LambdaLEH_half_rad = atand(LambdaLEH_half_deg) * (pi/180);
LambdaLEH_quarter_rad = atand(LambdaLEH_quarter_deg) * (pi/180);

LambdaLEH_rad = deg2rad(LambdaLEH_deg);

xMacH = ((bH_ft/6)*((1+2*lambdaH)/(1+lambdaH)))*tand(LambdaLEH_deg) ;
LambdaLEH_half_rad = tan(LambdaLEH_rad) - (((4*.5)*(1-lambdaH))/(ARH*(1+lambdaH)));
LambdaLEH_quarter_rad = tan(LambdaLEH_rad) - (((4*.25)*(1-lambdaH))/(ARH*(1+lambdaH)));

%Wing Tail Geometric Parameters

X_WH = XWHr_ft + (crH_ft/4) - (cr_ft/4);
r = (2*X_WH)/b_ft;
%m = (2*Z_WH)/b_ft
m = .207; %should be this 
x_AC_H = XWHr_ft + xMacH + (cbarH/4) - xMAC;
xbar_AC_H = x_AC_H/cbar_ft;

%Wing Lift-Slope Coefficients - Done

k = 1 + ((AR*(1.87-(.000233*LambdaLE_rad)))/100);
c_L_alpha_W_mach = (2*pi*AR)/(2+sqrt(((((AR^2)*(1-Mach^2))/(kw^2))*(1+((tan(LambdaLE_half_rad)^2)/(1-Mach^2))))+4));

%Horizontal Tail Lift-Slope Coefficients - Done

k_H = 1 + (((8.2-(2.3*LambdaLEH_rad))-(AR*(0.22-(.153*LambdaLEH_rad))))/100);
c_L_alpha_H_mach = (2*pi*ARH)/(2+sqrt(((((ARH^2)*(1-Mach^2))/(k_H^2))*(1+((tan(LambdaLEH_half_rad)^2)/(1-Mach^2))))+4));
c_L_alpha_H_mach0 = (2*pi*ARH)/(2+sqrt(((((ARH^2)*(1-0^2))/(k_H^2))*(1+((tan(LambdaLEH_half_rad)^2)/(1-0^2))))+4));

%Wing Aerodynamic Center

(tan(LambdaLE_rad))/(sqrt(1-Mach^2));
(sqrt(1-Mach^2))/(tan(LambdaLE_rad));
AR*tan(LambdaLE_rad);

xprime_AC_W_over_cR = 0.65;
K1 = 1.42;
K2 = .3940 ;%closest value
xbar_AC_W = K1 * (xprime_AC_W_over_cR-K2); %Done
xbar_AC_W2 =  ((tan(LambdaLE_rad))/(sqrt(1-Mach^2))); % may need this term in front: xbar_AC_W -
xbar_AC_W3 =  AR*tan(LambdaLE_rad); %may need this term in front: xbar_AC_W2 - 

Delta_x_i = [4.5 4.5 4.5 4.5 4.5 9 9 3.2 3.2 3.2 3.2 3.2 3.2]';
w_i = [2.2 3.7 4.3 6.5 8.9 8.3 7.9 7.6 7 3.7 2.7 1.8 1]';
depsilon_dalpha_i = [1.16 1.22 1.3 1.47 3.2 0.16 0.05 0.02 0.06 0.09 0.13 0.17 0.2]';
sum = 0;


for i=1:length(Delta_x_i)
    sum = sum + ((w_i(i)^2)*Delta_x_i(i)*depsilon_dalpha_i(i));
end

Delta_xbar_AC_B = - (1/(2.92*S_ft2*cbar_ft)) * sum;

xbar_AC_WB = Delta_xbar_AC_B + xbar_AC_W;

%Aerodynamic Parameters
c_L_alpha_W = c_L_alpha_W_mach;
c_L_alpha_H = c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(1-d_epsilon_dalpha_mach); %Done to here
c_L_alpha = c_L_alpha_W_mach + c_L_alpha_H ;%G

c_L_delta_E = 0; %G

c_L_i_H = c_L_alpha_H_mach*eta_H*(S_H/S_ft2); %G

c_m_alpha = (c_L_alpha_W_mach*(xbar_CG - xbar_AC_WB)) - c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(1-d_epsilon_dalpha_mach)*(xbar_AC_H - xbar_CG); %G

c_m_i_H = -c_L_alpha_H_mach*eta_H*(S_H/S_ft2) * (xbar_AC_H - xbar_CG); %G

c_m_delta_E = c_m_i_H * tau_E ;%G

c_D_alpha = ((2*c_L_1)/(pi*AR*e))*c_L_alpha; %G

c_L_alphadot = 2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(xbar_AC_H - xbar_CG)*(d_epsilon_dalpha_mach); %G

piece1 = (AR+(2*cos(LambdaLE_quarter_rad)))/(AR*(sqrt((1-Mach^2)*cos(LambdaLE_quarter_rad)^2))+(2*cos(LambdaLE_quarter_rad)));
piece2 = ((0.5+2*abs(xbar_AC_W-xbar_CG))*c_L_alpha_W_mach0);
piece3 = 2*c_L_alpha_H*eta_H*((S_H/S_ft2)*(xbar_AC_H-xbar_CG));
c_L_q = piece1+piece2+piece3 ;%G


c_m_alphadot = -2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*((xbar_AC_H - xbar_CG)^2)*d_epsilon_dalpha_mach; %G 

K_q = 0.7;

lefttopfrac = ((AR^3) * (tan(LambdaLE_quarter_rad)^2))/(AR+(6*cos(LambdaLE_quarter_rad)));
righttopfrac = 3/(sqrt((1-Mach^2)*(cos(LambdaLE_quarter_rad)^2)));
bottomfrac = (((AR^3) * (tan(LambdaLE_quarter_rad)^2))/(AR+(6*(tan(LambdaLE_quarter_rad)^2)))) +3;

secondpiece = -K_q*c_L_alpha_W_mach0*(cos(LambdaLE_quarter_rad));
thirdpiece = ((AR*(0.5*abs(xbar_AC_W-xbar_CG)+(2*abs(xbar_AC_W-xbar_CG)^2)))/(AR+(2*cos(LambdaLEH_quarter_rad)))) + (((1/24) * ((AR^3)* tan(LambdaLE_quarter_rad)^2)/(AR+(6*cos(LambdaLE_quarter_rad))))) + 1/8;
fourthpiece = -2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*((xbar_AC_H-xbar_CG)^2) ;
frac = (lefttopfrac+righttopfrac)/(bottomfrac);
c_m_q = (frac * secondpiece * thirdpiece) + fourthpiece;

%Bare Minimum Test - Aero Forces and Moments
c_D_u = 0;
a = 968;
uv = a*Mach; %velocity?
V_P_1 = 0; %what is this 
c_D_i = -0.10; %Given by Krishna
c_D_alphadot = 0;
c_D_q = 0;
c_D_delta_E = 0;
i_H = 0; %needs to be set as an array based of time 
delta_Stabilitator = 0;
deltaE = 0;
deltaR = 0;
deltaA = 0;
c_D_0 = 0;

c_D_i_H = 0; %Needs to be calculated
c_m_u = 0; %needs to be calculated?


%quaternions
qr = 1;
qi=0;
qj=0;
qk = 0;
q = [qr qi qj qk]';

theta = 0;
psi = 0;
phi = 0;
p=0;
q=0;
W=0;
r=0;
V=0;
U=0;

u_dot=0;
v_dot=0;
w_dot=0;
Xe = 0;
Ye = 0;
Ze = 0;

InitialInertialPosition = [Xe Ye Ze]';
U = 0;
v = 0;
w = 0;

InitialVelocity = [U v w]';

InitialEulerOrientation = [phi theta psi]';

InitialRotationRates = [p q r]';
%load('QuickModelCode.m')
modelname = 'QuickModel.slx';
sim(modelname)