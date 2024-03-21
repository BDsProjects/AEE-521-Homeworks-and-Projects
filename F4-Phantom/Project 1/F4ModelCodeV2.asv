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
LambdaLE_rad = deg2rad(LambdaLE_deg);
SP_V_ft2 = 14.7;
LambdaLEH_deg = 43.0;
wmax_ft = 8.9;
LambdaLEV_deg = 63;
LambdaLEV_rad = deg2rad(LambdaLEV_deg);
XACR_ft = 15.1;
eta_H = 0.85;
eta_V = 0.9;
tau_E = 0.5;
tau_A = 0.44;
e = .95;
K_q = 0.7;

x_AC_H = 23.6;
xbar_AC_H = 1.479;
t = horzcat(0.0, 0.1, 70.0);

IXX = 25000; %slug/ft^2
IYY = 122200; %slug/ft^2
IZZ = 139800; %slug/ft^2
IXZ = 2200; %slug/ft^2 Might be different values

q = 0; %pitching rate

alt = 35000;
Mach = 0.9;
a=968;
qbar1 = 283.2;
alpha1 = 2.6;
Length = 63.75; %(ft)
Height = 16.5;
xbar_CG = 0.29;
qbar = 283.2;
e=.95;
%C_D_alpha should be .2322
Weight = 39000; %lbs
c_L_1 = .26;
c_D_1 = .030;
c_m_1 = 0;
c_T_X1 = .030;
c_m_T1 = 0;
alpha_rad = deg2rad(alpha1);
mass = Weight/32.2;
uv = Mach*a;

alphadot = 0;

InitialInputs = [qbar1 S_ft2 c_D_1 c_L_1 cbar_ft c_m_1 uv b_ft alphadot];
% deltaA = 0;
% deltaR = 0;
% deltaS = 0;
% 
% CS = [deltaA deltaS deltaR];

% Initial Conditions
u_dot = 0;
v_dot = 0;
w_dot = 0;
phidot = 0;
thetadot= 0;
psidot = 0;
u=0;
v=0;
w=0;


LT = 0;
NT=0;
MT=0;
lT = 0;
mT = 0;
nT = 0;


q0 = angle2quat(phidot,thetadot,psidot);
%% Hw3
lambda = cT_ft/cr_ft;
S_ft2;
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

lambdaH = cTH_ft/crH_ft;
S_H = (bH_ft/2)*(crH_ft)*(1+lambdaH);
ARH = (bH_ft^2)/S_H;
cbarH = .66667*crH_ft*(((1+lambdaH+lambdaH^2))/(1+lambdaH));

LambdaLEH_half_deg = tand(LambdaLEH_deg) - (((4*.5)*(1-lambdaH))/(ARH*(1+lambdaH)));
LambdaLEH_quarter_deg = tand(LambdaLEH_deg)  - (((4*.25)*(1-lambdaH))/(ARH*(1+lambdaH))); 

LambdaLEH_half_rad = atand(LambdaLEH_half_deg) * (pi/180);
LambdaLEH_quarter_rad = atand(LambdaLEH_quarter_deg) * (pi/180);

LambdaLEH_rad = deg2rad(LambdaLEH_deg);

xMacH = ((bH_ft/6)*((1+2*lambdaH)/(1+lambdaH)))*tand(LambdaLEH_deg); 
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
K2 = .3940; %closest value
xbar_AC_W = K1 * (xprime_AC_W_over_cR-K2); %Done
xbar_AC_W2 =  ((tan(LambdaLE_rad))/(sqrt(1-Mach^2))); 
xbar_AC_W3 =  AR*tan(LambdaLE_rad);  

Delta_x_i = [4.5 4.5 4.5 4.5 4.5 9 9 3.2 3.2 3.2 3.2 3.2 3.2];
w_i = [2.2 3.7 4.3 6.5 8.9 8.3 7.9 7.6 7 3.7 2.7 1.8 1];
depsilon_dalpha_i = [1.16 1.22 1.3 1.47 3.2 0.16 0.05 0.02 0.06 0.09 0.13 0.17 0.2];
sum = 0;


for i=1:length(Delta_x_i)
    sum = sum + ((w_i(i)^2)*Delta_x_i(i)*depsilon_dalpha_i(i));
end

Delta_xbar_AC_B = - (1/(2.92*S_ft2*cbar_ft)) * sum;

xbar_AC_WB = Delta_xbar_AC_B + xbar_AC_W;

%Aerodynamic Parameters
c_L_alpha_W = c_L_alpha_W_mach;
c_L_alpha_H = c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(1-d_epsilon_dalpha_mach); %Done to here
c_L_alpha = c_L_alpha_W_mach + c_L_alpha_H; %G

c_L_delta_E = 0; %G

c_L_i_H = c_L_alpha_H_mach*eta_H*(S_H/S_ft2); %G

c_m_alpha = (c_L_alpha_W_mach*(xbar_CG - xbar_AC_WB)) - c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(1-d_epsilon_dalpha_mach)*(xbar_AC_H - xbar_CG); %G

c_m_i_H = -c_L_alpha_H_mach*eta_H*(S_H/S_ft2) * (xbar_AC_H - xbar_CG); %G

c_m_delta_E = c_m_i_H * tau_E; %G

c_D_alpha = ((2*c_L_1)/(pi*AR*e))*c_L_alpha; %G

c_L_alphadot = 2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*(xbar_AC_H - xbar_CG)*(d_epsilon_dalpha_mach); %G

piece1 = (AR+(2*cos(LambdaLE_quarter_rad)))/(AR*(sqrt((1-Mach^2)*cos(LambdaLE_quarter_rad)^2))+(2*cos(LambdaLE_quarter_rad)));
piece2 = ((0.5+2*abs(xbar_AC_W-xbar_CG))*c_L_alpha_W_mach0);
piece3 = 2*c_L_alpha_H*eta_H*((S_H/S_ft2)*(xbar_AC_H-xbar_CG));
c_L_q = piece1+piece2+piece3; %G


c_m_alphadot = -2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*((xbar_AC_H - xbar_CG)^2)*d_epsilon_dalpha_mach; %G 

lefttopfrac = ((AR^3) * (tan(LambdaLE_quarter_rad)^2))/(AR+(6*cos(LambdaLE_quarter_rad)));
righttopfrac = 3/(sqrt((1-Mach^2)*(cos(LambdaLE_quarter_rad)^2)));
bottomfrac = (((AR^3) * (tan(LambdaLE_quarter_rad)^2))/(AR+(6*(tan(LambdaLE_quarter_rad)^2)))) +3;

secondpiece = -K_q*c_L_alpha_W_mach0*(cos(LambdaLE_quarter_rad));
thirdpiece = ((AR*(0.5*abs(xbar_AC_W-xbar_CG)+(2*abs(xbar_AC_W-xbar_CG)^2)))/(AR+(2*cos(LambdaLEH_quarter_rad)))) + (((1/24) * ((AR^3)* tan(LambdaLE_quarter_rad)^2)/(AR+(6*cos(LambdaLE_quarter_rad))))) + 1/8;
fourthpiece = -2*c_L_alpha_H_mach*eta_H*(S_H/S_ft2)*((xbar_AC_H-xbar_CG)^2) ;
frac = (lefttopfrac+righttopfrac)/(bottomfrac);
c_m_q = (frac * secondpiece * thirdpiece) + fourthpiece;


Hw3Coeffs = [c_L_alpha c_L_delta_E c_L_i_H c_m_alpha c_m_delta_E c_m_i_H c_L_alphadot c_L_q c_m_alphadot c_m_q c_D_alpha];
%% Hw3 Answer Key
c_L_alpha = 3.75;
c_L_delta_E =0;
c_L_i_H =.4;
c_m_alpha =-.4;
c_m_delta_E =0;
c_m_i_H =-.580;
c_L_alphadot = .86; 
c_L_q =1.8;
c_m_alphadot =-1.3;
c_m_q =-2.7;
c_D_alpha = .3;
Hw3Coeffs = [c_L_alpha c_L_delta_E c_L_i_H c_m_alpha c_m_delta_E c_m_i_H c_L_alphadot c_L_q c_m_alphadot c_m_q c_D_alpha];
%% Hw4 Answer Key

c_Y_beta
c_Y_delta_A
c_Y_delta_R
c_n_beta
c_n_delta_A
c_n_delta_R
c_l_beta = 
c_l_delta_A = .0420;
c_l_delta_R = .0060;
c_l_p = -.240;
c_Y_p = 0;
c_n_p = -.036;
c_l_r = .070;
c_Y_r = 0;
c_n_r = -.27;
Hw4Coeffs = [c_Y_beta c_Y_delta_A c_Y_delta_R c_n_beta c_n_delta_A c_n_delta_R c_l_beta c_l_delta_A c_l_delta_R c_l_p c_Y_p c_n_p c_l_r c_Y_r c_n_r];
%% Hw4
lambdaV = cTV_ft/crV_ft;
S_V = (bV_ft/2)*crV_ft*(1+(cTV_ft/crV_ft));
AR_V = (bV_ft^2/S_V);
LambdaV = tan(LambdaLEV_rad) - ((4*(1-lambdaV))/(AR_V*(1+lambdaV)));
LambdaLEV_rad = deg2rad(LambdaLEV_deg);
LambdaLEV_rad_half= tan(LambdaLEV_rad) - (((4*.5)*(1-lambdaV))/(AR_V*(1+lambdaV))); %should be 0.25
tan(LambdaLEV_rad);
LambdaLEV_rad_quarter = tan(LambdaLEV_rad_half) - (((4*.25)*(1-lambdaV))/(AR_V*(1+lambdaV)));
X_AC_H_to_V = 2.6 ;% need to verify
cbarV =  (2/3 * crV_ft*((1+lambdaV+lambdaV^2))/(1+lambdaV));
b2V_ft = (2*bV_ft);
S2V = (b2V_ft/2)*crV_ft*(1+lambdaV);
AR2_V = (b2V_ft^2)/S2V;
beta1 = sqrt(1-Mach^2);


K_Y_V = .76;
c1_x_axis = b2V_ft/(2*r1_ft);
c1 = 1.1;
c2_x_axis = ZH_ft/b2V_ft;

xMACV = (b2V_ft/6) * ((1+(2*lambdaV))/(1+lambdaV)) * tan(LambdaLEV_rad) ;
yMACV = (b2V_ft/6) * ((1+(2*lambdaV))/(1+lambdaV)) ;%verify with group mates 
c2line = X_AC_H_to_V/cbarV;
c2 = 0.9;
S_H/S_V;
K_HV = 1;
AR_V_eff = c1 * AR2_V * (1+K_HV*(c2-1));
c_L_alpha_V = (2*pi*AR_V_eff)/(2 + sqrt(((AR_V_eff*(1-Mach^2))/(K_Y_V))*(1+((tan(LambdaLEV_rad_half)^2)/(1-Mach^2)))+4));

c_Y_delta_A = 0;
 %- Done
%cbarR = 0; %Needs to be plugged in to forrect
%tau_R_x_axis = cR_ft/cbarR
tau_R = 0.42;
K_R = 0;
K_R_F = .94;
delta_K_R = K_R_F-K_R; %0 - .94
c_Y_delta_R = abs(c_L_alpha_V) *eta_V*((0.5*S2V)/S_ft2)*delta_K_R*tau_R ;% This is correct 
% - Done
c_Y_beta_W = -.0001 * ((GammaW_deg)) * 57.3 ;
K_int = 1.15;
%c_Y_beta_W = -2*(SP_V_ft2/S_ft2)
c_Y_beta_H = -.0001 * abs((GammaH_deg)) * 57.3 * (.724 + 3.06 * ((S_H/S_ft2)/(1+cos(rad2deg(LambdaLE_quarter_rad))))+(.4*(ZW_ft/d_ft))+(.009*ARH)) *(S_H/S_ft2);
c_Y_beta_B = -2*(.96*(SP_V_ft2/S_ft2));
K_HV = 1;
AR_V_eff = c1 * AR2_V * (1+K_HV*(c2-1));
k_V = 1 + ((AR_V_eff*(1.87 - (0.000233*LambdaLEV_rad)))/100);
c_L_alpha_V_eff = (2*pi*AR_V_eff)/(2 + sqrt(((AR_V_eff*(1-Mach^2))/(k_V))*(1+((tan(LambdaLEV_rad_half)^2)/(1-Mach^2)))+4));
disp(rad2deg(LambdaLEV_rad_quarter));

c_Y_beta_V = -K_Y_V * abs(c_L_alpha_V_eff) * (.724 + 3.06 * (S_V/S_ft2)/(1+cos((LambdaLE_quarter_rad)))+(.4*(ZW_ft/d_ft))+(.009*AR)) *(S2V/S_ft2);
c_Y_beta = c_Y_beta_W + c_Y_beta_B + c_Y_beta_H  + c_Y_beta_V; %abt double what it needs to be
 %- Done
c_n_beta_W = 0; %negligible for all configs

lcg_ft/lb_ft;
(lb_ft^2)/SBS_ft;
Z1 = 6.8; % Marked in F4 Document, Also in figure 4.47 in lecture 8-1
Z2 = 6.9; % Marked in F4 Document, Also in figure 4.47 in lecture 8-1
sqrt(Z1/Z2);
zmax_ft/wmax_ft ;%f4 document
K_N = .001 ;%figure 4.68 
a = 968;
mu = 4.058*10^-4;
RE_fuselage = (Mach*a*lb_ft)/(mu);
K_Re_l =  2 ;
c_n_beta_B = -57.3*K_N*K_Re_l *(SBS_ft/S_ft2)*(lb_ft/b_ft);
X_V = 13.75; 
Z_V = 7.664;
c_n_beta_V = -c_Y_beta_V * (((X_V*cos((alpha_rad)))+(Z_V*sin((alpha_rad))))/b_ft);
c_n_beta_H = -c_Y_beta_H * (((x_AC_H*cos((alpha_rad)))+(ZH_ft*sin((alpha_rad))))/b_ft);
c_n_beta = c_n_beta_W+c_n_beta_B+c_n_beta_V+c_n_beta_H;
%c_n_beta = .04567; %should be .04567
 %- Done
RME_I = 0.07;
RME_F = .175;
%delta_RME = RME_F-RME_I
delta_RME = .105;
c_l_delta_prime = 0; %need to add in 
cbar_Aileron_ft/cWing_at_Aileron_ft;
eta_I = yAI_ft/(b_ft/2); %verified
eta_O = yAO_ft/(b_ft/2); %verified
Beta_side_slip_angle = sqrt(1-Mach^2);
rad2deg(Beta_side_slip_angle);
(Beta_side_slip_angle*AR)/(K_AR);
LambdaBeta = 64.72;
delta_K_n_A = .28-.27;

LambdaLE_half_deg = tand(LambdaLE_deg) - ((4*.5 *(1-lambda))/(AR*1+lambda));
LambdaLE_half_rad = atand(LambdaLE_half_deg) * (pi/180);
k = 1 + ((AR*(1.87 - (0.000233*LambdaLE_rad)))/100);

c_L_alpha_Wing_mach = (2*pi*AR)/(2+sqrt((((AR^2*(1-Mach^2))/(k^2))*(1+((tan(LambdaLE_half_rad)^2)/(1-Mach^2))))+4));
k_RME = (c_L_alpha_Wing_mach * beta1) /(2*pi);
c_prime_l_delta = delta_RME*k_RME/beta1;
c_l_delta_A = c_prime_l_delta * tau_A;


%- Done
RME_I = 0.07;
RME_F = .175;
delta_RME = RME_F-RME_I;
c_l_delta_prime = 0; %need to add in 
cbar_Aileron_ft/cWing_at_Aileron_ft;
eta_I = yAI_ft/(b_ft/2) ;%verified
eta_O = yAO_ft/(b_ft/2); %verified
Beta_side_slip_angle = sqrt(1-Mach^2);
rad2deg(Beta_side_slip_angle);
(Beta_side_slip_angle*AR)/(K_AR);
LambdaBeta = 64.72;
delta_K_n_A = .28-.27;
%very small 6.77E-5
c_n_delta_A = delta_K_n_A * c_L_1 * c_l_delta_A;
%- Done 
X_R = 27; % verified
Z_R = 7.4;
c_l_delta_R = (c_Y_delta_R/2) * (((Z_R*cos(alpha_rad)) - (X_R*sin(alpha_rad)))/b_ft);
 %- Done
c_n_delta_R = (-c_Y_delta_R) * (((X_R*cos(alpha_rad))+(Z_R*sin(alpha_rad)))/b_ft);
% - Done
beta1 = sqrt(1-Mach^2);
disp(lambda)
k = (c_L_alpha_Wing_mach*beta1)/(2*pi);
(beta1*AR)/k ;%this aint right 
clp_quarterchord_rad = (LambdaLE_quarter_rad);
atand(clp_quarterchord_rad/(beta1))
RDP = -.225;
c_l_p_W = RDP * (k/beta1); %needs to ne -.11
c_l_p_WB = c_l_p_W;
%c_l_p_H
beta1 = sqrt(1-Mach^2);
disp(lambdaH)
LambdaLEH_half_deg = tand(LambdaLEH_deg) - ((4*.5 *(1-lambdaH))/(ARH*1+lambdaH));
LambdaLEH_half_rad = atand(LambdaLEH_half_deg) * (pi/180);
LambdaLEH_rad = LambdaLEH_deg*(pi/180);
k_H = 1 + ((8.2-(2.3*LambdaLEH_rad))-(ARH*(0.22-(.153*LambdaLEH_rad)))/100);
c_L_alpha_H_mach = (2 * pi * ARH) / (2 + sqrt((((ARH^2 * (1 - Mach^2)) / (k_H^2)) + ((1+(tan(LambdaLEH_half_rad)^2)/(1-Mach^2)))+ 4)));
%c_L_alpha_H_mach = (2*pi*ARH)/(2+sqrt((((ARH^2*(1-Mach^2))/(k_H^2))*(1+((tan(LambdaLEH_half_rad)^2)/(1-Mach^2))))+4))
kH = (c_L_alpha_H_mach*beta1)/(2*pi);
(beta1*ARH)/kH ;%this aint right 
clp_quarterchord_radH = (LambdaLEH_quarter_rad);
atand(clp_quarterchord_radH/(beta1))
RDPH = -.25;
c_l_p_H = RDPH *(kH/beta1)*0.5 * (c_l_p_W)*(S_H/S_ft2)*((bH_ft/b_ft)^2) ;
%c_l_p_V
c_l_p_v = 2*c_Y_beta_V * ((Z_V/b_ft)^2);
c_l_p = c_l_p_W + c_l_p_H + c_l_p_v; % Need to verify
% - Done
c_Y_p = 2 * c_Y_beta_V * (((Z_V*cos(alpha_rad))-(X_V*sin(alpha_rad)))/b_ft);
 %- Done
delta_c_n_p_over_epsilon_W = -.0002;
B = sqrt(1-((Mach^2)*cos(LambdaLE_quarter_rad)^2));
C = (AR+(4*cos(LambdaLE_quarter_rad)))/(AR*B+(4*cos(LambdaLE_quarter_rad)));
C = .9223; 
cnpclpmach0 = -.1892;
c_n_p_W = cnpclpmach0 * c_L_1 + (delta_c_n_p_over_epsilon_W * epsilonW_deg_assumed);

c_n_p_V = -2 *c_Y_beta_V *  ((X_V*cos(alpha_rad)+(Z_V*sin((alpha_rad))))/b_ft) * (((Z_V*cos(alpha_rad))-(X_V*sin(alpha_rad))-Z_V)/b_ft);
%c_n_p_V = -2*c_Y_beta_V*((X_V*cos(alpha_1) + Z_V*sin(alpha_1))/b)*((Z_V*cos(alpha_1)-X_V*sin(alpha_1)-Z_V)/b)
c_n_p = c_n_p_V + c_n_p_W;

 %- Done
B = sqrt((1-Mach^2)*(cos(LambdaLE_quarter_rad)^2));
D = (1 + AR*(1 - B^2)/(2*B * (AR*B + 2*cos(LambdaLE_quarter_rad))) + (AR*B + 2*cos(LambdaLE_quarter_rad/4))/(AR*B + 4*cos(LambdaLE_quarter_rad/4)) * tan(LambdaLE_quarter_rad/4)^2 / 8) / (1 + (AR+2*cos(LambdaLE_quarter_rad))/(AR+4*cos(LambdaLE_quarter_rad/4)) * tan(LambdaLE_quarter_rad/4)^2 / 8);
delta_clr_epsilonW = .0085;
c_l_r_V = -2 *c_Y_beta_V *  (((X_V*cos((alpha_rad)))+(Z_V*sin((alpha_rad))))/b_ft) * (((Z_V*cos(alpha_rad))-(X_V*sin(alpha_rad)))/b_ft) ;

clr_cl1_mach_cl0 = .32;
delta_clr_over_gamma = (1/12)*((pi*AR*sin(LambdaLE_quarter_rad))/(AR+4 *cos(LambdaLE_quarter_rad)));
c_l_r_W = (clr_cl1_mach_cl0 * c_L_1) + delta_clr_over_gamma * GammaW_rad +delta_clr_epsilonW * epsilonW_deg_assumed;

c_l_r = c_l_r_W + c_l_r_V;
%c_l_r = .1643;

 %- Done
c_Y_r = -2 * c_Y_beta_V *((((X_V*cos((alpha_rad)))+(Z_V*sin((alpha_rad))))/b_ft));
 %- Done with modification
cnrcl1_squared = -.02;
%cnr_cD0 = 
c_n_r_W = cnrcl1_squared *(c_L_1^2);
c_n_r_V = 2* (c_Y_beta_V-.05) *(((X_V*cos(alpha_rad))+Z_V*sin(alpha_rad))/(b_ft))^2;
c_n_r = c_n_r_V+c_n_r_W ;%should be -.1128
 %- Done with modification


(rad2deg(LambdaLE_half_rad));
c_l_beta_over_c_L_1 = -.0028 ;





disp(lambda)
dB = sqrt(SfAVG_ft2/(pi/4)); % diameter of the fuselage
delta_clbeta_over_Gamma_W = -.0005 * AR * ((dB/b_ft)^2); %slide 30 (1/deg^2)

delta_clbeta_ZW = ((1.2*sqrt(AR)));
tand(rad2deg(LambdaLE_half_rad)) 



Mach*(cosd((LambdaLE_half_rad)));
AR/(cos((LambdaLE_half_rad)));
K_M_Lambda = 1.1;

A_ft/b_ft;
AR/(cos((LambdaLE_half_rad)));
K_f = .94;
c_l_beta_c_L_1_LE_half = -.00094;

c_l_beta_over_c_L_1_AR = (-2*10^-3); % 1/deg
rad2deg((cos((LambdaLE_half_rad))))
c_l_beta_over_Gamma = -1.25; %1/deg^2

clbeta_cl1_Lambda_c__2 = -2.4*10^-3; % 1/deg 7/24 lec 9, not sure what lambda_c__2 =
clbeta_cl1_AR = -2*10^-3; % 1/deg

K_M_Lambda = 1.05; 
K_f = 1.125; 

clbeta_GW = -1.2*10^-4; % 1/degrees^2   
K_M_Gamma = 1.07;

S_f_avg = 24.6; %ft^2


d_B = sqrt(S_f_avg/(pi/4));

delta_clbeta_GW = -0.0005*AR*((d_B/b_ft)^2); %1/deg^2
delta_clbeta_Z_W = ((1.2*sqrt(AR))/57.3)*(ZW_ft/b_ft)*((2*d_B)/b_ft);

corr_factor = -2*(10^-5); %1/deg^2
ep_W = 2*pi/180; % rads

Gamma_W = 4;
block_5_1 = clbeta_cl1_Lambda_c__2*K_M_Lambda*K_f + clbeta_cl1_AR;
block_5_2 = Gamma_W*(clbeta_GW*K_M_Gamma + delta_clbeta_GW) + delta_clbeta_Z_W;
block_5_3 = ep_W*tan(LambdaLEV_rad_quarter)*corr_factor;

c_l_beta_WB = 57.3*c_L_1*(block_5_1) + 57.3*(block_5_2 + block_5_3);

Mach*(cosd((LambdaLE_half_rad)));

K_M_Gamma = 1.1;
epsilonWingTwist = -2.6; % 1/deg^2
k_YV = .761;
delta_clbeta_over_epsilonW = -2*10^-5;





c_l_beta_V = c_Y_beta_V * (((Z_V*cos(alpha_rad))-X_V*sin(alpha_rad))/b_ft);
c_l_beta_H = (eta_H*(S_H/S_ft2)*(bH_ft/b_ft)) - (.01); %need to multiply this by c_l_beta_WB
c_l_beta = c_l_beta_H+c_l_beta_V+c_l_beta_WB;

Hw4Coeffs = [c_Y_beta c_Y_delta_A c_Y_delta_R c_n_beta c_n_delta_A c_n_delta_R c_l_beta c_l_delta_A c_l_delta_R c_l_p c_Y_p c_n_p c_l_r c_Y_r c_n_r];
%c_m_q = 
%% u Terms
c_L_u = ((Mach^2)/(1-Mach^2))*c_L_1;
c_D_u = 0;
c_m_u = 0;


uCoeffs = [c_L_u c_D_u c_m_u];

%% Quaternion Block Initialization
Xe = 0;
Ye = 0;
Ze = 35000;
InitialInertialPosition = [Xe Ye Ze];
% 
% Xe1 = 0;
% Ye1 = 0;
% Ze1 = 0;
% SecondInertialPosition = [Xe1 Ye1 Ze1];

U = uv;
v = 0;
w = 0;
InitialVelocity = [U v w];

% U1 = 0;
% v1 = 0;
% w1 = 0;
% SecondVelocity = [U1 v1 w1];

phi = 0;
theta = 0;
psi = 0;
InitialEulerOrientation = [phi theta psi];

p = 0;
q = 0;
r = 0;
InitialRotationRates = [p q r];

% p1 = 0;
% q1 = 0;
% r1 = 0;
% SecondRotationRates = [p1 q1 r1];

qr = 1.0;
qi = 0;
qj = 0;
qk = 0;
quat = [qr qi qj qk]';

% qr1 = 0;
% qi1 = 0;
% qj1 = 0;
% qk1 = 0;
% quat1 = [qr1 qi1 qj1 qk1];
IXX = 25000; %slug/ft^2
IYY = 122200; %slug/ft^2
IZZ = 139800; %slug/ft^2
IXZ = 2200; %slug/ft^2 Might be different values

I = [IXX 0 -IXZ; 
     0   IYY 0; 
     0 0 IZZ];

%Quaternion States

%% Assumptions
P1 = 0;
Q1= 0;
R1 = 0;

V1 = 0;

fTx = 0;
fTy = 0;
fTz = 0;
FTx = 0;
FTy = 0;
FTz = 0;

%% Unsure abt these
c_D_delta_E = 0;
c_D_q = 0;
c_D_i_H = -.10;
V_P_1 = 875.6;
c_Y_1 = 0;
c_Y_betadot = 0;
c_l_betadot = 0;
c_n_1 = 0;
c_n_betadot = 0;

uk = [c_D_delta_E c_D_q c_D_i_H c_Y_1 c_Y_betadot c_l_betadot c_n_1 c_n_betadot];



%% Input Generation

U1 = V_P_1 * cos(alpha_rad);
W1 = V_P_1 * sin(alpha_rad);
V1 = 0;
% %% 
% 
% 
% t_final = 60.0;
% Ts = 0.001;
% 
% timeVec = [0:Ts:t_final]';
% 
% deltaA = zeros(length(timeVec),1);
% % deltaS = zeros(length(timeVec),1);
% deltaR = zeros(length(timeVec),1);
% i_H = zeros(length(timeVec),1);
% 
% deltaA_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
% deltaA_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
% doubletMag = 2;
% deltaA(deltaA_doublet_up_Idx,1) = deg2rad(doubletMag);
% deltaA(deltaA_doublet_down_Idx,1) = deg2rad(-doubletMag);
% deltaR_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
% deltaR_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
% doubletMag = 2;
% deltaR(deltaR_doublet_up_Idx,1) = deg2rad(doubletMag);
% deltaR(deltaR_doublet_down_Idx,1) = deg2rad(-doubletMag);
% % deltaS = zeros(length(timeVec),1);
% % deltaS_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
% % deltaS_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
% % doubletMag = 2;
% % deltaS(deltaS_doublet_up_Idx,1) = deg2rad(doubletMag);
% % deltaS(deltaS_doublet_down_Idx,1) = deg2rad(-doubletMag);
% % deltaS_vector = horzcat(timeVec,deltaS);
% 
% i_H_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
% i_H_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
% doubletMag = 2;
% i_H(i_H_doublet_up_Idx,1) = deg2rad(doubletMag);
% i_H(i_H_doublet_down_Idx,1) = deg2rad(-doubletMag);
% 
% i_H_vector = horzcat(timeVec,i_H);
% deltaA_vector = horzcat(timeVec,deltaA);
% deltaR_vector = horzcat(timeVec,deltaR);
% 
% % CSc = [deltaA deltaS deltaR];
% % CSv = [deltaA_vector deltaS_vector deltaR_vector];
%% Aileron Movement
tf = 20.0;
Ts = 0.01;

timeVec = [0:Ts:tf]';

deltaA = zeros(length(timeVec),1);
% deltaS = zeros(length(timeVec),1);
deltaR = zeros(length(timeVec),1);
i_H = zeros(length(timeVec),1);

deltaA_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
deltaA_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
doubletMag = 2;
deltaA(deltaA_doublet_up_Idx,1) = deg2rad(doubletMag);
deltaA(deltaA_doublet_down_Idx,1) = deg2rad(-doubletMag);

i_H_vector = horzcat(timeVec,i_H);
deltaA_vector = horzcat(timeVec,deltaA);
deltaR_vector = horzcat(timeVec,deltaR);
%% Rudder Movement
tf = 20.0;
Ts = 0.01;

timeVec = [0:Ts:tf]';

deltaA = zeros(length(timeVec),1);
% deltaS = zeros(length(timeVec),1);
deltaR = zeros(length(timeVec),1);
i_H = zeros(length(timeVec),1);

deltaR_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
deltaR_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
doubletMag = 2;
deltaR(deltaR_doublet_up_Idx,1) = deg2rad(doubletMag);
deltaR(deltaR_doublet_down_Idx,1) = deg2rad(-doubletMag);

i_H_vector = horzcat(timeVec,i_H);
deltaA_vector = horzcat(timeVec,deltaA);
deltaR_vector = horzcat(timeVec,deltaR);


%% Stabilator Movement
tf = 20.0;
Ts = 0.01;

timeVec = [0:Ts:tf]';

deltaA = zeros(length(timeVec),1);
deltaR = zeros(length(timeVec),1);
i_H = zeros(length(timeVec),1);

i_H_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
i_H_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
doubletMag = 2;
i_H(i_H_doublet_up_Idx,1) = deg2rad(doubletMag);
i_H(i_H_doublet_down_Idx,1) = deg2rad(-doubletMag);

i_H_vector = horzcat(timeVec,i_H);
deltaA_vector = horzcat(timeVec,deltaA);
deltaR_vector = horzcat(timeVec,deltaR);


%% 

% WhichCase = input("What casue would you like to run, stable, deltaA, deltaS, or deltaR?", "s");
% for i = 1:4
% 
%     if WhichCase == "stable"
%         deltaA = zeros(length(timeVec),1);
%         deltaS = zeros(length(timeVec),1);
%         deltaR = zeros(length(timeVec),1);
%         i_H = zeros(length(timeVec),1);
%     elseif WhichCase == "deltaA"
% 
%         deltaA = zeros(length(timeVec),1);
%         deltaA_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
%         deltaA_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
%         doubletMag = 2;
%         deltaA(deltaA_doublet_up_Idx,1) = deg2rad(doubletMag);
%         deltaA(deltaA_doublet_down_Idx,1) = deg2rad(-doubletMag);
%         %disp(deltaA)
%         deltaR = zeros(length(timeVec),1);
%         deltaS = zeros(length(timeVec),1);
% 
%     elseif WhichCase == "deltaR"
%         deltaR = zeros(length(timeVec),1);
%         deltaR_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
%         deltaR_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
%         doubletMag = 2;
%         deltaR(deltaR_doublet_up_Idx,1) = deg2rad(doubletMag);
%         deltaR(deltaR_doublet_down_Idx,1) = deg2rad(-doubletMag);
%         deltaA = zeros(length(timeVec),1);
%         deltaS = zeros(length(timeVec),1);
%     elseif WhichCase == "deltaS"
%         deltaS = zeros(length(timeVec),1);
%         deltaS_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
%         deltaS_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
%         doubletMag = 2;
%         deltaS(deltaS_doublet_up_Idx,1) = deg2rad(doubletMag);
%         deltaS(deltaS_doublet_down_Idx,1) = deg2rad(-doubletMag);
%         deltaR = zeros(length(timeVec),1);
%         deltaA = zeros(length(timeVec),1);
%     else
%         print("The incorrect case was used. Please use one of the 4 options listed in the input question.")
% 
%     end 
% end 
% deltaA_vector = horzcat(timeVec,deltaA);
% deltaS_vector = horzcat(timeVec,deltaS);
% deltaR_vector = horzcat(timeVec,deltaR);
% %iH_vector = horzcat(timeVec,delta_iH);
% CSc = [deltaA_vector deltaS_vector deltaR_vector]';
%% Model Simulation


% deltaA_vector = horzcat(timeVec,deltaA);
% deltaS_vector = horzcat(timeVec,deltaS);
% deltaR_vector = horzcat(timeVec,deltaR);
% iH_vector = horzcat(timeVec,delta_iH);
% CSc = [deltaA_vector deltaS_vector deltaR_vector];


modelname = 'F4PhantomRunningModel.slx';
sim(modelname)




%% Rudder Plot Section and Variable Outputs

% beta1Angles = beta1(:,0);

figure
subplot(4,1,1);
plot(timeVec,rad2deg(beta1),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('beta (deg)')
title('Rudder Doublet vs Time')
grid on

subplot(4,1,2); 
plot(timeVec,rad2deg(phi),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('ϕ (deg)')
grid on

subplot(4,1,3);
plot(timeVec,rad2deg(phi),'linewidth',1.5)
xlabel('Time (secs)')
ylabel(' ψ (deg)')
grid on

subplot(4,1,4);
plot(timeVec,rad2deg(deltaR_vector(:,end)),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('del_R (deg)')
grid on


%Need to get exports as values and not the timeseries

%% Aileron Plot Section and Variable Outputs

figure
subplot(4,1,1);
plot(timeVec,(beta1),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('beta (deg)')
title('Rudder Doublet vs Time')
grid on

subplot(4,1,2); 
plot(timeVec,(phi),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('ϕ (deg)')
grid on

subplot(4,1,3);
plot(timeVec,(psi),'linewidth',1.5)
xlabel('Time (secs)')
ylabel(' ψ (deg)')
grid on

subplot(4,1,4);
plot(timeVec,rad2deg(deltaA_vector(:,end)),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('del_R (deg)')
grid on

%% Stabilitator Plot Section and Variable Outputs

figure
subplot(4,1,1);
plot(timeVec,U,'linewidth',1.5)
xlabel('Time (secs)')
ylabel('beta (ft/s)')
title('Rudder Doublet vs Time')
grid on

subplot(4,1,2); 
plot(timeVec,rad2deg(alpha1),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('ϕ (deg)')
grid on

subplot(4,1,3);
plot(timeVec,rad2deg(theta),'linewidth',1.5)
xlabel('Time (secs)')
ylabel(' ψ (deg)')
grid on

subplot(4,1,4);
plot(timeVec,i_H_vector(:,end),'linewidth',1.5)
xlabel('Time (secs)')
ylabel('del_R (deg)')
grid on
%% Output Forces and Moments
disp(FAx)
disp(FAy)
disp(FAz)
disp(LA)
disp(NA)
disp(MA)

