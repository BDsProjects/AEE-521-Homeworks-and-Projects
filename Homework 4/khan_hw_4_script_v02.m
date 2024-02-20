% Abdul Khan
clear
close all
clc
% Intial Variables
mach = 0.459;
g = 9.81; %m/s
mass = 23000; %kg mass of T-37
theta = 0;
phi = 0;
F_A_X = 0;
F_T_X = 0;
F_A_Y = 0;
F_T_Y = 0;
F_A_Z = 0;
F_T_Z = 0;
U = 0;
V = 0;
W = 0;
Q = 0;
R = 0;
P = 0;

% Initial Conditions
u_dot = 0;
v_dot = 0;
w_dot = 0;

% ---------------------------------------

phidot = 0;
thetadot= 0;
psidot = 0;

q0 = angle2quat(phidot,thetadot,psidot);
%% Plane Description
% T37-A [ Cessna T37-A Fighter Training Jet ]

%% Wing Geometric Parameters

C_T = 4.5; %ft
gamma_LE = deg2rad(1.5); %deg
C_R = 6.2; %ft
b = 33.8; %ft
X_WHR = 15.9; %ft
Z_WH = 3; %ft
b_H = 14; %ft
C_TH = 2.2; %Ft
C_RH = 4.6; %ft
gamma_LEH = deg2rad(12.5); %deg
lambda = C_T/C_R;%unitless
S = (b/2)*C_R*(1+lambda); %ft^2
AR = (b^2)/S; %unitless
c_bar = (2/3)*(C_R)*((1+lambda+lambda+(lambda^2))/(1+lambda));%ft
X_MAC = (b/6)*((1+2*lambda)/(1+lambda))*(tan(gamma_LE));%ft

tan_gamma_x_05 = tan(gamma_LE)-((4*0.5*(1-lambda))/(AR*(1+lambda)));
gamma_05 = atan(tan_gamma_x_05); % this is just arctangent of tan_gamma_x_05
tan_gamma_x_025 = tan(gamma_LE)-((4*0.25*(1-lambda))/(AR*(1+lambda)));
gamma_025 = atan(tan_gamma_x_025);% this is just arctangent of tan_gamma_x_025

%% Horizontal Tail Parameters

lambda_H = C_TH/C_RH; %unitless
S_H = (b_H/2)*(C_RH)*(1+lambda_H); %UNITS
AR_H = (b_H^2)/(S_H); %Unitless
c_bar_H = (2/3)*(C_RH)*((1+lambda_H+(lambda_H^2))/(1+lambda_H));%FT
X_MAC_H = (b_H/6)*((1+2*lambda_H)/(1+lambda_H))*tan(gamma_LEH); 
tan_gamma_x_05H = tan(gamma_LEH)-((4*0.5*(1-lambda))/(AR*(1+lambda)));
gamma_05H = atan(tan_gamma_x_05H); % this is just arctangent of tan_gamma_x_05

%% Wing Tail Geometric Parameters

X_WH = X_WHR+(C_RH/4)-(C_R/4); %ft
r = (2*X_WH)/b;
m = (2*Z_WH)/b;
X_ACH = X_WHR +X_MAC_H+(c_bar_H/4)-X_MAC;
X_bar_ACH = X_ACH/c_bar;

%% Wing Lift Slope Coefficients

K = 1+((8.2-2.3*gamma_LE)-(AR*(0.22-0.153*gamma_LE)))/100; %UNITLESS
CL_alpha_W_mach = (2*pi*AR) / (sqrt(2 + ((AR^2*(1 - mach^2)) / K^2) * (tan(tan_gamma_x_05)^2 / (1 - mach^2)) + 4));%UNITLESS

%% Horizontal Tail Lift Slope Coefficients

K_H = ((8.2-2.3*(deg2rad(gamma_LEH)))-(AR_H*(0.22-0.153*(deg2rad(gamma_LEH)))/100)) +1; %UNITLESS
cl_alpha_H_mach = (2 * pi * AR_H) / (2 + sqrt((AR_H^2 * (1 - mach^2)) / (K_H^2) + (tan(tan_gamma_x_05H)^4) + 4)); %UNITLESS

%% Modeling Donwash Coefficients

K_AR = (1/AR)-(1/(1+(AR^1.7)));  %UNITLESS
K_lambda = (10-3*lambda)/7;  %UNITLESS
K_mr = (1-(m/2))/(r^0.33); %UNITLESS
de_dalpha_at_mach_zero = 4.44*(K_AR*K_lambda*K_mr*(sqrt(cos(gamma_025))^1.19));
CL_alpha_W_at_mach_zero = (2*pi*AR)/(2+sqrt(((AR^2)/(K^2))*(1+tan(gamma_05)^2)+4));
d_episilon_d_alpha_mach = de_dalpha_at_mach_zero*(CL_alpha_W_at_mach_zero/CL_alpha_W_mach);
%% Wing Aerodynamic Center

K1 = 1.16; %Figure 2.28 in lecture 5-1 (T-37A)
K2 = .15; %Figure 2.29 in lecture 5-1 (T-37A)
K1_with_lambda = 1.425;
K2_with_lambda = .65;
x_bar_AC_B = -0.19; %Given from pdf
xprime_AC_W_over_CR = 0.19; %Napolitino
xbar_AC_W = K1_with_lambda*(xprime_AC_W_over_CR-K2_with_lambda);
xbar_AC_W2 = xbar_AC_W-((tan(gamma_LE))/(sqrt(1-mach^2)));
xbar_AC_W3 = xbar_AC_W-AR*tan(gamma_LE);

%Given K1 and K2 values 
xbar_AC_WB = xbar_AC_W+x_bar_AC_B;

%% Aerodynamic Parameters

eta_H = 0.9;
tau_E = 0.5;
Xcg = 0.27;
X_bar_cg_H = Xcg/c_bar_H;
X_bar_cg_W = Xcg/c_bar;
xbarAC = xbar_AC_WB;
CL_alpha_WB = (2*pi*AR)/(2+sqrt((AR^2*(1-mach^2))/(K^2)*(1+((tan(gamma_05)^2)/(1-mach^2)+4))));
CL_alpha_H = (2*pi*AR_H)/(2+sqrt((AR_H^2*(1-mach^2))/(K^2)*(1+((tan(gamma_05)^2)/(1-mach^2)+4))));
de_dalpha_at_mach = 4.44*(K_AR*K_lambda*K_mr*(sqrt(cos(gamma_025))^1.19))*(sqrt(1-mach^2));
CL_APLHA = CL_alpha_WB+(CL_alpha_H)*(eta_H)*(S_H/S)*(1-de_dalpha_at_mach);
CL_iH = eta_H*(S_H/S)*CL_alpha_H;
CL_delta_E = CL_iH*tau_E;

CL_alphadot_H = 2*CL_alpha_H*(eta_H)*(S_H/S)*(X_bar_ACH-X_bar_cg_H)*(d_episilon_d_alpha_mach);
CL_alphadot_W = 2*CL_alpha_WB*(eta_H)*(S_H/S)*(x_bar_AC_B-X_bar_cg_W)*(d_episilon_d_alpha_mach);
CL_alphadot = CL_alphadot_W+CL_alphadot_H;

B = sqrt(1-mach^2*(cos(gamma_025))^2);
CL_qw_at_mach = (.5+2*abs(xbar_AC_W-Xcg))*CL_alpha_W_at_mach_zero;
CL_qW = ((AR+2*cos(gamma_025))/(AR*B+2*cos(gamma_025)))*CL_qw_at_mach;

CL_qH_at_mach = (.5+2*abs(X_bar_ACH-Xcg))*cl_alpha_H_mach; %***THESE ARE LARGE NUMBERS***
CL_qH = ((AR_H+2*cos(gamma_025))/(AR_H*B+2*cos(gamma_025)))*CL_qH_at_mach;%***THESE ARE LARGE NUMBERS***
CL_q_tot = CL_qW+CL_qH;  %***THESE ARE LARGE NUMBERS***

%BOOK
Cm_alpha = -.7;
C_m_alphadot = -6.95;
C_m_delta_E = -1.12;
C_m_i_H = 0;
C_m_q = -18.7;

%% Aerodynamic Parameters 2

% Find CY_beta
Kint = 0; %about zero for our plane
SPtoV = 1.9; %ft^2
CY_beta_B = -2*Kint*(SPtoV/S); %this should be zero
gamma_w = deg2rad(3);
CY_beta_w = -.001*abs(gamma_w)*57.3;
CY_beta_WB = CY_beta_B+CY_beta_w;
CY_beta_H = 0; % Assumed negliable
CRV = 6;%ft
BV = 4.8;%ft
CTV = 2.5;%ft
S_V = (BV/2)*CRV*(1+(CTV/CRV));
AR_V = (BV)^2/S_V;
r_1 = 2.2;%FT
b_v = 4.8;%ft
two_r1_bv = (2*r_1)/b_v;
bv_over_two_r1 = b_v/(2*r_1);
b_H = 14; %ft
l_B = 29.2; %ft
b_H_over_l_b = b_H/l_B;
lambda_v = 2.5/6;
c_1 = 1.55; %figure 4.15
Z_H = -3.1;
x_AC = 5.1;
K_HV = 1.16; 
X_AC_H_to_V = 1.5;
cbarV = 2.5;
c2c_2 = .92;
C_VT = 0.75; % From figure 4.22 Napolitano
AR_V_eff = c_1 * AR_V * (1+K_HV*(c2c_2-1));
CYbetav_eff = 1.35; % approximate from figure 4.20
CY_beta_V = -2*C_VT*CYbetav_eff*(S_V/S);

CY_Beta = CY_beta_WB+CY_beta_H+CY_beta_V;
display(CY_Beta)

% ---
CY_deta_A = 0; % considered negliable due to geometry
display(CY_deta_A)
% ---
%FIND CY_delta_R
c_bar_R = 1.4; %ft
c_bar_TV = 2.5; %ft
tau_r = 0.75; 
eta_V = 0.9;
K_HV = 1.15;
K_R = .95; % VERIFY THIS

lambda_LEV = deg2rad(33);
lambda_V = tan(lambda_LEV) - ((4*(1-lambda_v))/(AR_V*(1+lambda_v)));
% lambda_LEV_half= tan(lambda_LEV) - (((4*.5)*(1-lambda_V))/(AR_V*(1+lambda_V)))
lambda_LEV_half = -0.0798;
% lambdaLEV_rad_quarter = tan(lambda_LEV_half) - (((4*.25)*(1-lambda_V))/(AR_V*(1+lambda_V)))
lambdaLEV_rad_quarter = -0.4445;
k_V = 1 + ((8.2 - (2.3*lambda_LEV) - (AR_V*(0.22-(.153*lambda_LEV))))/100);
C_L_alpha_V = (2*pi*AR_V_eff)/(2 + sqrt(((AR_V_eff*(1-mach^2))/(k_V))*(1+((tan(lambda_LEV_half)^2)/(1-mach^2)))+4)) ;%Polhamus Formula in chaoter 2 using vertical tail specifc values
C_Y_delta_R = abs(C_L_alpha_V) *eta_V*(S_V/S)*K_R*tau_r;
display(C_Y_delta_R)

% ---
%FIND Cn_Beta

Cn_Beta_H = 0;
CN_Beta_W = Cn_Beta_H;
lB = 18.28; %ft
SBs = 80.2; %ft
z1 = 11.4;
z2 = 22;
z_max = 4.4;%ft
w_max = 9; %ft
KN = 0.0005;  %emperical factor from figure 4.68


%% Connect it to Simulink
modelname = 'Khan_hw_4';
sim(modelname)
