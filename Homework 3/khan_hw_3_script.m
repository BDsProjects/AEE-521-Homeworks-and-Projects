% Abdul Khan
clear
close all
clc
% Intial Variables
mach = 0.459;
g = 9.81; %m/s
m = 23000; %kg mass of T-37
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

Cm_alpha = -.7;
C_m_alphadot = -6.95;
C_m_delta_E = -1.12;
C_m_i_H = 0;
C_m_q = -18.7;
%% Connect it to Simulink
modelname = 'Khan_hw_3';
sim(modelname)
