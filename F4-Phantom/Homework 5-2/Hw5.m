% Call the Project 1 Matlab File, but make it S_ft2o it'S_ft2 only the coefficientS_ft2
% in a S_ft2eperate file 
F4ModelCode

%% 

% Small perturbation Assumptions

%Textbook Values
c_T_X1 = -c_D_1; %Page 368 Napolitano 
c_m_T1 = 0;
c_T_X_u = -.064;
c_m_T_u = 0;
c_m_T_alpha = 0;
c_n_T_beta = -.036;

% Lateral-Directional Dimensional Stability and Control Derivatives
Y_beta = (qbar1*S_ft2*c_Y_beta)/ mass ;%* (ftSec^-2);
Y_p = ((qbar1*S_ft2*c_Y_p) / mass) *(b_ft / (2*U1)) ;%* (ftsec^-2);
Y_r = ((qbar1*S_ft2*c_Y_r)/mass) *(b_ft/(2*U1)) ;%* (ftsec^-1);
Y_delta_A = (qbar1*S_ft2*c_Y_delta_A / mass) ;%* (Sec^-2);
Y_delta_R = (qbar1*S_ft2*c_Y_delta_R / mass) ;

L_beta = (qbar1*S_ft2*c_l_beta*b_ft) / IXX ;%*(Sec^-2);
L_p = ((qbar1*S_ft2*b_ft*c_l_p)/IXX) * (b_ft/(2*U1));%*(Sec^-1);
L_r = ((qbar1*S_ft2*b_ft*c_l_r)/(IXX)) * (b_ft/(2*U1));%*(Sec^-1);
L_delta_A = (qbar1*S_ft2*c_l_delta_A*b_ft)/IXX ; %*(Sec^-2);
L_delta_R = (qbar1*S_ft2*c_l_delta_R*b_ft)/IXX ;

N_beta = (qbar1*S_ft2*c_n_T_beta*b_ft)/IZZ;%*(Sec^-2);
N_Tbeta = (qbar1*S_ft2*c_n_beta*b_ft)/IZZ;
N_p= ((qbar1*S_ft2*b_ft*c_n_p)/IZZ)*(b_ft/(2*U1)); %*( Sec^(-1));
N_r= ((qbar1*S_ft2*b_ft*c_n_r)/(IZZ))*(b_ft/(2*U1)); %*( Sec^(-1));
N_deltaA = (qbar1*S_ft2*c_n_delta_A * b_ft)/IZZ; % (Sec^-1);
N_deltaR= (qbar1*S_ft2*c_n_delta_R * b_ft)/IZZ;  %*( Sec^(-2));

%All above signs were verified 



%Need X's here %might need to zero out both of these bc c_D_u
X_u = -(qbar1*S_ft2*(c_D_u+(2*c_D_1)))/(mass*V_P_1); %S_ft2lide 75 lecture 17
X_Tu = (qbar1*S_ft2*(c_T_X_u+(2*c_T_X1)))/(mass*V_P_1); %Both of these were checked for errors

%Z'S_ft2 
Z_alpha = - qbar1*S_ft2*(c_L_alpha + c_D_i_H) / mass;
Z_u = -(qbar1*S_ft2*(c_L_u+(2*c_L_1)))/(mass*V_P_1); %S_ft2lide 72 Lecture 17
%M'S_ft2 

M_alphadot = (c_m_alphadot*qbar1*S_ft2*(cbar_ft^2)) / (2*IYY*V_P_1);
M_q = (c_m_q*qbar1*S_ft2*(cbar_ft^2)) / (2*IYY*V_P_1);
M_alpha = (c_m_alpha*qbar1*S_ft2*cbar_ft) / IYY;
M_u = (qbar1*S_ft2*cbar_ft*(c_m_u+(2*(c_m_1)))) / (U1*IYY);

%Natural FrequencieS_ft2
omega_n_DR = sqrt(((Y_beta*N_r)-(Y_r*N_beta)+(V_P_1*N_beta))/(V_P_1));
omega_n_SP = 0; %S_ft2lide 67 Lecture 17
omega_n_PH = sqrt((-g*Z_u)/V_P_1); %Slide 72 Lecture 17
omega_n_PH_LowMach = (sqrt(2)) * (g/V_P_1); % For low SubSonic conditions

%Damping RatioS_ft2
zeta_DR = (-1/(2*omega_n_DR)) * ((Y_beta+(V_P_1*N_r))/V_P_1);
zeta_SP = (M_alphadot+M_q+(Z_alpha/V_P_1))/(2 *(sqrt(((Z_alpha*M_q)/V_P_1) - M_alpha)));
zeta_PH = (-(X_u+X_Tu)/(2*omega_n_PH)); %S_ft2lide 75 lecture 17

E1 = (Z_u*M_alpha)-(Z_alpha*M_u);
%% display
disp(omega_n_DR) %real
disp(omega_n_SP) %non-real
disp(omega_n_PH) %0
disp(zeta_DR) %non-real
disp(zeta_SP) %real
disp(zeta_PH) %non-real

%% Display as a Table
String_omega_n_DR = num2str(omega_n_DR);
String_omega_n_SP = num2str(omega_n_SP);
String_omega_n_PH = num2str(omega_n_PH);
String_omega_n_PH_LowMach = num2str(omega_n_PH_lowMach);
String_zeta_DR = num2str(zeta_DR);
String_zeta_SP = num2str(zeta_SP);
String_zeta_PH = num2str(zeta_DR);

DisplayTable = ["ω_n_DR:", String_omega_n_DR];
msgbox(DisplayTable, "Homework 5 Answers")







