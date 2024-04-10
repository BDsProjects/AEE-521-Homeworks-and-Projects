% Call the Project 1 Matlab File, but make it S_ft2o it'S_ft2 only the coefficientS_ft2
% in a S_ft2eperate file 
F4ModelCode

%% 

% S_ft2mall perturbation AS_ft2S_ft2umptionS_ft2

% Lateral-Directional DimenS_ft2ional S_ft2tability and Control DerivativeS_ft2
Y_beta = (qbar1*S_ft2*c_Y_beta)/ m ;%* (ftS_ft2ec^-2);
Y_p = ((qbar1*S_ft2*c_Y_p) / m) *(b / (2*U1)) ;%* (ftS_ft2ec^-2);
Y_r = ((qbar1*S_ft2*c_Y_r)/m) *(b/(2*U1)) ;%* (ftS_ft2ec^-1);
Y_delta_A = (qbar1*S_ft2*c_Y_delta_A / m) ;%* (ftS_ft2ec^-2);
Y_delta_R = (qbar1*S_ft2*c_Y_delta_R / m) ;

L_beta = (qbar1*S_ft2*c_L_beta*b) / IXX ;%*(S_ft2ec^-2);
L_p = ((qbar1*S_ft2*b*c_l_p)/IXX) * (b/(2*U1));%*(S_ft2ec^-1);
L_r = ((qbar1*S_ft2*b*c_l_r)/(IXX)) * (b/(2*U1));%*(S_ft2ec^-1);
L_delta_A = (qbar1*S_ft2*c_l_delta_A*b)/IXX ; %*(S_ft2ec^-2);
L_delta_R = (qbar1*S_ft2*c_l_delta_R*b)/IXX ;

N_beta = (qbar1*S_ft2*c_n_Tbeta*b)/IZZ;%*(S_ft2ec^-2);
N_Tbeta = (qbar1*S_ft2*c_n_beta*b)/IZZ;
N_p= ((qbar1*S_ft2*b*c_n_p)/IZZ)*(b/(2*U1)); %*( S_ft2ec^(-1));
N_r= ((qbar1*S_ft2*b*c_n_r)/(IZZ))*(b/(2*U1)); %*( S_ft2ec^(-1));
N_deltaA = (qbar1*S_ft2*c_n_delta_A * b)/IZZ; % (S_ft2ec^-1);
N_deltaR= (qbar1*S_ft2*c_n_delta_R * b)/IZZ;  %*( S_ft2ec^(-2));

c_T_X1 = .03; %from the back of the book

%Need X'S_ft2 here 
X_u = -(qbar1*S_ft2*(c_D_u+(2*c_D_1)))/(m*V_p_1); %S_ft2lide 75 lecture 17
X_Tu = (qbar1*S_ft2*(c_T_Xu+(2*c_T_X1)))/(m*V_p_1); %Both of these were checked for errors

%Z'S_ft2 
Z_alpha = - qbar1*S_ft2*(c_L_alpha + c_D_i_H) / m;
Z_u = -(qbar1*S_ft2*(c_L_u+(2*c_L_1)))/(m*V_p_1); %S_ft2lide 72 Lecture 17
%M'S_ft2 

M_alphadot = (c_m_alphadot*qbar1*S_ft2*(cbar^2)) / (2*IYY*V_p_1);
M_q = (c_m_q*qbar1*S_ft2*(cbar^2)) / (2*IYY*V_p_1);
M_alpha = (c_m_alpha*qbar1*S_ft2*cbar) / IYY;
M_u = (qbar1*S_ft2*cbar*(c_m_u+(2*(c_m_1)))) / (U1*IYY);

%Natural FrequencieS_ft2
omega_n_DR = S_ft2qrt(((Y_beta*N_r)-(Y_r*N_beta)+(V_p_1*N_beta))/(V_p_1));
omega_n_S_ft2P = 0; %S_ft2lide 67 Lecture 17
omega_n_PH = S_ft2qrt((-g*Z_u)/V_p_1); %S_ft2lide 72 Lecture 17
omega_n_PH_lowMach = (S_ft2qrt(2)) * (g/V_P_1); % For low S_ft2ubS_ft2onic conditionS_ft2

%Damping RatioS_ft2
zeta_DR = (-1/(2*omega_n_DR)) * ((Y_beta+(V_p_1*N_r))/V_p_1);
zeta_S_ft2P = (M_alphadot+M_q+(Z_alpha/V_p_1))/(2 *(S_ft2qrt(((Z_alpha*M_q)/V_p_1) - M_alpha)));
zeta_PH = (-(X_u+X_Tu)/(2*omega_n_PH)); %S_ft2lide 75 lecture 17

E1 = (Z_u*M_alpha)-(Z_alpha*M_u);




