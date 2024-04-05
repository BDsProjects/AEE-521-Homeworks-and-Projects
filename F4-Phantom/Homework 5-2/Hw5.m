% Call the Project 1 Matlab File, but make it so it's only the coefficients
% in a seperate file 
F4ModelCode

%% 

% Small perturbation Assumptions

% Lateral-Directional Dimensional Stability and Control Derivatives
Y_beta = (qbar1*S*c_Y_beta)/ m ;%* (ftsec^-2);
Y_p = ((qbar1*S*c_Y_p) / m) *(b / (2*U1)) ;%* (ftsec^-2);
Y_r = ((qbar1*S*c_Y_r)/m) *(b/(2*U1)) ;%* (ftsec^-1);
Y_deltaA = (qbar1*S*c_Y_delta_A / m) ;%* (ftsec^-2);
Y_deltaR = (qbar1*S*c_Y_delta_R / m) ;

L_beta = (qbar1*S*c_L_beta*b) / IXX ;%*(sec^-2);
L_p = ((qbar1*S*b*c_l_p)/IXX) * (b/(2*U1));%*(sec^-1);
L_r = ((qbar1*S*b*c_l_r)/(IXX)) * (b/(2*U1));%*(sec^-1);
L_deltaA = (qbar1*S*c_l_delta_A*b)/IXX ; %*(sec^-2);
L_deltaR = (qbar1*S*c_l_delta_R*b)/IXX ;

N_beta = (qbar1*S*c_n_Tbeta*b)/IZZ;%*(sec^-2);
N_Tbeta = (qbar1*S*c_n_beta*b)/IZZ;
N_p= ((qbar1*S*b*c_n_p)/IZZ)*(b/(2*U1)); %*( sec^(-1));
N_r= ((qbar1*S*b*c_n_r)/(IZZ))*(b/(2*U1)); %*( sec^(-1));
N_deltaA = (qbar1*S*c_n_delta_A * b)/IZZ; % (sec^-1);
N_deltaR= (qbar1*S*c_n_delta_R * b)/IZZ;  %*( sec^(-2));

c_T_X1 = .03; %from the back of the book

%Need X's here 
X_u = -(qbar1*S*(c_D_u+(2*c_D_1)))/(m*V_p_1); %slide 75 lecture 17
X_Tu = -(qbar1*S*(c_T_Xu+(2*c_T_X1)))/(m*V_p_1);

%Z's 
Z_alpha = - q1*S*(CL_alpha + CDi_1) / m;
Z_u = -(qbar1*S*(c_L_u+(2*c_L_1)))/(m*V_p_1); %Slide 72 Lecture 17
%M's 

M_alphadot = (c_m_alphadot*qbar1*S*(cbar^2)) / (2*IYY*V_p_1);
M_q = (c_m_q*qbar1*S*(cbar^2)) / (2*IYY*V_p_1);
M_alpha = (c_m_alpha*qbar1*S*cbar) / IYY;

%Natural Frequencies
omega_n_DR = sqrt(((Y_beta*N_r)-(Y_r*N_beta)+(V_p_1*N_beta))/(V_p_1));
omega_n_SP = 0; %Slide 67 Lecture 17
omega_n_PH = sqrt((-g*Z_u)/V_p_1); %Slide 72 Lecture 17
omega_n_PH_lowMach = (sqrt(2)) * (g/V_P_1); % For low subsonic conditions

%Damping Ratios
zeta_DR = (-1/(2*omega_n_DR)) * ((Y_beta+(V_p_1*N_r))/V_p_1);
zeta_SP = (M_alphadot+M_q+(Z_alpha/V_p_1))/(2 *(sqrt(((Z_alpha*M_q)/V_p_1) - M_alpha)));
zeta_PH = 0; %slide 75 lecture 17




