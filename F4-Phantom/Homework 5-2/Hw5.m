% Call the Project 1 Matlab File, but make it so it's only the coefficients
% in a seperate file 
F4ModelCode

%% 

% Small perturbation Assumptions

%Z's 
Z_alpha = - q1*S*(CL_alpha + CDi_1) / m;
Z_u = -(qbar1*S*(c_L_u+(2*c_L_1)))/(m*U1)
%M's 

M_alphadot = (c_m_alphadot*qbar1*S*(cbar^2)) / (2*IYY*V_p_1);
M_q = (c_m_q*qbar1*S*(cbar^2)) / (2*IYY*V_p_1);
M_alpha = (c_m_alpha*qbar1*S*cbar) / IYY;

%Natural Frequencies