%% Doublet Section

for i = 1:3
     % Stabilator
    i_H_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
    i_H_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
    doubletMag = 2;
    i_H(i_H_doublet_up_Idx,1) = deg2rad(doubletMag);
    i_H(i_H_doublet_down_Idx,1) = deg2rad(-doubletMag);

    % Aileron
    deltaA_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
    deltaA_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
    doubletMag = 2;
    deltaA(deltaA_doublet_up_Idx,1) = deg2rad(doubletMag);
    deltaA(deltaA_doublet_down_Idx,1) = deg2rad(-doubletMag);

    % Rudder
    deltaR_doublet_up_Idx = find(timeVec >= 5 & timeVec <= 7);
    deltaR_doublet_down_Idx = find(timeVec >= 15 & timeVec <= 17);
    doubletMag = 2;
    deltaR(deltaR_doublet_up_Idx,1) = deg2rad(doubletMag);
    deltaR(deltaR_doublet_down_Idx,1) = deg2rad(-doubletMag);
    
    modelname = 'F4PhantomRunningModel.slx';
    sim(modelname)

    if i==1
        % Plotting for 
        figure % No PID
        sgtitle('Stabilator Doublet Natural Response')
        subplot(4,1,1)
        plot(timeVec, u)
        grid on; xlabel('Time (s)'); ylabel('u (ft/s)');
        subplot(4,1,2)
        plot(timeVec, alpha1)
        grid on; xlabel('Time (s)'); ylabel('Alpha (deg/s)');
        subplot(4,1,3)
        plot(timeVec, theta)
        grid on; xlabel('Time (s)'); ylabel('Theta (deg/s)');
        subplot(4,1,4)
        plot(timeVec, i_H)
        grid on; xlabel('Time (s)'); ylabel('Delta Stabilatator (rad)');

        figure % PID
        sgtitle('Stabilator Doublet PID Response')
        subplot(4,1,1)
        plot(timeVec, u)
        grid on; xlabel('Time (s)'); ylabel('u (ft/s)');
        subplot(4,1,2)
        plot(timeVec, alpha1)
        grid on; xlabel('Time (s)'); ylabel('Alpha (deg/s)');
        subplot(4,1,3)
        plot(timeVec, theta)
        grid on; xlabel('Time (s)'); ylabel('Theta (deg/s)');
        subplot(4,1,4)
        plot(timeVec, i_H_vector_sim)
        grid on; xlabel('Time (s)'); ylabel('Delta Stabilatator (rad)');

        figure % Altitude Plot
        sgtitle('Stabilator Doublet PID Response Altitude')
        plot(timeVec, Zoriginal)
        hold on
        yline(NewAlt, 'r--')
        grid on; xlabel('Time (s)'); ylabel('Altitude (ft)');
        legend('Measured Altitude', 'Desired Altitude', 'Interpreter');

    elseif i==2
        % Plotting for aileron doublet
        figure % Before PID
        sgtitle('Aileron Doublet Natural Response')
        subplot(4,1,1)
        plot(timeVec, beta)
        grid on; xlabel('Time (s)');ylabel('beta (rad)');xlim([0 60]);
        subplot(4,1,2)
        plot(timeVec,phi)
        grid on; xlabel('Time (s)','Interpreter','latex');ylabel('$\phi$ (rad)','Interpreter','latex');xlim([0 60]);
        subplot(4,1,3)
        plot(timeVec, psi,'b-')
        grid on; xlabel('Time (s)');ylabel('psi (rad)');xlim([0 60]);
        subplot(4,1,4)
        plot(timeVec, deltaA(:,2),'b-')
        grid on; xlabel('Time (s)','Interpreter','latex');ylabel('delta_A (rad))');xlim([0 60]);

    else
        % Plotting for rudder doublet
        figure % Before PID
        sgtitle('Rudder Doublet Natural Response')
        subplot(4,1,1)
        plot(timeVec, beta, 'b-')
        grid on; xlabel('Time (s)');ylabel('beta (rad)');
        subplot(4,1,2)
        plot(timeVec,phi,'b-')
        grid on; xlabel('Time (s)');ylabel('phi (rad)');
        subplot(4,1,3)
        plot(timeVec, psi,'b-')
        grid on; xlabel('Time (s)');ylabel('psi (rad)');
        subplot(4,1,4)
        plot(timeVec, deltaR(:,2),'b-')
        grid on; xlabel('Time (s)');ylabel('delta_R (rad)');

    end

end
%% Switch case for input control surface  input

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
%% Output Forces and Moments
disp()
