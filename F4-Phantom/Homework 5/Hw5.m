
ts = .01;
tf = 30.0;
t = [0:ts:tf];










%K's 
dt = 0.5;
Kp = .05;
Ki = 3;
Kd = 1/50;

































modelName = 'Hw5PID.slx';
sim(modelName)