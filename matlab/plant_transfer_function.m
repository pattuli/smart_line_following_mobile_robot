% Load Data
data = readtable('pwm_and_speed_before_pid.csv');
t = data.Var1;      % Time column
u = data.Var2;      % PWM Input column
y = data.Var3;      % Measured Ticks column

% Create IDDATA object (Sample time = 0.05s)
motor_id = iddata(y, u, 0.05);

% Estimate a Transfer Function (The "Plant")
% We assume a 1-pole system for simple DC motors
motor_model = tfest(motor_id, 1, 0); 

fprintf('Motor Model Identified! Opening Simulink...\n');