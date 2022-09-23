clc, clear all
restoredefaultpath
addpath(genpath(pwd))


% Create Robot
R1 = RobotClass('json_fname', 'puzzle_bot_0002.json');
% Uncomment below for using the real robot (it's simulation otherwise)
% R1.connect('192.168.1.1');

% Create World
W = WorldClass('fname', 'world_empty.json');

% Total duration and sampling time parameters
TotalTime = 2;
t_sampling = 0.02;

t_start = tic;
t_loop = tic;

%% Initialise data

% Initialise motor angular velocity controllers
control_right = MotorControl();
control_left = MotorControl();

% Controller setpoints for right and left wheel
wR_set = 7;
wL_set = 6;

% Wheel angular velocities from the encoders
wR = 0;
wL = 0;

% Vectors for saving input/output data
wR_set_all = [];
wL_set_all = [];
wR_all = [];
wL_all = [];

while toc(t_start)<TotalTime
    
    dt = toc(t_loop);
    
    if(dt>=t_sampling)              % execute code when desired sampling time is reached
        t_loop = tic;
        
        %% Add your code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % uR,uL - control input pwm signals [-1,1]
        % wR,wL - wheel output angular velocities

        %% Replace with your PI controllers here %%%%%%%%%%%%%%%%
        % Right wheel controller (replace with your controller)        
        uR = control_right.Control(wR_set,wR,dt);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Left wheel controller (replace with your controller)
        uL = control_left.Control(wL_set,wL,dt);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Save data for ploting
        wR_set_all = [wR_set_all wR_set];
        wL_set_all = [wL_set_all wL_set];
        wR_all = [wR_all wR];
        wL_all = [wL_all wL];
                
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
        %% Do not edit this section
        % Update robot in this order: actuators, pose (in simulation), sensors
        actuator_signals = {'right motor', uR, 'left motor', uL};
        sensor_readings = R1.update(dt, W, 'kinematics', 'voltage_pwm', actuator_signals{:});
        
        % Update encoder velocity readings
        wR = sensor_readings('right encoder');
        wL = sensor_readings('left encoder');        
    end

     pause(0.001)
end

% Plot wheel angular velocities
plot(wR_all);
hold on;
plot(wL_all);
plot(wR_set_all);


