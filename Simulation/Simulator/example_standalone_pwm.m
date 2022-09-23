clc, clear all
restoredefaultpath
addpath(genpath(pwd))


% Create Robot
R1 = RobotClass('json_fname', 'puzzle_bot_0002.json');
% Uncomment below for using the real robot (it's simulation otherwise)
% R1.connect('192.168.1.1');

% Create World
W = WorldClass('fname', 'world_0001.json');

% Total duration and sampling time parameters
TotalTime = 1;
t_sampling = 0.02;

t_start = tic;
t_loop = tic;

%% Initialise data

% Wheel control pwm signals [-1,1]
uR = 0.7;
uL = 0.6;

% Wheel angular velocities from the encoders
wR = 0;
wL = 0;

% Vectors for saving input/output data
uR_all = [];
uL_all = [];
wR_all = [];
wL_all = [];

while toc(t_start)<TotalTime
    
    dt = toc(t_loop);
    
    if(dt>=t_sampling) % execute code when desired sampling time is reached
        t_loop = tic;
        
        %% Add your code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % uR,uL - control input pwm signals [-1,1]
        % wR,wL - wheel output angular velocities
                                    
        % Save data for ploting
        wR_all = [wR_all wR];
        wL_all = [wL_all wL];        
        uR_all = [uR_all uR];
        uL_all = [uL_all uL];
        
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        

        %% Do not edit this section
        % Update robot in this order: actuators, pose (in simulation), sensors
        actuator_signals = {'right motor', uR , 'left motor', uL};  
        sensor_readings = R1.update(dt, W, 'kinematics', 'voltage_pwm', actuator_signals{:});
        
        % Update encoder velocity readings
        wR = sensor_readings('right encoder');
        wL = sensor_readings('left encoder');   
        
    end

     pause(0.001)
end

% plot saved velocities for right and left wheels
plot(wR_all);
hold on
plot(wL_all);
plot(uR_all);

