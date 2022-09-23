function my_alg = example_sonar(my_alg, robot)
% This function implements two control loops:
%   - the inner loop implements angular velocity controllers for both
%   wheels;
%   - the outer loop implements a proportional controller for holding a
%   constant distance to a wall using the sonar;
%
%  Load world_wall.json in the Gui to test the example.
%
% Mohamed Mustafa, August 2020
% -------------------------------------------------------------------------
%
% Reading data from sensors (if present on the robot)
%    my_alg('right encoder') - right encoder velocity
%    my_alg('left encoder')  - left encoder velocity
%    my_alg('reflectance')   - reflectance sensor output value
%    my_alg('reflectance raw')   - reflectance sensor raw output values
%    my_alg('sonar')         - sonar measured distance (m)
% 
% Sending controls to actuators (if present on the robot)
%    my_alg('right motor')   - sets the right motor input signal (pwm or angular velocity)
%    my_alg('left motor')    - sets the left motor input signal (pwm or angular velocity)
%    my_alg('servo motor')   - sets the servomotor angle (radians)
% -------------------------------------------------------------------------

if my_alg('is_first_time')
    %% Setup initial parameters here
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise wheel angular velocity contollers
    my_alg('wR_set') = 7;
    my_alg('wL_set') = 7;
    
    my_alg('sonar_distance') = [];
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];    
    
    % Initialise time parameters
    my_alg('sampling_inner') = 0.03;
    my_alg('sampling_outer') = 0.1;
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
    my_alg('t_finish') = 10;
    
    my_alg('uR_previous') = 0;
    my_alg('uL_previous') = 0;
    my_alg('error_right_previous') = 0;
    my_alg('error_left_previous') = 0;    
    
    % desired wheel velocity 
    my_alg('w_desired') = 5;
        
    % Servo motor angle (1.57 radians = move servo to the left (towards the wall))
    my_alg('servo motor') = 1.57*3;
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    %% Outer Loop
    dt = toc(my_alg('t_outer_loop'));
    
    if dt>my_alg('sampling_outer')  % Execute code when desired outer loop sampling time is reached
        my_alg('t_outer_loop') = tic;
        
        %% Add your outer loop code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        sonar_dist = my_alg('sonar');  % Read sonar distance from robot
        
        if sonar_dist <0.2
            my_alg('wR_set')=8;
            my_alg('wL_set')=9;
            wR=8;
            wL=9;
        end
            
        if (sonar_dist >0.8 && sonar_dist <2) 
            my_alg('wR_set')=9;
            my_alg('wL_set')=8;
            wR=9;
            wL=8;
        end
        
        if sonar_dist ==0.8  
            my_alg('wR_set')=9;
            my_alg('wL_set')=9;
            wR=8;
            wL=8;
        end 
        
        my_alg('sonar_distance') = [my_alg('sonar_distance') my_alg('sonar')];
        
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    end
    %%
    
    %% Inner Loop
    dt = toc(my_alg('t_inner_loop'));
    
    if dt>my_alg('sampling_inner')  % Execute code when desired inner loop sampling time is reached 
        my_alg('t_inner_loop') = tic;

        %% Add your inner loop code here (replace with your controller)%%%
        kp_right = 0.2305389222;
        ki_right = 7.684630739;
        kd_right = 0.001729041;
        
        kp_left = 0.2305389222;
        ki_left = 7.684630739;
        kd_left = 0.001729041;        
        
        error_right = my_alg('wR_set') - my_alg('right encoder');        
        error_left = my_alg('wL_set') - my_alg('left encoder');
        
        
        uR = (kp_right*error_right) + (my_alg('uR_previous') + ki_right*error_right*my_alg('sampling_inner')) + (kd_right*((error_right - my_alg('error_right_previous'))/my_alg('sampling_inner'))) ;
        my_alg('uR_previous') = uR;        
        my_alg('error_right_previous') = error_right;
        
        uL = (kp_left*error_left) + (my_alg('uL_previous') + ki_left*error_left*my_alg('sampling_inner')) + (kd_left*((error_left - my_alg('error_left_previous'))/my_alg('sampling_inner'))) ;
        my_alg('uL_previous') = uL;        
        my_alg('error_left_previous') = error_left;
        
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;
        
        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
        my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];        
        
    end
    %%
else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    
    % Set servo angle to 0
    my_alg('servo motor') = 0;
    
    figure(2);
    clf
    plot(my_alg('wR_all'));
    hold on
    plot(my_alg('wL_all'));
    
    figure(3);
    clf
    plot(my_alg('sonar_distance'));   
    
    % Stop session
    my_alg('is_done') = true;
    
end

return