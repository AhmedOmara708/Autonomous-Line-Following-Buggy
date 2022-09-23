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
        
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];     
    
    my_alg('light_1') = [];
    my_alg('light_2') = [];
    my_alg('light_3') = [];
    my_alg('light_4') = []; 
    my_alg('light_5') = []; 
    my_alg('light_6') = [];
    my_alg('light_7') = [];
    my_alg('light_8') = []; 
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];     
            
    % Initialise time parameters
    my_alg('sampling_inner') = 0.01;
    my_alg('sampling_outer') = 0.001;
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
    my_alg('t_finish') = 15;
    
    my_alg('uR_previous') = 0;
    my_alg('uL_previous') = 0;
    my_alg('error_right_previous') = 0;
    my_alg('error_left_previous') = 0;    
    
    % desired wheel velocity 
    my_alg('w_desired') = 7;
        
    % Servo motor angle (1.57 radians = move servo to the left (towards the wall))
%      my_alg('servo motor') = 1.57/2;
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    %% Outer Loop
    dt = toc(my_alg('t_outer_loop'));
    
    if dt>my_alg('sampling_outer')  % Execute code when desired outer loop sampling time is reached
        my_alg('t_outer_loop') = tic;
        
        %% Add your outer loop code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       light = my_alg('reflectance raw');
       
       my_alg('light_1') = [my_alg('light_1') light(1)];
       my_alg('light_2') = [my_alg('light_2') light(2)];
       my_alg('light_3') = [my_alg('light_3') light(3)];
       my_alg('light_4') = [my_alg('light_4') light(4)];
       my_alg('light_5') = [my_alg('light_5') light(5)];
       my_alg('light_6') = [my_alg('light_6') light(6)];
       my_alg('light_7') = [my_alg('light_7') light(7)];
       my_alg('light_8') = [my_alg('light_8') light(8)];

              
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
    plot(my_alg('light_1')); 
    
    figure(4);
    clf
    plot(my_alg('light_2'));
    
    figure(5);
    clf
    plot(my_alg('light_3'));
    
    figure(6);
    plot(my_alg('light_4'));
    
    figure(7);
    plot(my_alg('light_5')); 
    
    figure(8);
    plot(my_alg('light_6')); 
    
    figure(9);
    plot(my_alg('light_7'));    
    
    figure(10);
    plot(my_alg('light_8'));    
    
    figure(11);
    clf
    plot(my_alg('light_1')); 
    hold on
    plot(my_alg('light_2'));
    hold on
    plot(my_alg('light_3'));
    hold on
    plot(my_alg('light_4'));
    hold on
    plot(my_alg('light_5')); 
    hold on
    plot(my_alg('light_6')); 
    hold on
    plot(my_alg('light_7'));    
    hold on
    plot(my_alg('light_8'));    
    % Stop session
    my_alg('is_done') = true;
    
end

return