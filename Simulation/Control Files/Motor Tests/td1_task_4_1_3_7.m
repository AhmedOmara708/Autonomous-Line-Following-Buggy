function my_alg = task_4_1_3_7(my_alg, robot)

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
    
    % Initialise vectors for saving velocity data
    my_alg('wR_all') = [0];
    my_alg('wL_all') = [0];
    my_alg('uR_all') = [0];
    my_alg('uL_all') = [0];
        
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    my_alg('right encoder') = 0;
    my_alg('left encoder') = 0;
    % Initialise time parameters
    my_alg('t_sampling') = 0.01;
    my_alg('t_loop') = tic;
    my_alg('t_finish') = 15;
end

%% Loop code runs here

time = toc(my_alg('tic'));       % Get time since start of session

if time < my_alg('t_finish')     % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
    
    if dt>my_alg('t_sampling')   % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;
        
        %% Add your loop code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %my_alg('right motor') = 0;
        %my_alg('left motor') = 0;  
        
        if time >= 0 && time < 3
            my_alg('right motor') = 0.3;
            my_alg('left motor') = 0.3;
        end
        
        if time >= 3 && time < 6
            my_alg('right motor') = 0.8;
            my_alg('left motor') = 0.8;
        end  
        
        if time >= 6 && time < 9
            my_alg('right motor') = 0.5;
            my_alg('left motor') = 0.5;
        end    
        
        if time >= 9 && time < 12
            my_alg('right motor') = 0.3;
            my_alg('left motor') = 0.3;
        end      
        
        if time >= 12
            my_alg('right motor') = 0;
            my_alg('left motor') = 0;
        end         

        
        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
        my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];
        my_alg('uR_all') = [my_alg('uR_all') my_alg('right motor')];
        my_alg('uL_all') = [my_alg('uL_all') my_alg('left motor')];        
    end

else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    
    % Plot saved velocities for right and left wheel
    figure(2);
    clf
    plot(my_alg('wR_all'));
    xlabel('time')
    ylabel('wR')
    
    figure(3);
    clf
    plot(my_alg('wL_all'));
    xlabel('time')
    ylabel('wL')    
    
    figure(4);
    clf
    plot(my_alg('uR_all'));
    xlabel('time')
    ylabel('uR')    
    
    figure(5);
    clf
    plot(my_alg('uL_all'));
    xlabel('time')
    ylabel('uL')    

end

return