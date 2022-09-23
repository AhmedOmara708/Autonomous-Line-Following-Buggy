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
    my_alg('wR_set') = 0;
    my_alg('wL_set') = 0;
            
    % Initialise time parameters
    my_alg('sampling_inner') = 0.01;
    my_alg('sampling_outer') = 0.001;
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
    my_alg('t_break') = 0;
    my_alg('t_finish') = 30;
    
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
break_time = 10;

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    %% Outer Loop
    dt = toc(my_alg('t_outer_loop'));
    
    if dt>my_alg('sampling_outer')  % Execute code when desired outer loop sampling time is reached
        my_alg('t_outer_loop') = tic;
        
        %% Add your outer loop code here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       a= my_alg('reflectance raw');
       disp(a)

    my_alg('wR_set') = 0;
    my_alg('wL_set') = 0;       
       
       if (a(1)==500)
           my_alg('wR_set')=7;
           my_alg('wL_set')=0;
           
       elseif (a(2)==500)
           my_alg('wR_set')=7;
           my_alg('wL_set')=0;
           
       elseif (a(3)==500)
           my_alg('wR_set')=7;
           my_alg('wL_set')=0;
           
           elseif (a(4)==500)
           my_alg('wR_set')=7;
           my_alg('wL_set')=7;
           
       elseif (a(5)==500)
           my_alg('wR_set')=7;
           my_alg('wL_set')=7;
           
           elseif (a(6)==500)
           my_alg('wR_set')=0;
           my_alg('wL_set')=7;
           
           elseif (a(7)==500)
           my_alg('wR_set')=0;
           my_alg('wL_set')=7;
           
           elseif (a(8)==500)
           my_alg('wR_set')=0;
           my_alg('wL_set')=7;
           
       elseif (a(1)==2500 && a(2)==2500 && a(3)==2500 && a(4)==2500 && a(5)==2500 && a(6)==2500 && a(7)==2500 && a(8)==2500)
          
           my_alg('t_break') = tic;
           break_time = toc(my_alg('t_break'));           
         
           if break_time < 3
                my_alg('wR_set')=9;
                my_alg('wL_set')=9;                 
           else
                my_alg('t_finish') = time;               
           end
       end
       
       my_alg('t_break') = 0;
      % elseif (a(1)==2500 && a(2)==2500 && a(3)==2500 && a(4)==2500 && a(5)==2500 && a(6)==2500 && a(7)==2500 && a(8)==2500 && break_time < 3)
            

       %    my_alg('wR_set')=9;
        %   my_alg('wL_set')=9;
      % end
      
        
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
    end
    %%
else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    
    % Set servo angle to 0
    my_alg('servo motor') = 0;
    
    % Stop session
    my_alg('is_done') = true;
    
end

return