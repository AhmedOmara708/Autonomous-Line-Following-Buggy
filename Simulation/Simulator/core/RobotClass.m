classdef RobotClass < dynamicprops
    properties (SetAccess = private)
        % Default values
        label           % Unique identifier
        shape           % includes the diameter of ball enclosing robot from its center of mass
        components_tree % Tree containing all sensors, actuators, and wheels
        transformation  % 3D homogeneous transformation matrix wrt World reference frame

        com_data            % stuct containing communication information
        connected = false;  % for communicating with physical robot
    end
    methods
        function obj = RobotClass(varargin)
            % Constructor
            % order of inputs shouldn't matter (check later)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'json_fname'
                        % Read json-file
                        json_fname = varargin{var_ind + 1};
                        json_struct = read_json_file(json_fname);
                        var_ind = var_ind + 2;  % index to next variable
                    case 'json_struct'
                        json_struct = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            % Sanity checks
            if ~isfield(json_struct, 'category')
                error('JSON-struct does not have `category` field!')
            end
            if ~strcmp(json_struct.category, 'mobile_robot')
                error(['Invalid category: `' json_struct.category '`!' ])
            end
            % Update properties
            if isfield(json_struct, 'label')
                if isempty(json_struct.label)
                    error('Invalid `label` field (empty) in JSON-struct!')
                end
                obj.label = json_struct.label;
            else
                error('JSON-struct does not have `label` field!')
            end
            if isfield(json_struct, 'shape')
                t1 = fieldnames(json_struct.shape);
                for j=1:length(t1)
                    t2 = json_struct.shape.(t1{j});
                    if isa(t2, 'cell')
                        t2 = cell2mat(vertcat(t2));
                        % Sanity check
                        if length(t2) ~=2
                            error(['`' t1{j} '` must have length of 2!'])
                        end
                        json_struct.shape.(t1{j}) = t2;
                    end
                end
                obj.shape = json_struct.shape;
            end         
            if isfield(json_struct, 'transformation')
                T = cell2mat(vertcat(json_struct.transformation{:}));
            else
                T = eye(4);
            end
            obj.set_transformation(T, 'nearest');
            % Build components tree
            obj.components_tree = tree(obj);  % initialize with current robot class
            if isfield(json_struct, 'components')
                % Get all `label` and `parent_label`
                labels{1} = obj.label;
                for i = 1:length(json_struct.components)
                    labels{i+1} = json_struct.components{i}.label;
                    parent_labels{i} = json_struct.components{i}.parent_label;
                end                
                % sanity check (make sure labels are unique including robot label)
                if length(labels) ~= length(unique(labels))
                    error('Robot and component labels are not unique!')
                end
                % construct parents with component index instead
                [~, parents] = ismember(parent_labels, labels);
                % Make sure each component has a valid parent, i.e. nonzero
                if ~all(parents)
                    error('At least one component has invalid parent! Check robot JSON file.')
                end
                % Add components
                for i=1:length(parents)
                    st1 = json_struct.components{i};
                    obj.add_component(st1, parents(i))
                end
            end
            % Add any additional properties
            basic_fields = {'category', 'label', 'shape', 'transformation', 'components'};
            additional_fields = setdiff(fieldnames(json_struct), basic_fields);
            for i = 1:length(additional_fields)
                property_name = additional_fields{i};
                obj.set_property(property_name, json_struct.(property_name))
            end
            % Initialize component properties to reduce processing time at
            % the first iteration of session. This needs to be done after
            % building components tree to consider parent-child relationships.
            actuator_signals = {};
            try
                for ind = obj.actuator_inds
                    actuator_signals = cat(2, actuator_signals, ...
                        {obj.components_tree.get(ind).label, 0});
                end
            catch
            end
            % TODO: the loop is only to make sure all initalizations are
            % done. Remove and fix later...
            try
            for i = 1:10
                obj.update(1e-2, WorldClass('fname', 'world_0001.json'), 'kinematics', 'voltage_pwm', actuator_signals{:});
%                 pause(0.02);
            end
            catch
                obj.update(1e-2, WorldClass(), 'kinematics', 'voltage_pwm', actuator_signals{:});
            end
        end
        
        function set_transformation(obj, T, mode)
            if nargin < 3
                mode = '';       % invalid T --> error message
            end
            if is_rigid_transformation(T, 3)
                obj.transformation = T;
            elseif strcmp(mode, 'Idn')
                obj.transformation = eye(4);
            elseif strcmp(mode, 'nearest')
                obj.transformation = get_nearest_transformation_matrix(T);
            else
                error('T is not a vaild rigit transformation is SE(3)!')
            end
        end
        
        function set_property(obj, property_name, property_value)
            % Try/catch is faster than isprop
            try
                obj.(property_name) = property_value;
            catch
                % Create property and add it to object
                P = obj.addprop(property_name);	% dynamic property
                P.SetAccess = 'private';        % make it private
                obj.(property_name) = property_value;
            end
        end

        function add_component(obj, comp_struct, parent_ind)
            % Sanity check
            if ~isfield(comp_struct, 'file_name')
                error('Missing field: `file_name`!')
            end
            % Create component object
            component = ComponentClass('json_fname', comp_struct.file_name);
            % Add label
            if isfield(comp_struct, 'label')
                % Sanity check
                if ~isa(comp_struct.label, 'char')
                    error('Component label must be a string (char)!')
                end
                component.set_property('label', comp_struct.label)
            end
            % Add transformation (with respect to parent component)
            if isfield(comp_struct, 'transformation')
                T = cell2mat(vertcat(comp_struct.transformation{:}));
                component.set_transformation(T, 'nearest');
            end
            % Add additional fields
            required_fields = {'file_name', 'label', 'parent_label', 'transformation'};
            additional_fields = setdiff(fieldnames(comp_struct), required_fields);
            for i = 1:length(additional_fields)
                property_name = additional_fields{i};
                component.set_property(property_name, comp_struct.(property_name))
            end
            % Add component to tree
            obj.components_tree = obj.components_tree.addnode(parent_ind, component);
            
            % For efficient robot update save sensor and actuator indices 
            % in separate arrays based on category
            if strcmp(component.category, 'actuator') || strcmp(component.category, 'sensor')
                property_name = [component.category '_inds'];
                try
                    obj.(property_name) = [obj.(property_name), obj.components_tree.nnodes];
                catch
                    obj.set_property(property_name, obj.components_tree.nnodes)
                end
            end
        end

        function [T_comp_robot, T_comp_world] = ...
                component_to_robot_world_transfromation(obj, comp_id)
            % Find transfromation from component reference frame to robot
            % and world reference frames.
            % Input: component ID (ind or label) in components tree
            
            % Find comp_id
            if isa(comp_id, 'char')
                comp_id = obj.components_tree.label_to_id_map(comp_id);
            end
            % Compute component-to-robot transformation
            T_comp_robot = obj.components_tree.get(comp_id).transformation;
            p = obj.components_tree.getparent(comp_id);   % parent
            while p > 1
                T_comp_robot = obj.components_tree.get(p).transformation * T_comp_robot;
                p = obj.components_tree.getparent(p);
            end
            if nargout > 1
                % Include robot transformation
                T_comp_world = obj.transformation * T_comp_robot;
            end
        end
        
        function h = plot(obj, mode, T_robot_world)
            % Plot the robot in the world reference frame
            % mode options: {'simple', 'full'}
            % If T_robot_world is provided and valid then the function 
            % ignores robot transformation and uses this parameter instead.
            % T_robot_world is assumed valid
            
            % Default values
            if nargin < 3
                T_robot_world = obj.transformation;
                if nargin < 2
                    mode = 'simple';
                end
            end
            % Sanity checks
            if isempty(obj.shape)
                error('Component: `shape` is not available!')
            end
            if strcmp(mode, 'simple')
                % Define the robot scale
                S = 0.5;
                % Define the three vertices around the origin
                X1 = S * [1 0 0;-0.5 -0.5 0 ;-0.5 0.5 0;1 0 0]';
                % Transform using T
                X1 = T_robot_world * cart2homog(X1);
                % Plot all
                h = patch(X1(1,:), X1(2,:), 'r', 'EdgeColor', 'b');
            elseif strcmp(mode, 'full')
                % Plot robot chassis
                h = plot_shape(obj.shape, 'transformation', T_robot_world);
                hold on
                % Plane navigation lights
                lights_homog = [0, -obj.shape.diameter / 2, 0, 1;
                                0, obj.shape.diameter / 2, 0, 1]';
                lights = homog2cart(T_robot_world * lights_homog);
                h = [h plot3(lights(1, 1), lights(2, 1), lights(3, 1), 'g.', 'MarkerSize', 20)];
                h = [h plot3(lights(1, 2), lights(2, 2), lights(3, 2), 'r.', 'MarkerSize', 20)];
                % Plot all other components
                for i = 2:obj.components_tree.nnodes()
                    % Compute component-to-world transformation
                    T_comp_world = T_robot_world * obj.component_to_robot_world_transfromation(i);
                    % Plot component
                    h = [h obj.components_tree.get(i).plot(T_comp_world)];
                end
                %hold off
            else
                error(['Plotting robot of mode`' mode '` is not recognized!'])
            end
        end
                
        function h = plot_measurements(obj, component_identifier, ...
                dimension, T_robot_world)
            % plot sensor measurements
            % If T_robot_world is provided and valid then the function 
            % ignores robot transformation and uses this parameter instead.
            
            % Default values
            if nargin < 4
                T_robot_world = obj.transformation;
                if nargin < 3
                    dimension = 2;
                end
            end
            h = [];
            % Find component_id
            if isa(component_identifier, 'char')
                component_id = obj.components_tree.label_to_id_map(component_identifier);
            else
                component_id = component_identifier;
            end
            % Consider component based on type
            component = obj.components_tree.get(component_id);
            use_lidar_2d = strcmp(component.type, 'lidar-2d');
            use_sonar = strcmp(component.type, 'sonar');
            if use_lidar_2d || use_sonar
                try
                    % Compute component transfromation to world frame
                    T_comp_world = T_robot_world * obj.component_to_robot_world_transfromation(component_id);
                    % Create rays in world frame
                    [p, v] = exteroceptive_sensor_to_rays(component, T_comp_world, dimension);
                    polygon.type = 'polygon';
                    polygon.color = 'blue';
                    if strcmp(component.type, 'lidar-2d')
                        dist = repmat(min(component.max_range, component.depth), [dimension, 1]);
                    elseif strcmp(component.type, 'sonar')
                        dist = min(component.max_range, component.depth);
                    end
                    polygon.vertices = [p, repmat(p, [1, size(v, 2)]) + v .* dist];
                    hold on
                    h = plot_shape(polygon);
                    %hold off
                catch
                end
            else
                % TODO: Add encoder, lidar-1d...
            end
        end
                
        function update_component(obj, component_identifier, varargin)
            % varargin contains world if robot is not connected (simulated)
            % Consider if the robot is connected (even if it's not clear in
            % this class) so we use `direct` to highlight this case.
            % Also consider different categories. See below for different
            % options
            % component_identifier can be either label or index.
            % TODO: Make a comprehensive docstring
            
            % Find component_id
            if isa(component_identifier, 'char')
                component_id = obj.components_tree.label_to_id_map(component_identifier);
            else
                component_id = component_identifier;
            end
            % Real robot flag
            use_real_robot = obj.connected;
            % Decode input parameters
            parameters = varargin_to_parameters(varargin);
            % Consider component based on type
            component = obj.components_tree.get(component_id);
            use_lidar_2d = strcmp(component.type, 'lidar-2d');
            use_sonar = strcmp(component.type, 'sonar');
            use_dc_motor = strcmp(component.type, 'dc-motor');
            use_servo_motor = strcmp(component.type, 'servo-motor');
            use_encoder = strcmp(component.type, 'encoder');
            use_reflectance = strcmp(component.type, 'reflectance');
            if use_lidar_2d || use_sonar
                use_simulation = isfield(parameters, 'world');
                if use_real_robot
                    if use_lidar_2d
                        % **** THIS FUNCTION ONLY WORKS WITH RPLIDAR *****
                        % TODO: Make it more general to other sensors.
                        [done, value] = obj.get_request(component.label);
                        if done
                            % Fix depth
                            depth = value(1:end-1) / 1000;  % convert into meters
                            depth = circshift(depth, [0, -floor((length(value) - 1) / 2)]);  % start at -180 deg
                            depth(depth > component.max_range | depth < 0.1) = inf; % handle invalid values
                            % Compute angle resoluion
                            angle_resolution = 2 * pi / length(depth);
                            % Convert angles to radian if necessary
                            try
                                if strcmp(component.angle_unit, 'degree')
                                    angle_resolution = rad2deg(angle_resolution);
                                end
                            catch
                            end
                        else
                            angle_resolution = 1;
                            depth = inf(1, component.angle_span / angle_resolution);
                        end
                        component.set_property('depth', depth);
                        component.set_property('angle_resolution', angle_resolution);
                    elseif use_sonar
                        [~, value] = obj.get_request(component.label);
                        component.set_property('depth', value)
                    end
                elseif use_simulation
                    % Compute component transfromation to world frame
                    [~, T_comp_world] = obj.component_to_robot_world_transfromation(component_id);
                    % Simulate depth
                    if use_lidar_2d
                        depth = simulate_lidar2d(component, parameters.world, T_comp_world);
                    elseif use_sonar
                        depth = simulate_sonar(component, parameters.world, T_comp_world);
                    end
                    % Add as object property (create new one if necessary)
                    component.set_property('depth', depth)
                else
                    error(['Cannot update component ' component.type '! Connect to real robot or provide world for simulation.'])
                end
            elseif use_dc_motor || use_servo_motor
                % Sanity checks and mode selection (based on type)
                if  use_dc_motor && isfield(parameters, 'voltage_pwm');
                    input_signal_mode = 'voltage_pwm';
                elseif use_dc_motor && isfield(parameters, 'omega_setpoint');
                    input_signal_mode = 'omega_setpoint';
                elseif use_servo_motor && isfield(parameters, 'angle')
                    input_signal_mode = 'angle';
                else
                    error(['Invalid input signal mode for ' component.type '!'])
                end
                % Default values
                try
                    comp_rotation_convension = component.rotation_convension;
                catch
                    comp_rotation_convension = 'rhr';
                end
                % Simulation flag
                use_simulation = isfield(parameters, 'delta_time') || use_servo_motor;

                if use_real_robot
                    % Consider `input_signal_mode`
                    obj.set_request(component.label, input_signal_mode, parameters.(input_signal_mode));
                elseif use_simulation
                    % Simulate component based on type
                    if use_dc_motor
                        try
                            state = [component.theta,	component.omega,	component.i]';
                        catch
                            state = [0, 0, 0]';
                        end
                        new_state = simulate_dc_motor(component, state, ...
                            parameters.delta_time, input_signal_mode, ...
                            parameters.(input_signal_mode));
                        % Update state vector in component
                        component.set_property('theta', new_state(1))
                        component.set_property('omega', new_state(2))
                        component.set_property('i', new_state(3))
                    elseif use_servo_motor
                        theta = simulate_servo_motor(component, parameters.(input_signal_mode));
                        % Update state vector in component
                        component.set_property('theta', theta)
                    end
                    % Update all children states and local transformations
                    for child_id = obj.components_tree.getchildren(component_id)
                        child_comp = obj.components_tree.get(child_id);
                        % Encoder and grearbox are special components that have shaft coupling 
                        % with parent motor, so their relative positions wrt motor are fixed.
                        shaft_coupling_condition = strcmp(child_comp.type, 'encoder') || strcmp(child_comp.type, 'gearbox');
                        % Update child state
                        if shaft_coupling_condition || strcmp(child_comp.category, 'wheel')
                            % Wheel needs to change state in order to be used for pose calculation.
                            try
                                child_comp_rotation_convension = child_comp.rotation_convension;
                            catch
                                child_comp_rotation_convension = 'rhr';
                            end
                            direction = 1;
                            if ~strcmp(comp_rotation_convension, child_comp_rotation_convension)
                                direction = -1;
                            end
                            child_comp.set_property('theta', direction * component.theta)
                            if use_dc_motor
                                child_comp.set_property('omega', direction * component.omega)
                            end
                        end
                        % Update child transfromation
                        if ~shaft_coupling_condition
                            % Exclude encoder and gearbox becasue they only change states
                            % Make sure T_0 is in child that saves initial transfromation
                            try
                                T_0 = child_comp.T_0;
                            catch
                                T_0 = child_comp.transformation;
                                child_comp.set_property('T_0', T_0)
                            end
                            % Make sure theta is in rhr rotation convension
                            theta = component.theta;
                            if ~strcmp(comp_rotation_convension, 'rhr')
                                theta = -theta;
                            end
                            % Apply rotation only
                            T_comp_motor = eye(4);      % initialize
                            T_comp_motor(1:3, 1:3) = rotationVector2Matrix([0, 0, theta]);
                            % update child component transformation
                            child_comp.set_transformation(T_comp_motor * T_0)
                        end
                    end
                else
                    error(['Cannot update component ' component.type '! Connect to real robot or provide valid inputs for simulation.'])
                end
            elseif use_encoder
                % Only update in real robot becuase in simulation
                % actuator will change its state
                if use_real_robot
                    [~, value] = obj.get_request(component.label);
                    component.set_property('omega', value)
                end
            elseif use_reflectance
                use_simulation = isfield(parameters, 'world');
                if use_real_robot
                    [~, value] = obj.get_request(component.label);
                    component.set_property('value', value)
                    [~, unit_readings] = obj.get_request([component.label '_raw']);
                    component.set_property('unit_readings', unit_readings)
                elseif use_simulation
                    % Compute component transfromation to world frame
                    [~, T_comp_world] = obj.component_to_robot_world_transfromation(component_id);
                    % Simulate reflectance sensor
                    [value, unit_readings] = simulate_reflectance(component, parameters.world, T_comp_world);
                    % Add as object property (create new one if necessary)
                    component.set_property('value', value)
                    component.set_property('unit_readings', unit_readings)
                else
                    error(['Cannot update component ' component.type '! Connect to real robot or provide world for simulation.'])
                end
            end
        end
        
        function update_pose(obj, mode, d_t)
            % Mode is either kinematics or dynamics (not for all
            % configurations yet, except DDR)
            % This function is only needed for simulation! It can be used
            % in localization but one must read from the encoders the wheel
            % angular velocities.
            %
            % Add w's noise constant K to parameters that represents the
            %       unmodeled wheel interaction with world surface. Assume that w's are
            %       decoupled so generating zero-mean Gaussain is easy
            %       (NOT multi-variate distribution with correlation)
            %
            
            % Only use in simulation
            if obj.connected
                return
            end
            % Get all active wheels (attached directly to DC motors)
            % Return indices in components tree.
            try
                active_wheel_inds = obj.active_wheel_inds;
            catch
                % Find all indices
                active_wheel_inds = [];
                for i = 2:obj.components_tree.nnodes
                    if strcmp(obj.components_tree.get(i).category, 'wheel')
                        % Make sure wheel is directly coupled with a DC motor
                        p = obj.components_tree.getparent(i);
                        if strcmp(obj.components_tree.get(p).type, 'dc-motor')
                            active_wheel_inds = [active_wheel_inds, i];
                        end
                    end
                end
                % Create and save as property for next iterations
                obj.set_property('active_wheel_inds', active_wheel_inds)
            end
            % Consider different modes
            if strcmp(mode, 'kinematics')
                % Prepare inputs to kinematics_pose_estimation function
                pose_old = transformation3d_to_pose2d(obj.transformation);
                omegas_map = containers.Map('KeyType','double','ValueType','double');
                for wheel_ind = active_wheel_inds;
                    wheel_comp = obj.components_tree.get(wheel_ind);
                    try
                        % No need to check rotation convension here because
                        % it's done inside kinematics model
                        omega = wheel_comp.omega;
                        % Add zero-mean Gaussian noise if necessary
                        try
                            % noise standard deviation
                            sigma = sqrt(obj.wheel_error_constant * abs(omega));
                            % additive noise
                            omega = omega + sigma * randn;
                        catch
                        end
                    catch
                        omega = 0;
                    end
                    omegas_map(wheel_ind) = omega;
                end
                % Compute robot new pose 
                pose_new = kinematics_pose_estimation(obj, pose_old, d_t, omegas_map);
                % Update robot transfromation
                obj.transformation = pose2d_to_transformation3d(pose_new);
            else
                error(['Mode: ' mode ' is not implemented!'])
            end
        end
        
        function sensor_readings = update(obj, d_t, world, robot_mode, ...
                dc_motor_input_mode, varargin)
            % Update robot given actuator signals
            % Any actuator signal not provided then old input persists.
            % Add docstring...
            % In physical robot put empty array for unnecessary parameters
            % d_t: change in time (simulation)
            % world: WorldClass object (simulation)
            % robot_mode: kinematics or dynamics (simulation)
            % dc_motor_input_mode: voltage_pwm or omega_setpoint
            % varargin: even number of actuator label and signal value.
            %
            % Returns
            %   sensor_readings: hash table with label as key.

            % Update input actuators
            var_ind = 1;
            while var_ind <= length(varargin)
                actuator_label = varargin{var_ind};
                if strcmp(obj.components_tree.get(actuator_label).type, 'dc-motor')
                    obj.update_component(actuator_label, dc_motor_input_mode, varargin{var_ind + 1}, 'delta_time', d_t)
                elseif strcmp(obj.components_tree.get(actuator_label).type, 'servo-motor')
                    obj.update_component(actuator_label, 'angle', varargin{var_ind + 1})
                end
                var_ind = var_ind + 2;  % index to next variable
            end
            % Update robot true pose (runs only in simulation)
            obj.update_pose(robot_mode, d_t)
            % Update all sensors
            if nargout > 0
                sensor_readings = containers.Map();
            end
            obj.update_requests();  % Read from physical robot (if possible)
            try
                for ind = obj.sensor_inds
                    obj.update_component(ind, 'world', world)
                    if nargout > 0
                        % This part is added for convienence only but it will
                        % increase processing time
                        component = obj.components_tree.get(ind);
                        if strcmp(component.type, 'lidar-2d') || strcmp(component.type, 'sonar')
                            sensor_readings(component.label) = component.depth;
                        elseif strcmp(component.type, 'encoder')
                            sensor_readings(component.label) = component.omega;
                        elseif strcmp(component.type, 'reflectance')
                            sensor_readings(component.label) = component.value;
                            sensor_readings([component.label ' raw']) = component.unit_readings;
                        end
                    end
                end
            catch
            end
        end
        
        %% Communication Functions
        % https://www.dropbox.com/home/Compabot%20V1.0/Firmware/Compa-Bot1.7/Documents?preview=puzzlebot_readme.txt
        function connect(obj, ip, port, protocol)
            % Default values
            if nargin < 4
                protocol = 'udp';
                if nargin < 3
                    port = 3141;
                end
            end

            try
                obj.com_data = Connect(ip, port, protocol);
                obj.connected = true;
            catch
                obj.connected = false;
            end
        end
        
        function disconnect(obj)
            if obj.connected
                try
                    % Stop all motors
                    for i = obj.actuator_inds
                        component = obj.components_tree.get(i);
                        if strcmp(component.type, 'dc-motor')
                            % Try both modes
                            obj.set_request(component.label, 'omega_setpoint', 0);
                            pause(5e-2)
                            obj.set_request(component.label, 'voltage_pwm', 0);
                            pause(5e-2)
                        end
                    end
                catch
                end
                % diconnect communication
                Disconnect(obj.com_data)
                obj.connected = false;
                obj.com_data = [];
            end
        end
        
        function done = set_request(obj, component_label, input_mode, ...
                value)
            % Supports dc motors (pwm or angular velocity) and servo motors (angle in degrees)
            % NOTE: This is a hard-coded function!
            % TODO: rewrite to have it work based on labels (that needs
            % defining labels in the firmeware).
            try
                % Sanity checks
                if strcmp(input_mode, 'angle')
                    % Assume we have only one servo per robot
                    % Convert value to degrees
                    label_in_firmware = 'ServoAngle';
                    value = rad2deg(value);
                elseif strcmp(input_mode, 'omega_setpoint')
                    % Hard-coded part
                    if strcmpi(intersect(component_label, 'right'), sort('right'))
                        label_in_firmware = 'VelocitySetR';
                    elseif strcmpi(intersect(component_label, 'left'), sort('left'))
                        label_in_firmware = 'VelocitySetL';
                    end
                elseif strcmp(input_mode, 'voltage_pwm')
                    % Hard-coded part
                    if strcmpi(intersect(component_label, 'right'), sort('right'))
                        label_in_firmware = 'ControlR';
                    elseif strcmpi(intersect(component_label, 'left'), sort('left'))
                        label_in_firmware = 'ControlL';
                    end
                end
                
                % Create message
                msg.stamp = GetTime(obj.com_data);
                msg.data = value;
                SendTopicMsg(obj.com_data, label_in_firmware, msg);
                done = true;
            catch
                done = false;
            end
        end
        
        function done = update_requests(obj)
            done = false;
            if obj.connected
                try
                    obj.com_data = SpinOnce(obj.com_data);
                    done = true;
                catch
                end
            end
        end
        
        function [done, value] = get_request(obj, component_label)
            % Supports encoder, sonar, and reflectance sensor
            % NOTE: This is a hard-coded function!
            % TODO: rewrite to have it work based on labels (that needs
            % defining labels in the firmeware).
            done = false;
            value = 0;      % TODO: It's better to have empty instead of 0 to remove any ambiguity with receiving actual value of 0.
            try
                % Sanity checks
                if strcmpi(component_label, 'right encoder')
                    label_in_firmware = 'VelocityEncR';
                elseif strcmpi(component_label, 'left encoder')
                    label_in_firmware = 'VelocityEncL';
                elseif strcmpi(component_label, 'sonar')
                    label_in_firmware = 'SonarDistance';
                elseif strcmpi(component_label, 'lidar')
                    label_in_firmware = 'LidarScan';
                elseif strcmpi(component_label, 'reflectance')
                    label_in_firmware = 'LineSensor';
                elseif strcmpi(component_label, 'reflectance_raw')
                    label_in_firmware = 'LineSensorRaw';
                end
                
                msg = GetTopicMsg(obj.com_data, label_in_firmware);
                if msg.status ~= -1
                    value = msg.data;
                    done = true;
                end
            catch
            end
        end
        
        
    end
end