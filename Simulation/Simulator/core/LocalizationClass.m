classdef LocalizationClass < dynamicprops
    properties (SetAccess = private)
        method              % {wv, pf, kf}
        robot               % RobotClass object to extract components readings and transformation
        map                 % World class object
        pose                % pose mean: 3D vector (x, y, theta)'
        cov                 % pose covariance: 3x3 matrix
        ext_sensor_label    % Label of exteroceptive sensor to use
        timestamp           % Used to compute d_t
    end
    methods
        function obj = LocalizationClass(varargin)
            % Constructor
            
            % Decode input parameters
            parameters = varargin_to_parameters(varargin);
            % Update properties
            if ~isfield(parameters, 'method')
                error('Valid localization method is required!')
            end
            obj.method = parameters.method;
            if ~isfield(parameters, 'robot')
                error('robot is required!')
            end
            obj.robot = parameters.robot;
            if isfield(parameters, 'map')
                if isa(parameters.map, 'WorldClass')
                    obj.map = parameters.map;
                else
                    error('Map is not a WorldClass object!')
                end
            end
            if isfield(parameters, 'pose')
                obj.pose = parameters.pose(:);
            else
                obj.pose = [0, 0, 0]';
            end
            if isfield(parameters, 'cov')
                obj.cov = parameters.cov;
            else
                obj.cov = zeros(3);
            end
            % Add any additional properties
            basic_fields = {'method', 'robot', 'map', 'pose', 'cov'};
            additional_fields = setdiff(fieldnames(parameters), basic_fields);
            for i = 1:length(additional_fields)
                property_name = additional_fields{i};
                obj.set_property(property_name, parameters.(property_name))
            end
            % Method-specific initialization
            if strcmp(obj.method, 'pf')
                % Initialize particles
                if isprop(obj, 'particles')
                    obj.set_property('n_particles', size(obj.particles, 2));
                else
                    % All particles are at the initial pose
                    obj.set_property('particles', repmat(obj.pose, [1, obj.n_particles]))
                end
            end
            % Start the timer
            obj.timestamp = tic;
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
        
        function h = plot(obj, col)
            % Default parameters
            if nargin < 2
                col = 'b';
            end
            % Plot mean and covariance of pose_xy
            h = [];
            h = plot(obj.pose(1), obj.pose(2), [col 's'], 'MarkerSize', 5, 'MarkerFaceColor', col);
            if ~ishold
                hold on
            end
            h = [h, plot_k_sigma_ellipse(obj.cov(1:2, 1:2), obj.pose(1:2), 3, col);];
            % Method-specific plots
            if strcmp(obj.method, 'pf')
                % plot particles as circles
                h = [h, plot(obj.particles(1,:), obj.particles(2,:), [col '.'], 'MarkerSize', 5)];
            end
        end

        function update(obj, omegas_map)
            % Works with encoders and range sensors ONLY.
            % TODO: work with IMU.

            % Compute Change in time
            dt = toc(obj.timestamp);
            obj.timestamp = tic;        % reset stopwatch
            % Consider different methods
            if strcmp(obj.method, 'wv')
                % Use wheel velocities
                % (1) Compute pose mean
                [obj.pose, Dp_p, Dp_w, omegas_rhr] = kinematics_pose_estimation(obj.robot, obj.pose, dt, omegas_map);
                % (2) Compute pose covariance
                Sigma_d = diag(abs(omegas_rhr) * obj.robot.wheel_error_constant);
                obj.cov = Dp_p * obj.cov * Dp_p' + Dp_w * Sigma_d * Dp_w';
            elseif strcmp(obj.method, 'pf')
                % (1) Sample from Motion Model and update each particle
                %     (1a) Find the exact next state of each particle
                [obj.particles, ~, Dp_w, omegas_rhr] = kinematics_pose_estimation(obj.robot, obj.particles, dt, omegas_map);
                %     (1b) Add noise to each particle
                noise_sigma = sqrt(obj.robot.wheel_error_constant * abs(omegas_rhr));
                omegas_noise = bsxfun(@times, noise_sigma, randn(length(omegas_rhr), obj.n_particles));
                for i = 1:obj.n_particles
                    obj.particles(:, i) = obj.particles(:, i) + Dp_w(:, :, i) * omegas_noise(:, i);
                end
                % Continue if map is available with LiDAR
                if ~isempty(obj.map) && ~isempty(obj.ext_sensor_label)
                    % (2) Assign weight to each particle
                    weights = obj.get_particles_weights();
                    % (3) Resample particles based on their weights
                    p_ind = obj.low_variance_resampler(weights);
                    if ~isempty(p_ind)
                        obj.particles = obj.particles(:, p_ind);
                    end
                end
                % (4) Compute pose mean and covariance
                obj.pose = mean(obj.particles, 2);
                dif = bsxfun(@minus, obj.particles, obj.pose);
                dif(3, :) = wrapToPi(dif(3, :));
                obj.cov = dif * dif' / (obj.n_particles - 1);    
            elseif strcmp(obj.method, 'kf')
                % Later...
            end
        end
    
        function weights = get_particles_weights(obj, max_n_measurements)
            % Default parameters
            if nargin < 2
                % Max number of measurements to consider for speed
                max_n_measurements = 50;
            end
            % Get exteroceptive sensor information
            sensor = obj.robot.components_tree.get(obj.ext_sensor_label);
            % Works only with LiDAR.
            if ~strcmp(sensor.type, 'lidar-2d')
                weights = [];
                return
            end
            % Transformation from sensor frame to robot frame
            T_comp_robot = obj.robot.component_to_robot_world_transfromation(obj.ext_sensor_label);
            % Extract all ranges and compute all angles
            rho = sensor.depth;
            n_angles = floor(sensor.angle_span / sensor.angle_resolution);
            alpha = linspace(-sensor.angle_span/2, sensor.angle_span/2, n_angles);
            % Convert angles to radian if necessary
            try
                if strcmp(sensor.angle_unit, 'degree')
                    alpha = deg2rad(alpha);
                end
            catch
            end
            % Compute angle threshold for ray casting
            angle_threshold = abs(alpha(2) - alpha(1)) / 2;
            % Use only finite depths
            ind_finite = find(isfinite(rho));
            if isempty(ind_finite)
                % Case 1: No landmark is visible
                rho(:) = sensor.max_range;
                n0 = length(rho);
            else
                % Case 2: Some landmarks are visible
                n0 = length(ind_finite);
            end
            n_elem = min(n0, max_n_measurements);
            res = n0 / n_elem;
            valid_ind = round((1:n_elem) * res - res / 2);
            if ~isempty(ind_finite)
                % Case 2: continuation
                valid_ind = ind_finite(valid_ind);
            end
            rho = rho(valid_ind);
            alpha = alpha(valid_ind);
            % Convert to 3D ray directions in sensor reference frame
            v0 = [cos(alpha);   sin(alpha);    zeros(1, n_elem)];
            % Create rays for all particle
            rays = zeros(4, n_elem * obj.n_particles);  % initialization
            for i = 1:obj.n_particles
                T_particle_world = pose2d_to_transformation3d(obj.particles(:, i));
                T_sensor_world = T_particle_world * T_comp_robot;
                p = T_sensor_world(1:3, 4);             % ray starting point
                v = T_sensor_world(1:3, 1:3) * v0;      % ray directions vectors
                
                startCol = (i-1) * n_elem + 1;          % start column index
                finishCol = i * n_elem;                 % finish column index
                
                rays(1, startCol:finishCol) = p(1);
                rays(2, startCol:finishCol) = p(2);
                rays(3:4, startCol:finishCol) = v(1:2, :);  % normally we apply unit vector here, but since v0 is unit vector and it's rotate about the z-axis, the unit vector is unnecessary
            end
            % Apply ray casting
            depth = ray_cast(rays, obj.map.primitives, 'dimension', obj.map.dimension, 'angle_threshold', angle_threshold);
            % Consider lidar max range
            depth(depth > sensor.max_range) = sensor.max_range;
            % Find the likelihood of each particle
            depth = reshape(depth, n_elem, [])';
            var = 10;    % (sensor.range_standard_deviation * depth).^2;
            dif = bsxfun(@minus, depth, rho);
            prob = exp(-0.5 * dif.^2 ./ var) ./ sqrt(2 * pi * var);
            weights = prod(prob, 2)';
        end
    end
    
    methods(Static)
        function p_ind = low_variance_resampler(w)
            % Low variance resampler
            % Returns indices of sampled points
            if isempty(w)
                p_ind = [];
                return
            end
            w = w/sum(w);           % make sure <w> adds up to 1
            M = length(w);          % number of particles
            r = rand() / M;         % equal spacing between roulette sectors
            c = w(1);               % weight accumulation initially
            i = 1;                  % initial index of resampling
            p_ind = zeros(1,M);     % pre-allocate indecies for speed
            for m = 1:M
                u = r + (m - 1) / M;
                while u > c
                    i = i + 1;
                    c = c + w(i);
                end
                p_ind(m) = i;
            end
        end
    end
end