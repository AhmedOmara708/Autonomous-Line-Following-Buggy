classdef MappingClass < dynamicprops
    properties (SetAccess = private)
        method              % {binary_grid, prob_grid}
        robot               % RobotClass object to extract components readings and transformation
        ext_sensor_label    % Label of exteroceptive sensor to use
        
        res                 % Grid resolution (length unit)
        ll_corner           % Lower-left corner (x, y)
        ur_corner           % Upper-right corner (x, y)
        log_odds_grid       % Occupancy grid in log odds
    end
    methods
        function obj = MappingClass(varargin)
            % Constructor
            
            % Decode input parameters
            parameters = varargin_to_parameters(varargin);
            % Update properties
            if ~isfield(parameters, 'method')
                error('Valid mapping method is required!')
            end
            obj.method = parameters.method;
            if ~isfield(parameters, 'robot')
                error('robot is required!')
            end
            obj.robot = parameters.robot;
            if ~isfield(parameters, 'ext_sensor_label')
                error('Exteroceptive sensor label is required!')
            end
            obj.ext_sensor_label = parameters.ext_sensor_label;
            
            if ~isfield(parameters, 'res')
                obj.res = 0.1;
            end
            if ~isfield(parameters, 'll_corner')
                obj.ll_corner = [-5, -5];
            end
            if ~isfield(parameters, 'ur_corner')
                obj.ur_corner = [5, 5];
            end
            
            % Add any additional properties
            basic_fields = {'method', 'robot', 'ext_sensor_label'};
            additional_fields = setdiff(fieldnames(parameters), basic_fields);
            for i = 1:length(additional_fields)
                property_name = additional_fields{i};
                obj.set_property(property_name, parameters.(property_name))
            end            
            
            % Initialize the occupancy grid (all cells are unknown)
            obj.ll_corner = obj.ll_corner(:);
            obj.ur_corner = obj.ur_corner(:);
            nxy = round((obj.ur_corner - obj.ll_corner) / obj.res);
            obj.ur_corner = obj.ll_corner + nxy .* obj.res;
            obj.log_odds_grid = zeros(nxy(2), nxy(1));
            % Method-specific initialization
            if strcmp(obj.method, 'prob_grid')
                % https://www.cs.cmu.edu/~16831-f14/notes/F14/16831_lecture06_agiri_dmcconac_kumarsha_nbhakta.pdf
                pm = 0.5;       % p(m)      initial probability of each cell being occupied
                pm_z = 0.52;    % p(m|z)    where z is the measurement
                log_odds_m = pm / (1 - pm);
                log_odds_m_z = pm_z / (1 - pm_z);
                set_property(obj, 'log_odds_const', log_odds_m_z - log_odds_m);
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
        
        function h = plot(obj, alpha)
            % Default values
            if nargin < 2
                alpha = 1;
            end
            % Convert log odds to probability (actually 1-probability because black is 0)
            prob_hat = 1 ./ (1 + exp(obj.log_odds_grid));
            % Plot as an image (note it is shifted by res/2 to left and down)
            x_lims = [obj.ll_corner(1) obj.ur_corner(1)-obj.res];
            y_lims = [obj.ll_corner(2) obj.ur_corner(2)-obj.res];
            h = imagesc(x_lims, y_lims, prob_hat, [0 1]);
            h.AlphaData = alpha;
            colormap('gray')
            set(gca, 'YDir', 'normal')
        end
        
        function update(obj, robot_pose)
            % Get indices of empty and occupied space in the grid
            [empty_ind, frontier_ind] = get_empty_and_frontier_indices(obj, robot_pose);
            % Consider different methods
            if strcmp(obj.method, 'binary_grid')
                obj.log_odds_grid(frontier_ind) = inf;
                obj.log_odds_grid(empty_ind) = -inf;
            elseif strcmp(obj.method, 'prob_grid')
                obj.log_odds_grid(frontier_ind) = obj.log_odds_grid(frontier_ind) + obj.log_odds_const;
                obj.log_odds_grid(empty_ind) = obj.log_odds_grid(empty_ind) - obj.log_odds_const;
            end
        end
        
        function [empty_ind, frontier_ind] = get_empty_and_frontier_indices(obj, robot_pose)
            % Both empty_ind and frontier_ind are absolute indices NOT rows
            % and columns.
            
            % Get exteroceptive sensor information
            sensor = obj.robot.components_tree.get(obj.ext_sensor_label);
            T_sensor_robot = obj.robot.component_to_robot_world_transfromation(obj.ext_sensor_label);
            T_sensor_world = pose2d_to_transformation3d(robot_pose) * T_sensor_robot;
            % Get rays in the world frame
            [p, v, angle_diff] = exteroceptive_sensor_to_rays(sensor, T_sensor_world);
            % Get valid measurements rhos
            rho = sensor.depth;
            if length(rho) == 1
                % This is for the sonar as it returns a single depth value
                rho = repmat(rho, 1, size(v, 2));
            end
            frontier_ind = isfinite(rho);
            rho(~frontier_ind) = sensor.max_range;     % make sure there are no inf
            pts_w = bsxfun(@plus, p, bsxfun(@times, rho, v));   % measurements in world cartisean frame
            % Get all frontier rows and columns
            rc_frontier = obj.xy2rc(pts_w(:, frontier_ind));
            % Remove outside rows-columns
            cond1 = rc_frontier(1, :) > 0 & rc_frontier(1, :) <= size(obj.log_odds_grid, 1);
            cond2 = rc_frontier(2, :) > 0 & rc_frontier(2, :) <= size(obj.log_odds_grid, 2);
            rc_frontier = rc_frontier(:, cond1 & cond2);
            % Get frontier indices
            frontier_ind = unique(sub2ind(size(obj.log_odds_grid), rc_frontier(1, :), rc_frontier(2,:)));   
            % Get all grid rows and columns inside rectangule enclosing coverage polygon
            coverage_polygon = [p pts_w];
            rc_min = obj.xy2rc(min(coverage_polygon, [], 2)) - 1;   % -1 to have some buffer
            rc_max = obj.xy2rc(max(coverage_polygon, [], 2)) + 1;   % +1 to have some buffer
            % Clip rows and columns to valid values
            row_lims = clip([rc_min(1), rc_max(1)], 1, size(obj.log_odds_grid, 1));
            col_lims = clip([rc_min(2), rc_max(2)], 1, size(obj.log_odds_grid, 2));
            % Get all rows and cols within valid limits
            [rows, cols] = meshgrid(row_lims(1):row_lims(2), col_lims(1):col_lims(2));
            rc = [rows(:) cols(:)]';
            % Convert rows-columns to xy grid points
            xy = obj.rc2xy(rc);
            % Get polar coordinates of xy points where sensor center is the origin
            xy_ = bsxfun(@minus, xy, T_sensor_world(1:2, 4));
            [alpha_grid, rho_grid] = cart2pol(xy_(1, :), xy_(2, :));
            % Shift alpha_grid to start at first measurement (to make comparison simpler)
            alpha_grid = wrapTo2Pi(alpha_grid - atan2(v(2), v(1)));
            alpha_max = angle_diff * (length(rho) - 1);
            % Identify valid xy points
            valid_ind = alpha_grid > 0 & alpha_grid < alpha_max & rho_grid < sensor.max_range;
            % Convert alpha_grid into index of the closest in alpha
            % The shift in alpha_grid is for considering the center of interval
            rho_ind = round((alpha_grid(valid_ind) - angle_diff / 2) / angle_diff) + 1;
            % Get all valid rows-columns
            valid_rho_ind = rho_grid(valid_ind) < rho(rho_ind);
            % TODO: These two line below can be written in a better elegant way!
            rc = rc(:, valid_ind);
            rc_empty = rc(:, valid_rho_ind);
            empty_ind = unique(sub2ind(size(obj.log_odds_grid), rc_empty(1, :), rc_empty(2,:)));
            % Remove frontier indices
            empty_ind = setdiff(empty_ind, frontier_ind);
        end
        
        function rc = xy2rc(obj, xy)
            % Convert xy points into row-columns in the grid
            rc = flipud(round(bsxfun(@minus, xy, obj.ll_corner) ./ obj.res) + 1);
        end
        
        function xy = rc2xy(obj, rc)
            % Convert row-columns in the grid into xy points
            xy = bsxfun(@plus, flipud(rc - 1) * obj.res, obj.ll_corner);
        end
    end
end