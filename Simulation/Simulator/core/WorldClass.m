classdef WorldClass < dynamicprops
    properties (SetAccess = private)
        dimension       % Demension of the world (2D or 3D)
        shapes          % List of shape structures in the world
        
        primitives      % Struct with geometric primitives as fields
                        % This property is used for sensor/world interaction
    end
    methods
        function obj = WorldClass(varargin)
            % Constructor
            % Supports mat-file and json-file format
            parameters = varargin_to_parameters(varargin);
            mode = 'empty';
            if isfield(parameters, 'fname')
                [~, ~, ext] = fileparts(parameters.fname);
                if strcmp(ext, '.mat')
                    W = load(parameters.fname);
                    mode = 'mat_file';
                elseif strcmp(ext, '.json')
                    json_struct = read_json_file(parameters.fname);
                    mode = 'json_struct';
                else
                    error('Invalid fname!')
                end
            end
            % Construct based on mode
            if strcmp(mode, 'empty')
                obj.dimension = 2;
                obj.set_property('description', 'Empty 2D world');
            elseif strcmp(mode, 'mat_file')
                tmp = fieldnames(W);
                obj = W.(tmp{1});
            elseif strcmp(mode, 'json_struct')
                % Sanity checks
                if ~strcmp(json_struct.category, 'world')
                    error(['Invalid category: `' json_struct.category '`!' ])
                end
                if ~isfield(json_struct, 'dimension')
                    error('JSON-struct does not have `dimension` field!')
                end
                obj.dimension = json_struct.dimension;
                % Add shapes if exist
                if isfield(json_struct, 'shapes')
                    for i = 1:length(json_struct.shapes)
                        obj.add_shape(obj.dimension, json_struct.shapes{i})
                    end
                end
                % Update `primitives` property
                obj.update_primitives()
            
                % Add additional properties if necessary
                basic_fields = {'category', 'dimension', 'shapes'};
                additional_fields = setdiff(fieldnames(json_struct), basic_fields);
                for i = 1:length(additional_fields)
                    property_name = additional_fields{i};
                    property_values = json_struct.(property_name);
                    % convert cell array to mat if necessary
                    if isa(property_values, 'cell')
                        property_values = cell2mat(vertcat(property_values));
                    end
                    obj.set_property(property_name, property_values)
                end
            end
        end
        
        function set_property(obj, property_name, property_value)
            % Try/catch is faster that isprop
            try
                obj.(property_name) = property_value;
            catch
                % Create property and add it to object
                P = obj.addprop(property_name);	% dynamic property
                P.SetAccess = 'private';        % make it private
                obj.(property_name) = property_value;
            end
        end
        
        function add_shape(obj, dimension, shape_struct)
            if dimension == 2
                shape_ind = length(obj.shapes) + 1;     % index in property
                % Add General fields
                obj.shapes{shape_ind}.type = shape_struct.type;
                if isfield(shape_struct, 'color')
                    obj.shapes{shape_ind}.color = shape_struct.color;
                else
                    try
                        obj.shapes{shape_ind}.color = obj.color;
                    catch
                        obj.shapes{shape_ind}.color = 'black';
                    end
                end
                % Add type-specific fields
                if strcmp(shape_struct.type, 'point')
                    if isa(shape_struct.coordinates, 'cell')
                        shape_struct.coordinates = cell2mat(shape_struct.coordinates);
                    end
                    obj.shapes{shape_ind}.coordinates = shape_struct.coordinates(:);
                elseif strcmp(shape_struct.type, 'circle')
                    if isa(shape_struct.center, 'cell')
                        shape_struct.center = cell2mat(shape_struct.center);
                    end
                    obj.shapes{shape_ind}.center = shape_struct.center(:);
                    obj.shapes{shape_ind}.diameter = shape_struct.diameter;
                elseif strcmp(shape_struct.type, 'polygon')
                    if isa(shape_struct.x_points, 'cell')
                        shape_struct.x_points = cell2mat(shape_struct.x_points);
                        shape_struct.y_points = cell2mat(shape_struct.y_points);
                    end
                    obj.shapes{shape_ind}.vertices = [shape_struct.x_points;...
                                                      shape_struct.y_points];
                else
                    error(['Shape type: ' shape_struct.type ' not implemented!'])
                end
            else
                error(['Efficient handling of shapes in ' num2str(dimension) '-D is not implemented!'])
            end
        end
        
        function h = plot(obj)
            h = [];
            hold on
            for i = 1:length(obj.shapes)
                h = [h plot_shape(obj.shapes{i}, 'alpha', 1)];
            end
            %hold off
            box on
            axis equal
            try
                axis(reshape([obj.min_corner; obj.max_corner], 1, []))
            catch
                axis([-10 10 -5 5])
            end
        end
        
        function update_primitives(obj)
            % Convert world shapes into lists of primitive shapes to be
            % used for interaction with exterioceptive sensors.
            % Primitive shapes in 2D include {points, circles, line segments}
            
            % Reset primitives
            obj.primitives = [];
            for i = 1:length(obj.shapes)
                shape_type = obj.shapes{i}.type;
                if strcmp(shape_type, 'point')
                    if ~isfield(obj.primitives, 'points')
                        obj.primitives.points = [];
                    end
                    % Append coordinates
                    obj.primitives.points = [obj.primitives.points...
                                             obj.shapes{i}.coordinates];
                elseif strcmp(shape_type, 'circle')
                    if ~isfield(obj.primitives, 'circles')
                        obj.primitives.circles = [];
                    end
                    % Convert to column vector [x, y, r]', then append
                    t1 = [obj.shapes{i}.center; obj.shapes{i}.diameter/2];
                    obj.primitives.circles = [obj.primitives.circles, t1];
                elseif strcmp(shape_type, 'polygon')
                    if ~isfield(obj.primitives, 'line_segments')
                        obj.primitives.line_segments = [];
                    end
                    % Convert to list of line segments in point-direction form
                    p1 = obj.shapes{i}.vertices;
                    p2 = circshift(p1, -1, 2);
                    % Append line segments
                    obj.primitives.line_segments = [obj.primitives.line_segments...
                                                    [p1; p2 - p1]];
                else
                    error(['Shape type:' shape_type ' not implemented!'])
                end

            end
        end
        
        function save(obj, fname)
            % Save world as mat-file
            
            % Assert file name extension as mat
            [filepath, ~, ext] = fileparts(fname);
            if ~strcmp(ext, '.mat')
                fname = [fname '.mat'];
            end
            try
                save(fname, 'obj')
            catch
                error(['Could not save world at: ' fullfile(filepath, fname)])
            end
        end
        
        function h = edit(obj, ax, path_width)
            % Edit world graphically
            % Return array of plot handles.
            
            % Default value
            if nargin < 3
                path_width = 0.02;      % this value is for line following task. TODO: move somewhere else
                if nargin < 2
                    % Create figure if necessary
                    figure()
                    ax = gca;
                end
            end
            % Plot current world
            axes(ax)
            %cla
            h = obj.plot();
            % Disable all top toolbar to avoid saving before updating primitives
            defaultToolbar = findall(gcf,'Type','uitoolbar');
            chld = defaultToolbar.Children;
            for i = 1:length(chld)
                set(chld(i), 'Enable', 'off')
            end
            % This variable is used in the loop below
            edit_mode = [];
            % Add toolbar
            tb = uitoolbar();
            % Add push tools and callbacks
            pt_add_polygon = uipushtool(tb,'CData',imread('add_polygon.png'),...
                'TooltipString','Add polygon','Separator','on',...
                'ClickedCallback', @addPolygonClicked);
            
            function addPolygonClicked(src, event)
                title({'Add polygon:    Use mouse left click to add vertices.';...
                    'Use SPACE to close polygon.'})
                edit_mode = 'polygon';
            end
            
            pt_add_circle = uipushtool(tb,'CData',imread('add_circle.png'),...
                'TooltipString','Add circle','Separator','on',...
                'ClickedCallback', @addCircleClicked);
            
            function addCircleClicked(src, event)
                title({'Add circle:    Use mouse left click to add cirle center.';...
                    'Then another left click to add radius.'})
                edit_mode = 'circle';
            end
            
            pt_add_point = uipushtool(tb,'CData',imread('add_point.png'),...
                'TooltipString','Add point','Separator','on',...
                'ClickedCallback', @addPointClicked);
            
            function addPointClicked(src, event)
                title({'Add point:    Use mouse left click to add point feature.';' '})
                edit_mode = 'point';
            end
            
            pt_add_path = uipushtool(tb,'CData',imread('add_path.png'),...
                'TooltipString','Add path','Separator','on',...
                'ClickedCallback', @addPathClicked);
            
            function addPathClicked(src, event)
                title({'Add path:    Use mouse left click to add vertices.';...
                    'Use SPACE to finish.'})
                edit_mode = 'path';
            end
            
            pt_delete_shape = uipushtool(tb,'CData',imread('delete_shape.png'),...
                'TooltipString','Delete shape','Separator','on',...
                'ClickedCallback', @deleteShapeClicked);
            
            function deleteShapeClicked(src, event)
                title({'Delete shape:    Use mouse left click inside shape to delete.';' '})
                edit_mode = 'delete';
            end
            
            % Initialize state machine parameters for circle, polygon, path
            first_time = true;
            old_mode = [];
            h_tmp = [];         % used for temporary plots
            while true
                % Check if figure is not closed
                if ~ishandle(ax)
                    break
                end
                % Instructions at the beginning
                if isempty(edit_mode)
                    title({'Select an option from the second toolbar.';...
                        'ESC to exit the editor.'})
                end
                % Get user input from interacting with graph
                try
                    [xt, yt, button] = ginput(1);
                catch
                    break
                end
                % Stop if ESC is pressed
                if button == 27
                    delete(h_tmp)
                    break
                end
                % Reset state machine if necessary
                if ~strcmp(old_mode, edit_mode)
                    old_mode = edit_mode;
                    first_time = true;
                    shape = [];
                    delete(h_tmp)
                end
                % Consider different edit modes
                if strcmp(edit_mode, 'point')
                    % add point to `shapes` property
                    shape.type = 'point';
                    shape.coordinates = [xt, yt];
                    obj.add_shape(2, shape)
                    % plot and add to axes
                    if ~ishold,     hold on,    end
                    h = [h, plot_shape(obj.shapes{end}, 'alpha', 1)];
                elseif strcmp(edit_mode, 'circle')
                    if first_time
                        center = [xt; yt];
                        first_time = false;
                        % plot radius
                        if ~ishold,     hold on,    end
                        h_tmp = plot(xt, yt, 'r.', 'MarkerSize', 10);
                    else
                        radius = distance_two_points(center, [xt; yt]);
                        % Add circle to `shapes` property
                        shape.type = 'circle';
                        shape.center = center;
                        shape.diameter = 2 * radius;
                        obj.add_shape(2, shape)
                        % plot and add to axes
                        if ~ishold,     hold on,    end
                        h = [h, plot_shape(obj.shapes{end}, 'alpha', 1)];
                        % Reset
                        first_time = true;
                        delete(h_tmp)
                    end
                elseif strcmp(edit_mode, 'polygon') || strcmp(edit_mode, 'path')
                    % Add to `shapes` property if SPACE bar is pressed
                    if button == 32
                        try
                            if strcmp(edit_mode, 'path')
                                % Construct polygon from path
                                pts_tmp = path_to_simple_polygon(pts_tmp, path_width);
                            end
                            % Only add polygon if it has at least 3 points
                            if size(pts_tmp, 2) > 2
                                shape.type = 'polygon';
                                shape.x_points = pts_tmp(1, :);
                                shape.y_points = pts_tmp(2, :);
                                obj.add_shape(2, shape)
                                % plot and add to axes
                                if ~ishold,     hold on,    end
                                h = [h, plot_shape(obj.shapes{end}, 'alpha', 1)];
                            end
                        catch
                        end
                        % Reset
                        old_mode = [];
                        delete(h_tmp)
                        continue
                    end
                    % Construct polygon
                    if first_time
                        pts_tmp = [xt; yt];
                        first_time = false;
                        % plot first point
                        if ~ishold,     hold on,    end
                        h_tmp = plot(xt, yt, 'r.', 'MarkerSize', 10);
                    else
                        pts_tmp = [pts_tmp, [xt; yt]];
                        % plot lines
                        delete(h_tmp)
                        h_tmp = plot(pts_tmp(1, :), pts_tmp(2, :), 'r-');
                    end
                elseif strcmp(edit_mode, 'delete')
                    % loop over all shapes and delete the FIRST match
                    to_delete = false;
                    for ind = 1:length(obj.shapes)
                        shape = obj.shapes{ind};
                        if strcmp(shape.type, 'point')
                            % Threshold is 2% of zoomed area
                            point_dist_threshold = max(diff([get(ax, 'xlim'); get(ax, 'ylim')], 1, 2)) * 0.02;
                            if distance_two_points([xt; yt], shape.coordinates) < point_dist_threshold
                                to_delete = true;
                                break
                            end
                        elseif strcmp(shape.type, 'polygon')
                            if is_point_inside_simple_polygon([xt; yt], shape.vertices)
                                to_delete = true;
                                break
                            end
                        elseif strcmp(shape.type, 'circle')
                            if (shape.diameter / 2) > distance_two_points([xt; yt], shape.center)
                                to_delete = true;
                                break
                            end
                        end
                    end
                    if to_delete
                        obj.shapes(ind) = [];
                        delete(h(ind))
                        h(ind) = [];
                    end
                end
            end
            % Enable all top toolbar
            for i = 1:length(chld)
                set(chld(i), 'Enable', 'on')
            end
            % Update `primitives` property
            obj.update_primitives()
            % Update axes
            if ishandle(ax)
                title('')       % Clear title
                delete(tb)      % Delete edit toolbar
            end
        end
    end
end