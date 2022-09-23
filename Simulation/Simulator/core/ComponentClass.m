classdef ComponentClass < dynamicprops
    properties (SetAccess = private)
        % General
        category
        type
        % For relations in RobotClass
        transformation
        label
        % For plotting
        shape
    end
    properties (SetAccess = private, GetAccess = private)
        % This list contains those accepted categories in the constructor.
        % Add more if necessary.
        valid_categories = {'actuator', 'sensor', 'wheel'};
    end
    methods
        % Constructor
        function obj = ComponentClass(varargin)
            % Decode input parameters
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
            if ~any(strcmp(obj.valid_categories, json_struct.category))
                error(['Invalid category: `' json_struct.category '`!'])
            end
            % add all available json-struct fields
            field_names = fieldnames(json_struct);
            for i=1:length(field_names)
                field_name = field_names{i};
                if strcmp(field_name, 'shape')
                    % Convert cell array to matrix in-place
                    t1 = fieldnames(json_struct.(field_name));
                    for j=1:length(t1)
                        t2 = json_struct.(field_name).(t1{j});
                        if isa(t2, 'cell')
                            t2 = cell2mat(vertcat(t2));
                            % Sanity check
                            if length(t2) ~=2
                                error(['`' t1{j} '` must have length of 2!'])
                            end
                            json_struct.(field_name).(t1{j}) = t2;
                        end
                    end
                end
                % Convert cell array to double array
                t1 = json_struct.(field_name);
                if isa(t1, 'cell')
                    t1 = cell2mat(vertcat(t1));
                end
                % update property (create new if necessary)
                obj.set_property(field_name, t1)
            end
        end
        
        function set_transformation(obj, T, mode)
            if nargin < 3
                mode = '';       % if T is invalid return error message
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
        
        function h = plot(obj, T)
            % T is in the world reference frame...
            
            if nargin < 2
                T = eye(4);
            end
            % Sanity checks
            if isempty(obj.shape)
                error('Component: `shape` is not available!')
            end
            h = plot_shape(obj.shape, 'transformation', T);
        end
        
    end
end