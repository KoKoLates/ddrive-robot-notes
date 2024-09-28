classdef robot
    properties
        p               % position
        v               % velocity
        wheel_speeds    % left and right wheel speeds
        wheel_radius    % wheel radius
        b               % half distance between the wheels
        dimensions      % robot dimension
        polygon         % robot polygon for visualization
    end

    methods
        % constructor
        function obj = robot(x, y)
            obj.p = [x; y; 0];
            obj.v = [0; 0; 0];
            obj.wheel_speeds = [0; 0];
            obj.wheel_radius = 5;
            obj.b = 25;

            % robot dimension
            obj.dimensions = [
                -obj.b, -obj.b, 1;
                 obj.b, -obj.b, 1;
                 obj.b,  obj.b, 1;
                -obj.b,  obj.b, 1
            ];

            % update polygon for visualization
            obj.update_polygon();
        end

        function obj = update(obj, dt)
            obj.wheel_speeds(obj.wheel_speeds > 3) = 3;
            obj.wheel_speeds(obj.wheel_speeds < -3) = -3;
            
            % calculate forward velocity from wheel speeds
            obj = obj.forward();
            
            % update position using velocity
            a = eye(3);
            c = [
                sin(obj.p(3) + pi/2) * dt, 0;
                cos(obj.p(3) + pi/2) * dt, 0;
                0, dt
            ];
             
            vel = [obj.v(1); obj.v(3)];
            obj.p = a * obj.p + c * vel;
            
            % update wheel speeds based on new velocity
            obj = obj.inverse();
        end

        function polygon = points(obj)
            obj.update_polygon();
            polygon = obj.polygon;
        end

        function [p, v] = state(obj)
            p = obj.p;
            v = obj.v;
        end

        function obj = set_robot_speed(obj, linear, angular)
            obj.v = [linear; 0; angular];
            obj = obj.inverse();
        end

        function obj = set_wheel_speed(obj, left, right)
            obj.wheel_speeds = [left; right];
            obj = obj.forward();
        end

        function obj = forward(obj)
            mat = [
                obj.wheel_radius / 2, obj.wheel_radius / 2;
                0, 0;
                obj.wheel_radius / (obj.b * 2), -obj.wheel_radius / (obj.b * 2)
            ];
            obj.v = mat * obj.wheel_speeds;
        end

        function obj = inverse(obj)
            mat = [
                1 / obj.wheel_radius, 0, obj.b / obj.wheel_radius;
                1 / obj.wheel_radius, 0, -obj.b / obj.wheel_radius
            ];
            obj.wheel_speeds = mat * obj.v;
        end

        function obj = update_polygon(obj)
            mat = [
                 cos(obj.p(3)), sin(obj.p(3)), obj.p(1);
                -sin(obj.p(3)), cos(obj.p(3)), obj.p(2);
                0, 0, 1;
            ];
            obj.polygon = round((obj.dimensions * mat')');
        end
    end
end