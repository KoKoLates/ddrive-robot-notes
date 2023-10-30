classdef robot
    % A class definition for the differential drive robot
    
    properties
        pose = [0, 0, 0]; % [x, y, theta]
        wheel_diameter = 0.25;
        track_width = 0.5;

        % velocity attributes
        velocity = 20;
        wheel_rate = [
            abs(sin(0: 0.01: 2 * pi)).*velocity; 
            abs(cos(0: 0.01: 2 * pi)).*velocity;
        ];
        dt = 0.01; % time between iterations in seconds
    end
    
    methods
        function [new_pose] = kinematics( ...
                current_pose, linear_velocity, rotation_velocity, dt)
            radius = abs(linear_velocity / rotation_velocity);

            theta = current_pose(3);
            % compute sin and cos at initial pose
            s = sin(theta);
            c = cos(theta);

            % compute sin and cos at final pose
            s_th = sin(theta + rotation_velocity * dt);
            c_th = cos(theta + rotation_velocity * dt);


            current_pose(1) = current_pose(1) + (linear_velocity * c_th) * dt;
            % ...
            
            new_pose = current_pose;
        end
    end
end

