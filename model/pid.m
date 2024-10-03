classdef pid
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        l_Kp, l_Kd, l_Ki    % linear PID parameters
        a_Kp, a_Kd, a_Ki    % angular PID parameters

    end

    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end