classdef application < handle
    properties
        w           % width of the canvas
        h           % height if the canvas
        name        % name of the application shown on window
        color       % background color
        
        canvas      % canvas 
        figure      % figure handle
        axes        % axes handle
    end

    methods
        function obj = application(w, h, name, func, color)
            if nargin < 5
                color = [255, 255, 255];
            end
            obj.w = w;
            obj.h = h;
            obj.name = name;
            obj.color = color / 255;

            % initialize figure and axes
            obj.figure = figure(Name', obj.name, 'NumberTitle', 'off', ...
                                'WindowButtonDownFcn', func, 'Position', [100, 100, w, h]);
            obj.axes = axes('Parent', obj.figure);
            axis(obj.axes, [0 w 0 h]);
            axis off;
            hold on;

            % initialize the canvas
            obj.clean();
        end

        function clean(obj)
            cla(obj.axes);
            fill(obj.axes, [0, obj.w, obj.w, 0], [0, 0, obj.h, obj.h], obj.color, 'EdgeColor', 'none');
        end

        function plot(obj, points, color, thickness)
            if nargin < 3
                color = [255, 0, 0]; % Default red color
            end
            if nargin < 4
                thickness = 2; % Default thickness
            end
            color = color / 255; % Normalize the color
            
            for i = 1:size(points, 1) - 1
                plot(obj.axes, [points(i, 1), points(i + 1, 1)], ...
                     [points(i, 2), points(i + 1, 2)], 'Color', color, 'LineWidth', thickness);
            end
            % Close the polygon if necessary
            plot(obj.axes, [points(1, 1), points(end, 1)], ...
                 [points(1, 2), points(end, 2)], 'Color', color, 'LineWidth', thickness);
        end

        function plot_path(obj, points, color, thickness, dotted)
            if nargin < 3
                color = [255, 0, 0]; % Default red
            end
            if nargin < 4
                thickness = 2; % Default thickness
            end
            if nargin < 5
                dotted = false; % Default is solid line
            end
            color = color / 255; % Normalize the color
            
            if dotted
                for i = 1:2:length(points) - 1
                    plot(obj.axes, points(i, 1), points(i, 2), 'o', 'Color', color, 'MarkerSize', thickness);
                end
            else
                for i = 1:length(points) - 1
                    plot(obj.axes, [points(i, 1), points(i + 1, 1)], ...
                         [points(i, 2), points(i + 1, 2)], 'Color', color, 'LineWidth', thickness);
                    plot(obj.axes, points(i, 1), points(i, 2), 'ko', 'MarkerSize', 3); % Small circles at points
                end
                plot(obj.axes, points(end, 1), points(end, 2), 'ko', 'MarkerSize', 3); % Circle at the end
            end
        end

        function text(obj, txt, color, font_scale, org)
            if nargin < 3
                color = [255, 0, 0];
            end
            if nargin < 5
                font_scale = 12;
            end
            if nargin < 6
                org = [100, 50];
            end
            color = color / 255; % normalize color
            
            text(obj.axes, org(1), org(2), txt, 'Color', color, ...
                 'FontSize', font_scale, 'FontWeight', 'Bold', 'Interpreter', 'none');
        end

        function show(obj)
            drawnow;
        end
    end
end