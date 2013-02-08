% Visualization output for world
% TODO: Figure out how to support plotting measurements across different times
classdef Plotter2D < handle
    
    properties
        
        dims;           % Visualization area size
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        name;           % Figure title
        
        % Parameters
        tick_length     = 0.04;
        tick_thickness  = 2;
        robot_size      = 0.025;
        robot_thickness = 2;
        circle_points   = 8;
        ellipse_points = 16;
        ellipse_thickness = 0.4;
        measurement_thickness = 0.4;
        text_size       = 10;
        
    end
    
    methods
        
        function obj = Plotter2D(size)
            if nargin == 0
                return
            end
            
            obj.dims = reshape(size, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
            grid off;
            axis vis3d;                        
            hold(obj.axe, 'on');           
            
        end
        
        % Set this plotter's plot title
        function [] = Label(obj, n)
            
            obj.name = n;
            title(obj.axe, obj.name);
            
        end
        
        % Set this plotter's color map, or default to rainbow
        function [] = SetColors(obj, cmap)
            
            if numel(cmap) == 1
                obj.colors = hsv(cmap);
            else
                obj.colors = cmap;
            end
            
        end
        
        % Clear this plotter's axes
        function [] = Clear(obj)
            
            cla(obj.axe);
            
        end
        
    end
    
    methods(Access = protected)
        
        % Plot a robot marker
        % Input:
        %   p - Pose of robot
        %   t - Time
        %   c - Color
        %   s - Size
        function PlotRobot(obj, p, t, c, s)
            
            x = p(1);
            y = p(2);
            
            % Plot circle
            obj.PlotPolygon([x,y,t], s*obj.robot_size, obj.circle_points, ...
                {'Color', c, 'LineWidth', obj.robot_thickness});
            
            % Plot orientation tick
            a = p(3);
            dx = s*obj.tick_length*cos(a);
            dy = s*obj.tick_length*sin(a);
            obj.PlotLine([x, y, t], [x + dx, y + dy, t], ...
                {'Color', c, 'LineWidth', obj.tick_thickness});
            
        end
        
        function PlotLabel(obj, p, t, l)
            
            x = p(1);
            y = p(2);
            text(x, y, t, l, 'FontSize', obj.text_size, ...
                'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
            
        end
        
        function PlotLegend(obj)
            
            n = size(obj.colors, 1);
            legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        function PlotEllipse(obj, p, t, cov)
            
            cov = cov(1:2,1:2);
            a = linspace(0, 2*pi, obj.ellipse_points);
            c = [cos(a); sin(a)];
            [V, D] = eig(cov);
            D = 3*sqrt(D);
            c = V*D*c;
            c = bsxfun(@plus, c, p(1:2));
            c = [c; t*ones(1, obj.ellipse_points)];
            plot3(c(1,:), c(2,:), c(3,:), '-b', 'LineWidth', obj.ellipse_thickness);
            
        end
        
        % Plots a n-vertex regular polygon
        function PlotPolygon(obj, center, r, n, params)
            
            if nargin == 4
                params = {};
            end
            
            t = linspace(0, 2*pi, n+1);
            x = center(1) + r*cos(t);
            y = center(2) + r*sin(t);
            z = center(3)*ones(length(t),1);
            plot3(obj.axe, x, y, z, params{:});
            
        end
        
        function PlotLine(obj, start, finish, params)
            
            if nargin == 3
                params = {};
            end
            
            x = [start(1), finish(1)];
            y = [start(2), finish(2)];
            z = [start(3), finish(3)];
            plot3(obj.axe, x, y, z, params{:});
            
        end
        
    end
    
end