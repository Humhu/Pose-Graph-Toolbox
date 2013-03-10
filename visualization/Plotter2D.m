% Visualization output for world
% TODO: Figure out how to support plotting measurements across different times
classdef Plotter2D < handle
    
    properties
        
        dims;           % Visualization area size
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        name;           % Figure title
        
        z_scale = 1; % Height scaling
        
        % Graphics handles
        line_handles;
        label_handles;
        robot_handles;
        
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
            
            % Assign graphics groups
            obj.line_handles = hggroup('parent', obj.axe);
            obj.label_handles = hggroup('parent', obj.axe);
            obj.robot_handles = hggroup('parent', obj.axe);
            
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
            
            %cla(obj.axe);
            delete(obj.line_handles);
            delete(obj.label_handles);
            delete(obj.robot_handles);
            refresh(obj.fig);
            obj.line_handles = hggroup('parent', obj.axe);
            obj.label_handles = hggroup('parent', obj.axe);
            obj.robot_handles = hggroup('parent', obj.axe);
            
        end
        
        % Have this plotter use another plotter's figure/axes
        function Link(obj, other)
           
            close(obj.fig);
            obj.fig = other.fig;
            obj.axe = other.axe;
            obj.line_handles = hggroup('parent', obj.axe);
            obj.label_handles = hggroup('parent', obj.axe);
            obj.robot_handles = hggroup('parent', obj.axe);
            
        end
        
        function ShowLines(obj)           
            set(obj.line_handles, 'visible', 'on');
        end
        
        function HideLines(obj)
            set(obj.line_handles, 'visible', 'off');
        end
        
        function ShowLabels(obj)
            set(obj.label_handles, 'visible', 'on');
        end
        
        function HideLabels(obj)
           set(obj.label_handles, 'visible', 'off'); 
        end
        
        function ShowRobots(obj)
           set(obj.robot_handles, 'visible', 'on');
        end
        
        function HideRobots(obj)
           set(obj.robot_handles, 'visible', 'off');
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
            h = hggroup;
            obj.PlotPolygon([x,y,t], s*obj.robot_size, obj.circle_points, ...
                {'Color', c, 'LineWidth', obj.robot_thickness, 'parent', h});
            
            % Plot orientation tick
            a = p(3);
            dx = s*obj.tick_length*cos(a);
            dy = s*obj.tick_length*sin(a);            
            obj.PlotLine([x, y, t], [x + dx, y + dy, t], ...
                {'Color', c, 'LineWidth', obj.tick_thickness, 'parent', h});
            set(h, 'parent', obj.robot_handles);
            
        end
        
        function PlotLabel(obj, p, t, l)
            
            x = p(1);
            y = p(2);
            t = obj.z_scale*t;
            text(x, y, t, l, 'FontSize', obj.text_size, ...
                'FontWeight', 'bold', 'HorizontalAlignment', 'Center', ...
                'parent', obj.label_handles);
            
        end
        
        function [h] = PlotLegend(obj)
            
            n = size(obj.colors, 1);
            h = legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        % Plots a covariance ellipse
        function [h] = PlotEllipse(obj, p, t, cov)
            
            t = obj.z_scale*t;
            cov = cov(1:2,1:2);
            a = linspace(0, 2*pi, obj.ellipse_points);
            c = [cos(a); sin(a)];
            [V, D] = eig(cov);
            D = 3*sqrt(D);
            c = V*D*c;
            c = bsxfun(@plus, c, p(1:2));
            c = [c; t*ones(1, obj.ellipse_points)];
            h = plot3(obj.axe, c(1,:), c(2,:), c(3,:), '-b', 'LineWidth', obj.ellipse_thickness);
            
        end
        
        % Plots a n-vertex regular polygon
        function PlotPolygon(obj, center, r, n, params)
            
            if nargin == 4
                params = {};
            end
            
            center(3) = center(3)*obj.z_scale;
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
            z = [start(3), finish(3)]*obj.z_scale;
            if ~any(strcmp(params, 'parent'))
               params = [params, 'parent', obj.line_handles];
            end
            plot3(obj.axe, x, y, z, params{:});            
            
        end
        
    end
    
end