% Visualization output for world
classdef Plotter2D < handle
    
    properties
        
        dims;           % Visualization area size
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        tick_length     = 0.025;
        tick_thickness  = 2;
        robot_size      = 0.025;
        robot_thickness = 2;
        measurement_thickness = 0.4;

        poses = Pose2D;
        
        name;
        
    end
    
    methods
        
        function obj = Plotter2D(world_size)
            if nargin == 0
                return
            end
            
            obj.dims = reshape(world_size, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
            
        end
        
        function [] = Label(obj, n)
            
            obj.name = n;
            title(obj.axe, obj.name);
            
        end
        
        function [] = ReadSource(obj, src)
           
            if isa(src, 'World2D')
                obj.poses = src.GetPoses();
            elseif isa(src, 'Pose2D')
                obj.poses = src;
            end
            
            obj.colors = hsv(numel(obj.poses));
            
        end        
        
        function [] = PlotPoses(obj)
            
            cla(obj.axe);
           % figure(obj.fig);
            axes(obj.axe);
            
            n = numel(obj.poses);
            
            for i = 1:n
                
                p = obj.poses(i);
                x = p.position(1);
                y = p.position(2);
                color = obj.colors(i,:);
                
                % Plot circle
                viscircles([x, y], obj.robot_size, 'linewidth', obj.robot_thickness, ...
                    'edgecolor', color);
                
                % Plot orientation tick
                dx = obj.tick_length*cos(double(p.orientation));
                dy = obj.tick_length*sin(double(p.orientation));
                line_x = [x, x + dx];
                line_y = [y, y + dy];
                line(line_x, line_y, 'linewidth', obj.tick_thickness, 'color', color);
                % Label robot
                text(x, y, num2str(i));
                
            end
            
        end
        
        function PlotMeasurements(obj, measurements)
            
            %figure(obj.fig);
            axes(obj.axe);
            
            num_meas = numel(measurements);
            
            for i = 1:num_meas
                
                m = measurements{i};
                %id1 = ids(1,i);
                id1 = m.observer_id;
                
                color = obj.colors(id1,:);
                
                p = obj.poses(id1);
                x = p.position(1);
                y = p.position(2);
                t = p.orientation;
                
                r = m.range;
                a = m.bearing;
                
                % Plot orientation tick
                dx = r*cos(double(t + a));
                dy = r*sin(double(t + a));
                line_x = [x, x + dx];
                line_y = [y, y + dy];
                line(line_x, line_y, 'linewidth', obj.measurement_thickness, 'color', color);
                
            end
            
        end
        
    end
    
end