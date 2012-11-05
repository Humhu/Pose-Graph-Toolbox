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
            axes(obj.axe);
            
            n = numel(obj.poses);
            
            for i = 1:n
                p = obj.poses(i);
                color = obj.colors(i,:);
                obj.PlotPose(p, color, obj.robot_size);
                obj.PlotLabel(p, num2str(i));
            end
            
            %legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        function PlotMeasurements(obj, measurements)
            
            for i = 1:numel(measurements)
                m = measurements{i};
                if isa(m, 'MeasurementRangeBearing')
                    obj.PlotRangeBearing(m);
                elseif isa(m, 'MeasurementRelativePose')
                    obj.PlotRelativePose(m);
                end
            end
            
        end
        
    end
    
    methods(Access = private)
        
        function PlotPose(obj, p, c, r)
            
            x = p.position(1);
            y = p.position(2);
            
            % Plot circle
            viscircles([x, y], r, 'linewidth', obj.robot_thickness, ...
                'edgecolor', c);
            
            % Plot orientation tick
            t = double(p.orientation);
            dx = obj.tick_length*cos(t);
            dy = obj.tick_length*sin(t);
            line_x = [x, x + dx];
            line_y = [y, y + dy];
            line(line_x, line_y, 'linewidth', obj.tick_thickness, 'color', c);
            
        end
        
        function PlotLabel(obj, p, l)
                        
            x = p.position(1);
            y = p.position(2);
            text(x, y, l);
            
        end
        
        function PlotRangeBearing(obj, m)                        
            
            id1 = m.observer_id;
            color = obj.colors(id1,:);
            p = obj.poses(id1);
            x = p.position(1);
            y = p.position(2);
            t = p.orientation;
            
            r = m.range;
            a = m.bearing;
            
            dx = r*cos(double(t + a));
            dy = r*sin(double(t + a));
            line_x = [x, x + dx];
            line_y = [y, y + dy];
            line(line_x, line_y, 'linewidth', obj.measurement_thickness, 'color', color);
            
        end
        
        function PlotRelativePose(obj, m)
            
            %axes(obj.axe);
            
            id1 = m.observer_id;
            id2 = m.target_id;
            color = obj.colors(id2,:);
            p = obj.poses(id1);
            x = p.position(1);
            y = p.position(2);
            t = double(p.orientation);
            
            R = [cos(t), -sin(t);
                sin(t), cos(t)];
            d = R*m.displacement;
            dx = d(1);
            dy = d(2);
            line_x = [x, x + dx];
            line_y = [y, y + dy];
            line(line_x, line_y, 'linewidth', obj.measurement_thickness, 'color', color);
            
            pn = Pose2D(reshape([x+dx,y+dy],1,1,2), t + m.rotation);
            obj.PlotPose(pn, color, obj.robot_size/2);            
            
        end
        
    end
    
end