% Visualization output for world
classdef Plotter2D < handle
    
    properties
        
        dims;           % Visualization area size
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        tick_length     = 0.04;
        tick_thickness  = 2;
        robot_size      = 0.025;
        robot_thickness = 2;
        circle_points   = 8;
        measurement_thickness = 0.4;
        text_size       = 10;
        
        poses = Pose2D;
        ref_world;
        
        name;
        
    end
    
    methods
        
        function obj = Plotter2D(ref)
            if nargin == 0
                return
            end
            
            obj.ref_world = ref;
            obj.dims = reshape(obj.ref_world.dims, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
            grid on;
            axis square;
            
        end
        
        function [] = Label(obj, n)
            
            obj.name = n;
            title(obj.axe, obj.name);
            
        end
        
        function [] = ReadSource(obj, src)
            
            if nargin == 1
                src = obj.ref_world;
            end
            
            if isa(src, 'World2D')
                obj.poses = src.GetPoses();
            elseif isa(src, 'Recorder2D')
                obj.poses = src.GetPoses();
            elseif isa(src, 'Pose2D')
                obj.poses = src;
            end
            
            if numel(obj.poses) ~= size(obj.colors,1)
                obj.colors = hsv(size(obj.poses,1));
                %obj.PlotLegend();
            end
            
        end
        
        function [] = ClearPlot(obj)
            
            cla(obj.axe);
            
        end
        
        function [] = PlotPoses(obj)
            
            axes(obj.axe);
            
            n = size(obj.poses,1);
            t_steps = size(obj.poses,2);
            
            hold on;
            for i = 1:n
                for t = 1:t_steps
                    p = obj.poses(i,t);
                    color = obj.colors(i,:);
                    obj.PlotPose(p, t, color, obj.robot_size);
                end
            end
            for i = 1:n
                for t = 1:t_steps
                    p = obj.poses(i,t);
                    obj.PlotLabel(p, t, num2str(i));
                end
            end
            hold off;
            
        end
        
        function PlotMeasurements(obj, measurements)
            
            axes(obj.axe);
            hold on;
            
            t_steps = size(measurements,2);
            
            for t = 1:t_steps
                m_t = measurements{t};
                n = numel(m_t);
                for i = 1:n                    
                    m = m_t{i};
                    if isa(m, 'MeasurementRangeBearing')
                        obj.PlotRangeBearing(m,t);
                    elseif isa(m, 'MeasurementRelativePose')
                        obj.PlotRelativePose(m,t);
                    end
                end
            end
            hold off;
            
        end
        
    end
    
    methods(Access = private)
        
        function PlotPose(obj, p, t, c, r)
            
            x = p.position(1);
            y = p.position(2);
            
            % Plot circle
            DrawCircle([x,y,t], r, obj.circle_points, ...
                'Color', c, 'LineWidth', obj.robot_thickness);
            
            % Plot orientation tick
            a = double(p.orientation);
            dx = obj.tick_length*cos(a);
            dy = obj.tick_length*sin(a);
            DrawLine([x, y, t], [x + dx, y + dy, t], ...
                'Color', c, 'LineWidth', obj.tick_thickness);
            
        end
        
        function PlotLabel(obj, p, t, l)
            
            x = p.position(1);
            y = p.position(2);
            text(x, y, t, l, 'FontSize', obj.text_size, ...
                'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
            
        end
        
        function PlotLegend(obj)
            
            n = numel(obj.poses);
            legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        function PlotRangeBearing(obj, m, t)
            
            id1 = m.observer_id;
            color = obj.colors(id1,:);
            p = obj.poses(id1,t);
            x = p.position(1);
            y = p.position(2);
            
            dx = m.range*cos(double(p.orientation + m.bearing));
            dy = m.range*sin(double(p.orientation + m.bearing));
            DrawLine([x, y, t], [x + dx, y + dy, t], ...
                'Color', color, 'LineWidth', obj.measurement_thickness);
            
        end
        
        function PlotRelativePose(obj, m, t)
            
            %axes(obj.axe);
            
            id1 = m.observer_id;
            id2 = m.target_id;
            color = obj.colors(id2,:);
            p = obj.poses(id1,t);
            x = p.position(1);
            y = p.position(2);
            a = double(p.orientation);
            
            R = [cos(a), -sin(a);
                sin(a), cos(a)];
            d = R*m.displacement;
            dx = d(1);
            dy = d(2);
            DrawLine([x, y, t], [x + dx, y + dy, t], ...
                'Color', color, 'LineWidth', obj.measurement_thickness);
            
            pn = Pose2D(reshape([x+dx,y+dy],1,1,2), a + m.rotation);
            obj.PlotPose(pn, t, color, obj.robot_size/2);
            
        end
        
    end
    
end