% Visualization output for world
% TODO: Figure out how to support plotting measurements across different times
classdef Plotter2D < handle
    
    properties
        
        dims;           % Visualization area size
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        name;           % Figure title
        history;        % Data being shown        
        
        % Parameters
        tick_length     = 0.04;
        tick_thickness  = 2;
        robot_size      = 0.025;
        robot_thickness = 2;
        circle_points   = 8;
        measurement_thickness = 0.4;
        text_size       = 10;        
        
    end
    
    methods
        
        function obj = Plotter2D(size)
            if nargin == 0
                return
            end
            
            obj.history = Sequence2D(100); % TODO: Un-hardcode
            
            obj.dims = reshape(size, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
            grid on;
            %axis vis3d;
            xlabel('x');
            ylabel('y');
            zlabel('t');
            
        end
        
        function [] = Label(obj, n)
            
            obj.name = n;
            title(obj.axe, obj.name);
            
        end
        
        function [] = SetColors(obj, cmap)
            
            if numel(cmap) == 1
                obj.colors = hsv(cmap);
            else
                obj.colors = cmap;
            end
            
        end
        
        
        function [] = Clear(obj)
            
            cla(obj.axe);                        
            obj.history.Clear();
            
        end                
        
        function [] = PlotSequence(obj, seq)
            
            T = seq.GetLength();
            for t = 1:T
               obj.PlotState(seq.states(t)); 
            end
            
        end
        
        function [] = PlotState(obj, state)   
            
            obj.history.Write(state);
            obj.PlotPoses(state);
            obj.PlotMeasurements(state);
            
        end
        
        function [] = PlotPoses(obj, state)
            
            axes(obj.axe);
            
            n = size(state.poses,1);
            
            hold on;
            for i = 1:n
                p = state.poses(i);
                color = obj.colors(i,:);
                obj.PlotPose(p, state.time, color, 1.0);
            end
            for i = 1:n
                p = state.poses(i);
                obj.PlotLabel(p, state.time, num2str(i));
            end
            hold off;
            
        end
        
        function PlotMeasurements(obj, state)
            
            axes(obj.axe);
            hold on;
            
            n = numel(state.measurements);
            for i = 1:n
                m = state.measurements{i};
                if isa(m, 'MeasurementRangeBearing')
                    obj.PlotRangeBearing(state, m);
                elseif isa(m, 'MeasurementRelativePose')
                    obj.PlotRelativePose(m);
                    obj.PlotRelativePose(m.ToInverse());
                end
            end
            hold off;
            
        end
        
    end
    
    methods(Access = private)
        
        function PlotPose(obj, p, t, c, s)
            
            x = p.position(1);
            y = p.position(2);
            
            % Plot circle
            DrawCircle([x,y,t], s*obj.robot_size, obj.circle_points, ...
                'Color', c, 'LineWidth', obj.robot_thickness);
            
            % Plot orientation tick
            a = double(p.orientation);
            dx = s*obj.tick_length*cos(a);
            dy = s*obj.tick_length*sin(a);
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
            
            n = size(obj.colors, 1);
            legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        % TODO: Update for consistency!
        function PlotRangeBearing(obj, fs, m)
            
            id1 = m.observer_id;
            color = obj.colors(id1,:);
            p = fs.states(m.observer_time).poses(id1);
            x = p.position(1);
            y = p.position(2);
            
            dx = m.range*cos(double(p.orientation + m.bearing));
            dy = m.range*sin(double(p.orientation + m.bearing));
            DrawLine([x, y, m.observer_time], [x + dx, y + dy, m.target_time], ...
                'Color', color, 'LineWidth', obj.measurement_thickness);
            
        end
        
        function PlotRelativePose(obj, m)
            
            color = obj.colors(m.target_id,:);
            p = obj.history.states(m.observer_time).poses(m.observer_id);            
            pEst = m.ToPose(p);
            
            DrawLine([p.position; m.observer_time], [pEst.position; m.target_time], ...
                'Color', color, 'LineWidth', obj.measurement_thickness);
                        
            obj.PlotPose(pEst, m.target_time, color, 0.5);
            
        end
        
    end
    
end