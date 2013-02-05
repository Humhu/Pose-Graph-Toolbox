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
        ellipse_points = 16;
        ellipse_thickness = 0.4;
        measurement_thickness = 0.4;
        text_size       = 10;
        
        time_z_scale    = 0.1 % Conversion from time to z height
        
        tMap;           % Time to index mapping
        idMap;          % ID to index mapping
        
    end
    
    methods
        
        function obj = Plotter2D(size)
            if nargin == 0
                return
            end
            
            obj.history = WorldState2D.empty(1,0);
            
            obj.dims = reshape(size, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
            grid off;
            axis vis3d;
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
            obj.history = [];
            
        end
        
        function [] = PlotSequence(obj, seq)
            
            T = numel(seq);
            for t = 1:T
                obj.PlotState(seq(t));
            end
            
        end
        
        function [] = PlotState(obj, state)
            
            obj.history = [obj.history, state];
            [obj.idMap, obj.tMap] = obj.history.BuildMaps();
            obj.PlotMeasurements(state);
            obj.PlotPoses(state);
            
        end
        
        function [] = PlotPoses(obj, state)
            
            axes(obj.axe);
            
            n = size(state.poses, 2);
            
            hold on;
            for i = 1:n
                p = state.poses(:,i);
                color = obj.colors(i,:);
                obj.PlotPose(p, state.time, color, 1.0);
            end
            for i = 1:n
                p = state.poses(:,i);
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
                    %obj.PlotRelativePose(m.ToInverse());
                end
            end
            hold off;
            
        end
        
        function [] = PlotSequenceCovariances(obj, truth, belief, cov)
            
            T = numel(belief);
            for t = 1:T
                obj.PlotMeasurements(belief.states(t));
                obj.PlotPoseCovariances(belief.states(t), cov(t,:));
            end
            
        end
        
        function PlotPoseCovariances(obj, state, cov)
            
            axes(obj.axe);
            
            n = size(state.poses, 2);
            
            hold on;
            for i = 1:n
                p = state.poses(:,i);
                obj.PlotCovEllipse(p, state.time, cov{i});
            end
            hold off;
            
        end
        
    end
    
    methods(Access = private)
        
        function PlotPose(obj, p, t, c, s)
            
            x = p(1);
            y = p(2);
            t = t*obj.time_z_scale;
            
            % Plot circle
            DrawCircle([x,y,t], s*obj.robot_size, obj.circle_points, ...
                'Color', c, 'LineWidth', obj.robot_thickness);
            
            % Plot orientation tick
            a = p(3);
            dx = s*obj.tick_length*cos(a);
            dy = s*obj.tick_length*sin(a);
            DrawLine([x, y, t], [x + dx, y + dy, t], ...
                'Color', c, 'LineWidth', obj.tick_thickness);
            
        end
        
        function PlotLabel(obj, p, t, l)
            
            x = p(1);
            y = p(2);
            t = t*obj.time_z_scale;
            text(x, y, t, l, 'FontSize', obj.text_size, ...
                'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
            
        end
        
        function PlotLegend(obj)
            
            n = size(obj.colors, 1);
            legend(cellstr(num2str((1:n)')), 'location', 'eastoutside');
            
        end
        
        function PlotCovEllipse(obj, p, t, cov)                      
            
            t = t*obj.time_z_scale;
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
        
        % TODO: Update for consistency!
%         function PlotRangeBearing(obj, fs, m)
%             
%             id1 = m.observer_id;
%             color = obj.colors(id1,:);
%             p = fs.states(m.observer_time).poses(id1);
%             x = p.position(1);
%             y = p.position(2);
%             
%             dx = m.range*cos(wrapToPi(p(3) + m.bearing));
%             dy = m.range*sin(wrapToPi(p(3) + m.bearing));
%             DrawLine([x, y, m.observer_time], [x + dx, y + dy, m.target_time], ...
%                 'Color', color, 'LineWidth', obj.measurement_thickness);
%             
%         end
        
        function PlotRelativePose(obj, m)
            
            tar_id = obj.idMap.Forward(m.target_id);
            obs_id = obj.idMap.Forward(m.observer_id);
            tar_t = obj.tMap.Forward(m.target_time);
            obs_t = obj.tMap.Forward(m.observer_time);
            
            if isempty(tar_t) || isempty(obs_t)
                return
            end
            
            color = obj.colors(tar_id,:);
            p = obj.history(obs_t).poses(:,obs_id);
            pEst = m.ToPose(p);
            
            thickness = obj.measurement_thickness;            
            to = m.observer_time*obj.time_z_scale;
            tt = m.target_time*obj.time_z_scale;
            DrawLine([p(1:2); to], [pEst(1:2); tt], ...
                'Color', color, 'LineWidth', thickness);
            
            %obj.PlotPose(pEst, m.target_time, color, 0.5);
            
        end
        
    end
    
end