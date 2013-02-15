% Extends a Plotter2D with sequence plotting
classdef SequencePlotter < Plotter2D
    
    properties
        
        history;        % Data being shown
        time_z_scale    = 1.0; % Conversion from time to z height
        tMap;           % Time to index mapping
        idMap;          % ID to index mapping
        
    end
    
    methods
        
        function [obj] = SequencePlotter(dims)
            
            obj = obj@Plotter2D(dims);
            
            obj.Clear();
            
            xlabel(obj.axe, 'x');
            ylabel(obj.axe, 'y');
            zlabel(obj.axe, 't');
            
        end
        
        function Clear(obj)
            
            Clear@Plotter2D(obj);
            obj.history =  WorldState2D.empty(1,0);
            
        end
        
        function [] = PlotSequence(obj, seq)
            
            T = numel(seq);
            for t = 1:T
                obj.PlotState(seq(t));
            end
            
        end               
        
%         function [] = PlotSequenceCovariances(obj, truth, belief, cov)
%             
%             T = numel(belief);
%             for t = 1:T
%                 obj.PlotMeasurements(belief.states(t));
%                 obj.PlotPoseCovariances(belief.states(t), cov(t,:));
%             end
%             
%         end
%         
%         function PlotPoseCovariances(obj, state, cov)
%             
%             axes(obj.axe);
%             
%             n = size(state.poses, 2);
%             
%             hold on;
%             for i = 1:n
%                 p = state.poses(:,i);
%                 obj.PlotEllipse(p, state.time, cov{i});
%             end
%             hold off;
%             
%         end
        
    end
    
    methods(Access = protected)
        
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
                obj.PlotRobot(p, state.time, color, 1.0);
            end
            for i = 1:n
                p = state.poses(:,i);
                obj.PlotLabel(p, state.time, num2str(state.ids(i)));
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
            obj.PlotLine([p(1:2); to], [pEst(1:2); tt], ...
                {'Color', color, 'LineWidth', thickness});
            
        end
        
    end
    
end