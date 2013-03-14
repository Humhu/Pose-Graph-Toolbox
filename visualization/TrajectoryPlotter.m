classdef TrajectoryPlotter < Plotter2D
    
    properties
        
    end
    
    methods
        
        function [obj] = TrajectoryPlotter(dims)
            
            obj = obj@Plotter2D(dims);
            
            obj.Clear();
            xlabel(obj.axe, 'x');
            ylabel(obj.axe, 'y');
            
        end
        
        function PlotTrajectory(obj, seq)
            
            last = seq(end);
            N = seq.GetDimension();
            T = numel(seq);
            for i = 1:N
                
                p = last.poses(:,i);
                c = obj.colors(i,:);
                obj.PlotRobot(p, 0, c, 1.0);
                
            end
            
            poses = [seq.poses];
            x = reshape(poses(1,:), N, T);
            y = reshape(poses(2,:), N, T);
            for i = 1:N
               
                c = obj.colors(i,:);
                plot(obj.axe, x(i,:), y(i,:), 'color', c, 'parent', obj.line_handles);
                
            end
            
        end
        
    end
    
end

