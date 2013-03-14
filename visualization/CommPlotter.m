classdef CommPlotter < Plotter2D
    
    properties
        
        arrow_handles;
        
    end
    
    methods
        
        function [obj] = CommPlotter(N)
           
            obj = obj@Plotter2D([1, N]);
            obj.Clear();
            
            xlabel('t');
            ylabel('Comm ID');
            
        end
        
        function Clear(obj)
           
            Clear@Plotter2D(obj);
            delete(obj.arrow_handles);
            obj.arrow_handles = [];
            
        end
        
        function PlotCommunications(obj, commlog, ids)
            
            from_match = ismember(commlog(:,2), ids);
            to_match = ismember(commlog(:,3), ids);
            commlog = commlog(from_match & to_match, :);
            
            
            N = size(commlog, 1);
            t = commlog(:,1);
            from = commlog(:,2);
            to = commlog(:,3);
            
            start_x = t;
            start_y = from;
            end_x = t + 1;
            end_y = to;
            
            h = arrow([start_x, start_y], [end_x, end_y], 'LineWidth', 2);
            obj.arrow_handles = [obj.arrow_handles, h];            
            axis(obj.axe, 'tight');
        end
        
    end
    
end