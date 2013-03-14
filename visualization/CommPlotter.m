classdef CommPlotter < Plotter2D
    
    properties
        
        arrow_handles;
        radial_r = 1.0;
        
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
        
        function PlotCommunications(obj, commlog, x, ids)
            
            if nargin == 3
               ids = unique([commlog(:,2); commlog(:,3)]);
            end
            
            from_match = ismember(commlog(:,2), ids);
            to_match = ismember(commlog(:,3), ids);
            commlog = commlog(from_match & to_match, :);
                        
            N = size(commlog, 1);
            t = commlog(:,1);
            from = commlog(:,2);
            to = commlog(:,3);                                  
            type = commlog(:,4);
            
            ids = unique([from; to]);
            num_ids = numel(ids);
            idMap = SearchMap(ids, 1:num_ids); % Supports multi-forward
            x = reshape(x, numel(x), 1);
            
            begin = [t, x(idMap.Forward(from))];            
            finish = [t + 1, x(idMap.Forward(to))];                        
            
            all_types = unique(type);
            num_types = numel(all_types);
            
            for i = 1:num_types
                match = type == all_types(i);
                c = obj.colors(all_types(i),:);
                h = arrow(begin(match,:), finish(match,:), 'LineWidth', 2, ...
                    'EdgeColor', c, 'FaceColor', c);
                obj.arrow_handles = [obj.arrow_handles; h];            
            end
            
            axis(obj.axe, 'tight');
            set(obj.axe, 'xtick', min(t):max(t), 'ytick', ids, ...
                'xgrid', 'on', 'ygrid', 'on');
            
        end
                
        % TODO: Make look better
        function PlotCommunicationsRadial(obj, commlog, ids)
           
            from_match = ismember(commlog(:,2), ids);
            to_match = ismember(commlog(:,3), ids);
            commlog = commlog(from_match & to_match, :);
                        
            N = size(commlog, 1);
            t = commlog(:,1);
            from = commlog(:,2);
            to = commlog(:,3);
            
            ids = unique([from; to]);
            num_ids = numel(ids);
            idMap = SearchMap(ids, 1:num_ids); % Supports multi-forward
            
            angles = linspace(0, 2*pi, num_ids + 1)';
            angles(end) = [];            
            origins = [cos(angles), sin(angles)];
            
            for i = 1:num_ids
               obj.PlotLabel(origins(i,:), 0, num2str(ids(i)));
            end
            
            begin = [origins(idMap.Forward(from),:), t];            
            finish = [origins(idMap.Forward(to),:), t + 1];
            
            h = arrow(begin, finish, 'LineWidth', 2);
            obj.arrow_handles = [obj.arrow_handles, h];            
            axis(obj.axe, 'tight');
            
        end        
        
    end
    
end