classdef ScopePlotter < Plotter2D
    
    properties
        
        patch_handles;
        
    end
    
    methods
        
        function [obj] = ScopePlotter()
           
            obj = obj@Plotter2D([1,1]);
            obj.Clear();                        
            
        end
        
        function Clear(obj)
           
            Clear@Plotter2D(obj);
            delete(obj.patch_handles);
            obj.patch_handles = [];            
                        
        end
        
        function PlotTimeScopes(obj, scopelog)
           
            N = size(scopelog, 2);
            
            for i = 1:N
               
                scopes = scopelog(:,i);
                T = size(scopes, 1);
                vertices = zeros(2*T, 2);
                
                for j = 1:T
                    times = scopes{j};
                    vertices(j,:) = [j, min(times)];
                    vertices(end - j + 1,:) = [j, max(times)];
                end
                h = patch(vertices(:,1), vertices(:,2), obj.colors(i,:), ...
                    'EdgeColor', obj.colors(i,:), 'LineWidth', 2, ...
                    'LineStyle', '-', ...
                    'FaceAlpha', 0.4, 'Parent', obj.axe);
                obj.patch_handles(end + 1) = h;
                
            end
            axis(obj.axe, 'tight');
            xlabel(obj.axe, 'Time step, t');
            ylabel(obj.axe, 'Time scope limits');
            title(obj.axe, 'Time scope coverages');
            
        end
       
        function PlotTimeScopesSparse(obj, scopelog, sizes)
            
            N = size(scopelog, 2);
            
            for i = 1:N
                
                scopes = scopelog(:,i);
                T = size(scopes, 1);
                points = [];
                
                for j = 1:T
                    times = scopes{j};
                    n = numel(times);
                    newpoints = [j*ones(1, n); times];
                    points = [points, newpoints];
                end
                h = plot(points(1,:), points(2,:), 'Marker', 'o', ...
                    'LineStyle', 'none', 'MarkerEdgeColor', 'k', ...
                    'MarkerFaceColor', obj.colors(i,:), ...
                    'MarkerSize', sizes(i));
                obj.patch_handles(end + 1) = h;
                
            end
            axis(obj.axe, 'tight');
            xlabel(obj.axe, 'Time step, t');
            ylabel(obj.axe, 'Time scope limits');
            title(obj.axe, 'Time scope coverages');
            
        end
        
    end
    
end