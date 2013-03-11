% Builds a graph using odometry only
classdef OdometrySolver < handle
    
    properties
        
    end
    
    methods
        
        function [obj] = OdometrySolver(tol, max_iters)
            
            if nargin == 0
                return
            end
            
            %obj = obj@GNSolver(tol, max_iters);
            
        end
        
        function [solution, cov] = Solve(obj, sequence, anchor)
                       
           solution = obj.StripMeasurements(sequence);
           [idMap, tMap] = solution.BuildMaps();
           
           for i = 1:numel(sequence)
              
               measurements = solution(i).measurements;
               for j = 1:numel(measurements)
                  
                   z = measurements{j};
                   id_ind = idMap.Forward(z.observer_id);
                   ts_ind = tMap.Forward(z.observer_time);
                   te_ind = tMap.Forward(z.target_time);
                   
                   obs_p = solution(ts_ind).poses(:,id_ind);
                   tar_p = z.ToPose(obs_p);
                   solution(te_ind).poses(:,id_ind) = tar_p;
                   
               end
               
           end
           
           %[solution, cov] = obj.Solve@GNSolver(sequence, anchor);
            
        end
        
    end
    
    methods(Access = private)
       
        % Strips non-odometric measurements
        function [stripped] = StripMeasurements(obj, sequence)
            
            stripped = sequence;
            for i = 1:numel(stripped)
               
                s = stripped(i);
                keep = false(1, numel(s.measurements));
                for j = 1:numel(s.measurements)
                   
                    z = s.measurements{j};
                    if z.target_id == z.observer_id
                        keep(j) = 1;
                    end
                    
                end
                stripped(i).measurements = s.measurements(keep);
                
            end
            
        end
        
    end
    
end