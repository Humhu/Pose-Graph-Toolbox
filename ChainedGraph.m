% CHAINEDGRAPH - A data structure containing a pose graph and a set of
% composable measurements known as a chain
%
% Supports basic operations of extension, contraction, incorporation, and
% extraction
classdef ChainedGraph < handle
    
    properties
        
        subgraph = WorldState2D.empty(1,0);
        chain = MeasurementRelativePose.empty(1,0);
        solver = GNSolver.empty(1,0);
        
        robot_scope = [];
        time_scope = [];
        time_scale = 1;
        time_overlap = 1;
        
        parent = ChainedGraph.empty(1,0);
        followers = ChainedGraph.empty(1,0);
        depth = 0;
        
    end
    
    methods
        
        % Construct a chained graph        
        function [obj] = ChainedGraph(depth, tscale, toverlap)
            
            % Tolerance 1E-6, maximum iterations 100
            obj.solver = GNSolver(1E-6, 100);
            
            obj.depth = depth;
            obj.time_scale = tscale;
            obj.time_overlap = toverlap;
            
        end
        
        function Initialize(obj, state)
            
           obj.subgraph = state;
           obj.robot_scope = state(1).ids;
           obj.time_scope = [state.time];
            
        end
        
        function Extend(obj, t_end)
            
            if mod(t_end, obj.time_scale) ~= 0
                error(['End time ', num2str(t_end), ' not a multiple of ', ...
                    'time scale ', num2str(obj.time_scale)]);
            end
                
            t_last = obj.time_scope(end);
            t_new = t_last:obj.time_scale:t_end;
            
            for i = 1:numel(t_new)
               new_state = obj.subgraph(end);
               new_state.measurements = {}; 
               % Add dummy measurements                              
               for j = 1:numel(new_state.ids)
                  z = MeasurementRelativePose();
                  z.displacement = zeros(2,1);
                  z.rotation = 0;
                  z.observer_id = new_state.ids(j);
                  z.target_id = new_state.ids(j);
                  z.observer_time = new_state.time;
                  z.target_time = t_new(i);
                  new_state.measurements{j} = z;
               end                       
               new_state.time = t_new(i);
               obj.subgraph(end+1) = new_state;
            end
            
            obj.time_scope = [obj.time_scope, t_new];
            
        end
        
        function Contract(obj, t_start)
            
            
            
        end
        
        function Incorporate(obj, z)
            
        end
        
        % Measure a relation in the subgraph
        function [z] = Extract(obj, relation)
            
            
            
        end
        
    end
    
end