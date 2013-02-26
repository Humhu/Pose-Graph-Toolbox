% CHAINEDGRAPH - A data structure containing a pose graph and a set of
% composable measurements known as a chain
%
% Supports basic operations of otpimization, extension, contraction, 
% incorporation, and extraction.
classdef ChainedGraph < handle
    
    properties
        
        subgraph = WorldState2D.empty(1,0);
        chain = MeasurementRelativePose.empty(1,0);
        solver = GNSolver.empty(1,0);
        estimate_covariance;
        
        robot_scope = [];
        time_scope = [];
        time_scale = 1;
        time_overlap = 1;
        
        base_id; % The actual ID, not mapped to index
        base_time; % The actual time, not mapped to index
        parent = ChainedGraph.empty(1,0);
        children = ChainedGraph.empty(1,0);
        depth = 0;
        
    end
    
    methods
        
        % Construct a chained graph        
        function [obj] = ChainedGraph(depth, tscale, toverlap)
            
            % Default Tolerance 1E-6, maximum iterations 100
            obj.solver = GNSolver(1E-6, 100);
            
            obj.depth = depth;
            obj.time_scale = tscale;
            obj.time_overlap = toverlap;
            
        end
        
        % Returns a shallow copy (doesn't copy parent, followers)
        function [new] = Copy(obj)
            
            new = ChainedGraph(obj.depth, obj.time_scale, obj.time_overlap);
            new.subgraph = obj.subgraph;
            new.chain = obj.chain;
            new.solver = obj.solver.Copy();
            new.estimate_covariance = obj.estimate_covariance;
            new.robot_scope = obj.robot_scope;
            new.time_scope = obj.time_scope;
            new.base_id = obj.base_id;
            new.base_time = obj.base_time;
            new.parent = obj.parent;
            new.children = obj.children;            
            
        end
        
        % Initialize the subgraph and set the base parameters
        % base time automatically 
        function Initialize(obj, state)
            
           obj.subgraph = state;
           obj.robot_scope = state(1).ids;
           obj.time_scope = [state.time];
           N = 3*numel(obj.robot_scope)*numel(obj.time_scope);
           obj.estimate_covariance = 1E-6*eye(N); % Hard coded initialization uncertainty
           
        end
        
        function SetBase(obj, base_time, base_id)

            if ~ismember(base_time, obj.time_scope)
               error('Base time %d not in time scope %d.', base_time, obj.time_scope); 
            end            
            obj.base_time = base_time;
            
            if ~any(obj.subgraph(1).ids == base_id)
                error(['Cannot set base ID to outside of initialized scope. ', ...
                    'base ID: %d, ID scope: %d'], base_id, state(1).ids);
            end
            obj.base_id = base_id;
            
        end
        
        % Optimize the subgraph
        function Optimize(obj)
            
            [obj.subgraph, obj.estimate_covariance] = ...
                                    obj.solver.Solve(obj.subgraph);
            
        end
        
        % Extend the subgraph
        function Extend(obj, t_end)
                                    
            if mod(t_end, obj.time_scale) ~= 0
                error(['End time ', num2str(t_end), ' not a multiple of ', ...
                    'time scale ', num2str(obj.time_scale)]);
            end
                
            if t_end <= obj.time_scope(end)
                return;
            end
            
            t_last = obj.time_scope(end);
            t_new = t_last:obj.time_scale:t_end;
            
            % Start from 2 so we don't include t_last
            for i = 2:numel(t_new)
               new_state = obj.subgraph(end);
               new_state.measurements = {}; 
               % Add dummy measurements               
               dummies = cell(1, numel(new_state.ids));
               for j = 1:numel(new_state.ids)
                  z = MeasurementRelativePose();
                  z.displacement = zeros(2,1);
                  z.rotation = 0;
                  z.observer_id = new_state.ids(j);
                  z.target_id = new_state.ids(j);
                  z.observer_time = new_state.time;
                  z.target_time = t_new(i);
                  dummies{j} = z;
               end                       
               obj.subgraph(end).measurements = ...
                   [obj.subgraph(end).measurements, dummies];
               new_state.time = t_new(i);
               obj.subgraph(end+1) = new_state;
            end
            
            obj.time_scope = [obj.time_scope, t_new];
            
        end
        
        % Contract and marginalize the subgraph
        function Contract(obj, t_start)
            
            if t_start <= obj.time_scope(1)
                return;
            end
            
            [~, tMap] = obj.subgraph.BuildMaps();            
            cut_ind = tMap.Forward(t_start);
            
            if isempty(cut_ind)
                error('Cut time %d not in scope $d', t_start, obj.time_scope);
            end
            
            % Marginalize by optimizing info leaving scope
            % Will initialize overlap state to prev state's poses
            margin_graph = obj.subgraph(1:cut_ind);
            margin_graph(end).poses = margin_graph(end-1).poses;
            margin_graph(end).measurements = {};
            [margin_graph, margin_cov] = obj.solver.Solve(margin_graph);
                        
            nonbase_ids = obj.subgraph(1).ids;
            nonbase_ids(nonbase_ids == obj.base_id) = [];
            
            template_relation = MeasurementRelativePose();
            template_relation.observer_id = obj.base_id;
            template_relation.observer_time = t_start;
            template_relation.target_time = t_start;
            
            margin_relations = cell(1, numel(nonroot_ids));
            
            for i = 1:numel(nonroot_ids)
                                
                template_relation.target_id = nonbase_ids(i);
                margin_relations{i} = ChainedGraph.ReadRelation(margin_graph, ...
                    template_relation, margin_cov);
                
            end
            
            % Cut subgraph and incorporate relations generated from
            % marginalizing cut information
            obj.subgraph = obj.subgraph(cut_ind:end);            
            obj.Incorporate(margin_relations);
            
        end
        
        % Incorporate new measurements into the subgraph
        function Incorporate(obj, measurements)
            
            [~, tMap] = obj.subgraph.BuildMaps();
            for i = 1:numel(measurements)
                
                z = measurements{i};
                t_ind = tMap.Forward(z.observer_time);
                % Append non-odometric, replace odometric
                if z.observer_time == z.target_time
                    obj.subgraph(t_ind).measurements{end+1} = z;
                else
                    match = z.SameRelation([obj.subgraph(t_ind).measurements{:}]);
                    if ~any(match)
                        obj.subgraph(t_ind).measurements{end + 1} = z;
                    else
                        obj.subgraph(t_ind).measurements{match} = z;
                    end
                end
                
            end
            
        end
        
        % Measure relations in the subgraph
        function [z] = Extract(obj, relation)
            
            z = ChainedGraph.ReadRelation(obj.subgraph, relation, ...
            obj.estimate_covariance);
            
        end
        
    end
    
    methods(Static, Access=private)
    
        % Measures a relation from an optimized graph with estimation
        % covariance
        function [z] = ReadRelation(graph, relation, covariance)
            
            N = graph.GetDimension();
            T = numel(graph);
            if 3*N*T ~= size(covariance,1)
               error('Graph scope %d does not match covariance size %d.', ...
                    3*N*T, size(covariance,1));
            end
            
            [idMap, tMap] = graph.BuildMaps();
            
            s_id = idMap.Forward(relation.observer_id);
            s_time = tMap.Forward(relation.observer_time);
            e_id = idMap.Forward(relation.target_id);
            e_time = tMap.Forward(relation.target_time);
            
            start_pose = graph(s_time).poses(:,s_id);
            end_pose = graph(e_time).poses(:,e_id);
            
            z = MeasurementRelativePose(start_pose, end_pose, zeros(3));
            z.observer_id = relation.observer_id;
            z.observer_time = relation.observer_time;
            z.target_id = relation.target_id;
            z.target_time = relation.target_time;     
            
            prev_start = 3*N*(s_time - 1) + (s_id - 1) + 1;
            prev_end = prev_start + 2;
            curr_start = 3*N*(e_time - 1) + (e_id - 1) + 1;
            curr_end  = curr_start + 2;
            inds = [prev_start:prev_end, curr_start:curr_end];
            
            jointcov = covariance(inds, inds);
            b = [-eye(3), eye(3)];
            z.covariance = b*jointcov*b';
            
        end
        
    end
    
end