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
        time_overlap = 1;  % After receiving new chain, contracts to new_base - time_overlap*parent_time_scale 
        max_time_length = 9; % How many times to keep if no chains being received
        chain_holdoff = 0; % Sends chains from base to (latest_time - chain_holdoff)
        representative_holdoff = 0; % Sends representatives up to (latest_time - representative_holdoff)        
        
        base_id; % The actual ID, not mapped to index
        base_time; % The actual time, not mapped to index
        parent = ChainedGraph.empty(1,0);
        children = ChainedGraph.empty(1,0);
        depth = 0;
        
    end
    
    methods
        
        % Construct a chained graph        
        function [obj] = ChainedGraph(depth, tscale, toverlap, chold, rhold)
            
            % Default Tolerance 1E-6, maximum iterations 100
            obj.solver = GNSolver(1E-6, 100);
            
            obj.depth = depth;
            obj.time_scale = tscale;
            obj.time_overlap = toverlap;
            obj.chain_holdoff = chold;
            obj.representative_holdoff = rhold;
            
        end
        
        % Returns a shallow copy (doesn't copy parent, followers)
        function [new] = Copy(obj)
            
            new = ChainedGraph(obj.depth, obj.time_scale, obj.time_overlap, ...
                obj.chain_holdoff, obj.representative_holdoff);
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
            
            if numel(state) > 1
                error('ChainedGraph does not yet support initializing from multiple states.');
            end
            
            obj.subgraph = state;
            obj.robot_scope = state.ids;
            obj.time_scope = state.time;            
            % Hard coded initialization uncertainty
            obj.estimate_covariance = 1E-6*eye(3*numel(obj.robot_scope));                         
                        
            template_relation = MeasurementRelativePose();
            template_relation.observer_id = obj.subgraph.ids(1);
            template_relation.observer_time = obj.subgraph.time;
            template_relation.target_time = obj.subgraph.time;
            
            init_relations = cell(1, numel(obj.subgraph.ids) - 1);
            
            for i = 2:numel(obj.subgraph.ids);                
                template_relation.target_id = obj.subgraph.ids(i);
                init_relations{i-1} = ChainedGraph.ReadRelation(obj.subgraph, ...
                    template_relation, obj.estimate_covariance);                
            end
            
            obj.Incorporate(init_relations);
            
        end
        
        function SetBase(obj, base_time, base_id)

            if ~ismember(base_time, obj.time_scope)
               error('Base time %d not in time scope %4d.', base_time, obj.time_scope); 
            end            
            obj.base_time = base_time;
            
            if ~any(obj.subgraph(1).ids == base_id)
                error(['Cannot set base ID to outside of initialized scope. ', ...
                    'base ID: %d, ID scope: %4d'], base_id, state(1).ids);
            end
            obj.base_id = base_id;
            obj.Zero();
            
        end
        
        % Optimize the subgraph
        function Optimize(obj)
            
            anchor = [obj.base_id, obj.base_time];
            [obj.subgraph, obj.estimate_covariance] = ...
                                    obj.solver.Solve(obj.subgraph, anchor);
            obj.Zero(); % Make sure base node is at origin
                                
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
            t_new(1) = [];
            
            for i = 1:numel(t_new)
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
                error('Cut time %d not in scope $4d', t_start, obj.time_scope);
            end
            
            % Marginalize by optimizing info leaving scope
            % Will initialize overlap state to prev state's poses
            % EDIT: Turns out this seems bad. Just read off the graph for
            % now.
%             margin_graph = obj.subgraph(1:cut_ind);
%             margin_graph(end).poses = margin_graph(end-1).poses;
%             margin_graph(end).measurements = {};
%             [margin_graph, margin_cov] = obj.solver.Solve(margin_graph);
            margin_graph = obj.subgraph();
            margin_cov = obj.estimate_covariance;

            nonbase_ids = obj.subgraph(1).ids;
            nonbase_ids(nonbase_ids == obj.base_id) = [];
            
            template_relation = MeasurementRelativePose();
            template_relation.observer_id = obj.base_id;
            template_relation.observer_time = t_start;
            template_relation.target_time = t_start;
            
            margin_relations = cell(1, numel(nonbase_ids));
            
            for i = 1:numel(nonbase_ids)
                                
                template_relation.target_id = nonbase_ids(i);
                margin_relations{i} = ChainedGraph.ReadRelation(margin_graph, ...
                    template_relation, margin_cov);
                
            end
            
            % Cut subgraph and incorporate relations generated from
            % marginalizing cut information
            obj.time_scope = obj.time_scope(cut_ind:end);
            obj.subgraph = obj.subgraph(cut_ind:end);            
            obj.Incorporate(margin_relations);
            
            % Optimize and zero
            obj.Zero();
            obj.Optimize();
            
        end
        
        % Incorporate new measurements into the subgraph
        function Incorporate(obj, measurements)
            
            [~, tMap] = obj.subgraph.BuildMaps();
            for i = 1:numel(measurements)
                
                z = measurements{i};
                t_ind = tMap.Forward(z.observer_time);
                
                if isempty(t_ind)
                    continue;
                    % Not an error anymore..just ignore it.
                    %error(['Cannot incorporate out-of-scope measurement.', ...
                    %    ', z_t: ', num2str(z.observer_time), ...
                    %    ' scope: ', num2str(obj.time_scope),'\n']);
                end
                    
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
            
            obj.subgraph = obj.Compress(obj.subgraph);
            
        end
        
        % Measure relations in the subgraph
        function [z] = Extract(obj, relation)
            
            z = ChainedGraph.ReadRelation(obj.subgraph, relation, ...
            obj.estimate_covariance);
            
        end
        
        % Convenience function for creating relations compactly
        % obs is of form [obs_id, obs_time], same for tar        
        function [r] = CreateRelation(obj, obs, tar) 
        
            if strcmp(obs, 'base')
                obs = [obj.base_id, obj.base_time];
            end
            if strcmp(tar, 'base')
                tar = [obj.base_id, obj.base_time];
            end
            
            r = MeasurementRelativePose();
            r.observer_id = obs(1);
            r.observer_time = obs(2);
            r.target_id = tar(1);
            r.target_time = tar(2);
            
        end
        
        % Returns the difference between the overlapping poses with another
        % graph. We assume the IDs and times are sorted in ascending order.
        function [diff] = CompareGraphs(obj, other)
            
            if isa(other, 'ChainedGraph')
                other = other.subgraph;
            end            
            our = obj.subgraph;
            
            % Shift other subgraph into our reference frame
            % ID = -1 stands for global frame
            if obj.base_id ~= -1
                other = other.Zero(obj.base_id, obj.base_time);
                %other = other.Zero(obj.base_id, obj.time_scope(1));
            end
            
            % Get subset of other graph corresponding to local times
            t_matches = ismember([other.time], [our.time]);
            if sum(t_matches) ~= numel(obj.subgraph)
               error(['Comparison graph does not contain local subgraph.\n', ...
                   'Local ID: ', num2str(obj.subgraph(1).ids), '\n', ...
                   'Local t: ', num2str([obj.subgraph.time]), '\n', ...
                   'Other ID: ', num2str(other(1).ids), '\n', ...
                   'Other t: ', num2str([other.time])]);                    
            end
            other = other(t_matches);                      
            
            % Get subset of other graph that matches local
            id_matches = ismember(other(1).ids, our(1).ids);            
            T = numel(our);                     
            
            other_poses = [other.poses];
            other_poses = other_poses(:, repmat(id_matches, 1, T));
            our_poses = [our.poses];
            diff = other_poses - our_poses;
            diff(3,:) = wrapToPi(diff(3,:));
            
        end       
        
        % Returns a copy of this chained graph but set in a higher level
        % reference frame. num_links specifies number of links to apply,
        % with 0 links returning an unmodified copy.
        function [shifted] = ApplyLinks(obj, num_links)
           
            shifted = obj.Copy();
            
            for i = 1:num_links
               
                link = shifted.chain(end-i+1);
                shifted.subgraph = shifted.subgraph.Rotate(link.rotation);
                shifted.subgraph = shifted.subgraph.Shift(link.displacement);
                shifted.base_id = link.observer_id;
                shifted.base_time = link.observer_time;
                
            end
            
        end
        
        % Returns relation of (id, time) to k-th ancestral base node
        function [rel] = DepthExtract(obj, id, time, k)
        
            local_relation = obj.CreateRelation('base', [id, time]);
            rel = obj.Extract(local_relation);
            for i = 1:k
               rel = obj.chain(end-i+1).Compose(rel); 
            end
            
        end
        
    end
    
    methods(Access = private)
       
        function Zero(obj)
           
            [idMap, tMap] = obj.subgraph.BuildMaps();
            id_ind = idMap.Forward(obj.base_id);
            t_ind = tMap.Forward(obj.base_time);
            base_pose = obj.subgraph(t_ind).poses(:,id_ind);
            obj.subgraph = obj.subgraph.Shift(-base_pose(1:2));
            obj.subgraph = obj.subgraph.Rotate(-base_pose(3));
            
        end                
        
    end
    
    methods(Static)
            
        % Measures a relation from an optimized graph with estimation
        % covariance
        function [z] = ReadRelation(graph, relation, covariance)
            
            N = graph.GetDimension();
            T = numel(graph);
            if 3*N*T ~= size(covariance,1)
               error('Graph scope %4d does not match covariance size %d.', ...
                    3*N*T, size(covariance,1));
            end
            
            [idMap, tMap] = graph.BuildMaps();
            
            s_id = idMap.Forward(relation.observer_id);
            s_time = tMap.Forward(relation.observer_time);
            e_id = idMap.Forward(relation.target_id);
            e_time = tMap.Forward(relation.target_time);
            if isempty(s_id) || isempty(s_time) || isempty(e_id) || isempty(e_time)
                z = MeasurementRelativePose.empty(1,0);
                return
            end
            
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
       
        % Compacts a graph and its relations
        function [graph] = Compress(graph)
            
            for i = 1:numel(graph)
                
                z = [graph(i).measurements{:}];                
                graph(i).measurements = MeasurementRelativePose.Compact(z);
                
            end
            
        end 
        
    end
    
end