% HIERARCHYROLE - Reactive softare agent that manages a chained graph
% Contains all information required to perform role's duties
classdef HierarchyRole < handle
    
    properties
        
        time;       % Internal time counter
        
        ownerID;    % ownerID of corresponding robot
        leader;     % Reference to leader HierarchyRole object
        followers;  % Array of follower HierarchyRole objects
        
        estimates = MeasurementRelativePose();
        higher_beliefs; % MeasurementRelativePose array representing ancestor beliefs
        % Each Measurement at index i corresponds to the relative pose of
        % the ancestor at level (i + 1) wrt ancestor at level (i).
        
        chained_graph;
        last_sent;
        
        tx_buffer;  % BinBuffer storing higher level measurements to send
        rx_buffer;  % BinBuffer that stores measurements from followers
        
    end
    
    methods
        
        function [obj] = HierarchyRole(level, time_scale, time_overlap, ...
                chold, rhold)
            
            obj.chained_graph = ChainedGraph(level, time_scale, time_overlap, ...
                chold, rhold);
            obj.higher_beliefs = MeasurementRelativePose.empty(1, 0);
            obj.tx_buffer = BinBuffer(1, 50);
            obj.rx_buffer = BinBuffer(1, 50);
            obj.time = 0;
            obj.last_sent = 0;
            
        end
        
        % Assigns followers and initializes relevant fields
        % Also assigns followers' leader field
        function AssignFollowers(obj, followers)
            
            n = numel(followers);
            obj.followers = followers;
            obj.chained_graph.children = [followers.chained_graph];
            for i = 1:n
                obj.followers(i).leader = obj;
                obj.followers(i).chained_graph.parent = obj.chained_graph;
            end
            
        end
        
        % Creates a deep copy of this HierarchyRole object
        function [new] = Copy(obj)
            
            new = HierarchyRole(obj.chained_graph.depth, ...
                obj.chained_graph.time_scale, ...
                obj.chained_graph.time_overlap);
            new.time = obj.time;
            new.ownerID = obj.ownerID;
            new.leader = obj.leader;
            new.followers = obj.followers;
            new.higher_beliefs = obj.higher_beliefs;
            new.chained_graph = obj.chained_graph.Copy();
            new.last_sent = obj.last_sent;
            new.tx_buffer = obj.tx_buffer.Copy();
            new.rx_buffer = obj.rx_buffer.Copy();
            
        end
        
        % TODO: Clean up naming for these Get/Traverse type methods
        % Return indices of team (self and followers)
        function [ids] = GetTeam(obj)
            
            ids = obj.chained_graph.robot_scope;
            
        end
        
        % Get IDs of all ancestors FIX?
        function [ids] = GetDescendants(obj)
            
            ids = obj.GetTeam();
            for i = 1:numel(obj.followers)
                ids = [ids, obj.followers(i).GetDescendants()];
            end
            ids = unique(ids);
            
        end
        
        % Return ancestor k levels above
        function [anc] = GetAncestor(obj, k)
            
            anc = obj;
            for i = 1:k
                anc = anc.leader;
            end
            
        end
        
        % Queries position at time t relative to level k leader
        % k = 0 returns relative to root, k = -1 returns global estimate
        function [rel] = QueryRelation(obj, k, t)
            
            if ~ismember(t, obj.chained_graph.time_scope)
                error('Invalid relation query %d in scope %d', ...
                    t, obj.chained_graph.time_scope);
            end
            
            relation = obj.chained_graph.CreateRelation('base', 'base');
            relation.target_time = t;
            
            local_rel = obj.chained_graph.Extract(relation);
            
            if k < obj.chained_graph.depth
                k_rel = obj.estimates(k+2);
                rel = k_rel.Compose(local_rel);
            else
                rel = local_rel;
            end
            
        end        
        
        % Initialize agent's local_beliefs
        function Initialize(obj, state)
            
            % Doesn't work until graph initialized... fix?
            %ids = obj.GetTeam();
            if isempty(obj.followers)
                ids = obj.ownerID;
            else
                ids = [obj.followers.ownerID];
            end
                
            substate = state;
            substate.poses = substate.poses(:,ismember(substate.ids, ids));
            substate.ids = ids;
            substate = substate.Zero(); % Leader is at origin
            substate.measurements = {}; % Do we want this?
            
            obj.chained_graph.Initialize(substate);
            obj.chained_graph.SetBase(substate.time, substate.ids(1));
            
            obj.time = 0;
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f.Initialize(state);
            end
            
            if obj.chained_graph.depth ~= 0
                return
            end
            
            global_pose = state.poses(:,1);
            global_belief = MeasurementRelativePose(zeros(3,1), global_pose, zeros(3));
            global_belief.observer_id = -1;
            global_belief.observer_time = 0;
            global_belief.target_id = obj.ownerID;
            global_belief.target_time = 0;
            global_belief.covariance = 1E-6*eye(3);
            
            obj.ChainUpdate(global_belief);
            
        end
        
        % Update the subgraph chain in response to receiving new links
        % TODO: Switch to use TX buffer instead of direct function call
        function ChainUpdate(obj, beliefs)
            
            obj.chained_graph.chain = beliefs;
            
            % Update local position estimates
            obj.BuildEstimates(obj.chained_graph.chain);
            
            % Update local graph times
            base_time = beliefs(end).target_time;
            obj.chained_graph.SetBase(base_time, obj.chained_graph.base_id);
            
            % We leave some overlap when contracting
            if ~isempty(obj.chained_graph.parent)
                parent_scale = obj.chained_graph.parent.time_scale;
            else
                parent_scale = obj.chained_graph.time_scale;
            end
            
            alpha = obj.chained_graph.time_overlap;
            start_time = base_time - alpha*parent_scale;
            obj.chained_graph.Contract(start_time);
            
            obj.TransmitChains();
            
        end
        
        % Translates an extra-group measurement to the appropriate frame
        % and level
        % Currently we assume that only the target ID can be extra-group
        function TranslateMeasurement(obj, m)
            
            % Check target
            if ~ismember(m.target_id, obj.GetTeam())
                % Find target's corresponding role agent and LCA
                other = FindLMR(obj, m.target_id);
                common = FindLCA(obj, other);
                
                % Determine proxies to translate to
                local_proxy = obj.GetAncestor(obj.chained_graph.depth - common.chained_graph.depth - 1);
                other_proxy = other.GetAncestor(other.chained_graph.depth - common.chained_graph.depth - 1);
                
                % Need to translate to nearest valid common time step
                % TODO: How to consolidate?
                %common_times = common.GetGraphTimes();
                %common_obs_time = m.observer_time - mod(m.observer_time, common.time_scale);
                %common_tar_time = m.target_time - mod(m.target_time, common.time_scale);
                
                local_to_proxy = obj.QueryRelation(local_proxy.chained_graph.depth, m.observer_time);
                other_to_proxy = other.QueryRelation(other_proxy.chained_graph.depth, m.target_time);
                
                m = local_to_proxy.Compose(m);
                m = m.Compose(other_to_proxy.ToInverse());
                common.PushMeasurements(m);
                
            end
            
            
        end
        
        % Input measurements into this agent for processing, whether from a
        % robot or follower agent
        function PushMeasurements(obj, measurements)
        
            obj.rx_buffer.Push(1, measurements);
        
        end
        
        % Process internal functions. Should be called once every time
        % step.
        function Step(obj)
        
            % Begins a 'fake' chain update for root
            if obj.chained_graph.depth == 0

                relation = obj.chained_graph.CreateRelation('base', 'base');
                %relation.target_time = obj.chained_graph.time_scope(end);
                beta = obj.chained_graph.chain_holdoff;
                if beta >= numel(obj.chained_graph.time_scope)
                    t_ind = 1;
                else
                    t_ind = numel(obj.chained_graph.time_scope) - beta;
                end
                relation.target_time = obj.chained_graph.time_scope(t_ind);
                z = obj.chained_graph.Extract(relation);
                gchain = obj.chained_graph.chain(1);
                gchain = gchain.Compose(z);
                obj.ChainUpdate(gchain);
                
            end

            % If there are no new measurements, there's nothing to do!
            if obj.rx_buffer.IsEmpty()
                obj.time = obj.time + 1;
                return
            end
            
            measurements = obj.rx_buffer.PopAll();
            
            % Translate and transmit extra-team measurements
            team_ids = obj.GetTeam();
            remove = false(numel(measurements), 1);
            for i = 1:numel(measurements)
                z = measurements{i};
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    obj.TranslateMeasurement(z); % Sends it to a better place
                    remove(i) = 1;
                end
            end
            measurements(remove) = [];
            
            % Determine new times
            times = zeros(1,2*numel(measurements));
            for i = 1:numel(measurements)
                z = measurements{i};
                times(2*i-1:2*i) = [z.observer_time, z.target_time];
            end
            latest_t = max(times);
            
            % Extend to cover new times, then incorporate and optimize
            obj.chained_graph.Extend(latest_t);
            obj.chained_graph.Incorporate(measurements);            
            
            %fprintf('Optimizing at ID: %d, k: %d, t: %d\n', obj.ownerID, ...
            %    obj.chained_graph.depth, obj.time)
            %fprintf(['\t in robot scope: ', num2str(obj.chained_graph.robot_scope), ...
            %    ' time scope: ', num2str(obj.chained_graph.time_scope), '\n']);
            obj.chained_graph.Optimize();
            
            % Inform leader here
            obj.TransmitRepresentatives();
            
            % Chain update here
            obj.TransmitChains();
            obj.time = obj.time + 1;
            
        end
        
    end
    
    methods(Access = private)
        
        % Builds this agent's root position in all higher reference frames
        % given a chain of relative measurements. The chain should be
        % ordered with measurements(1) corresponding to root->f1, and
        % measurements(end) corresponding to fn->obj
        function BuildEstimates(obj, measurements)
            
            if numel(measurements) ~= obj.chained_graph.depth + 1
                error(['Wrong number of measurements given to BuildEstimates: ', ...
                    'level: %d, #meas: %d'], obj.chained_graph.depth, ...
                    numel(measurements));
            end
            
            % Initialize all as last link
            for i = 1:obj.chained_graph.depth + 1
                obj.estimates(i) = measurements(end);
            end
            
            % Build chain by composing measurements in reverse-order
            for i = obj.chained_graph.depth:-1:1
                m = measurements(i);
                for j = i:-1:1
                    obj.estimates(j) = m.Compose(obj.estimates(j));
                end
            end
            
        end
        
        % Extract and send new links to followers
        function TransmitChains(obj)
            
            relation = obj.chained_graph.CreateRelation('base', 'base');
            
            beta = obj.chained_graph.chain_holdoff;
            ages = obj.time - obj.chained_graph.time_scope;
            t_ind = find(ages > beta, 1, 'last');
            if ~isempty(t_ind)
                relation.target_time = obj.chained_graph.time_scope(t_ind);
            end            
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                relation.target_id = f.ownerID;
                z = obj.chained_graph.Extract(relation);
                f.ChainUpdate([obj.chained_graph.chain, z]);
            end
            
        end
        
        function TransmitRepresentatives(obj)
           
            if isempty(obj.chained_graph.parent)
                return
            end
            
            parent_scale = obj.chained_graph.parent.time_scale;
            start_time = obj.chained_graph.subgraph(1).time;
            
            gamma = obj.chained_graph.representative_holdoff;
            if gamma >= numel(obj.chained_graph.time_scope)
                te_ind = 1;
            else
                te_ind = numel(obj.chained_graph.time_scope) - gamma;
            end            
            end_time = obj.chained_graph.subgraph(te_ind).time;                        
            
            representative_times = start_time:parent_scale:end_time;
            representatives = cell(1, numel(representative_times - 1));
            relation = obj.chained_graph.CreateRelation('base', 'base');
            for i = 1:numel(representative_times) - 1
               
                relation.observer_time = representative_times(i);
                relation.target_time = representative_times(i + 1);                
                
                representatives{i} = obj.chained_graph.Extract(relation);
                
            end
            
            obj.leader.PushMeasurements(representatives);
            
        end        
        
    end
    
end