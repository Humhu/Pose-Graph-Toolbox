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
        
        chained_graph; % Estimator data structure
        translate_buff; % Buffer of measurements to translate
        
        last_rep_transmit; % Last time we transmitted up
        last_sent_rep_time;     % Latest time of the last set of reps we sent
        last_chain_transmit; % Last time we transmitted down
        last_sent_chain_time;    % Latest time of the last chains we sent out
        
        comms;      % Post office handle
        commID;     % Post office ID
        
    end
    
    methods
        
        function [obj] = HierarchyRole(level, time_scale, time_overlap, ...
                chold, rhold)
            
            obj.chained_graph = ChainedGraph(level, time_scale, time_overlap, ...
                chold, rhold);
            obj.higher_beliefs = MeasurementRelativePose.empty(1, 0);
            obj.time = 0;
            obj.last_rep_transmit = 0;
            obj.last_sent_rep_time = 0;
            obj.last_chain_transmit = 0;
            obj.last_sent_chain_time = 0;
            obj.translate_buff = BinBuffer(100);
            
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
            new.translate_buff = obj.translate_buff.Copy();
            
        end
        
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
        
        % Initialize agent's local_beliefs
        function Initialize(obj, state, chain)
            
            % For root case
            if nargin == 2
                global_pose = state.poses(:,1);
                global_belief = MeasurementRelativePose(zeros(3,1), global_pose, zeros(3));
                global_belief.observer_id = -1;
                global_belief.observer_time = 0;
                global_belief.target_id = obj.ownerID;
                global_belief.target_time = 0;
                global_belief.covariance = 1E-6*eye(3);
                chain = global_belief;
            end
            
            obj.chained_graph.chain = chain;
            obj.BuildEstimates(obj.chained_graph.chain);
            
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
            obj.chained_graph.SetBase(substate.ids(1), substate.time);
            
            obj.time = 0;
            
            % Recursive initialization and chain construction
            relation = obj.chained_graph.CreateRelation('base', 'base');
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                relation.target_id = f.ownerID;
                
                z = obj.chained_graph.Extract(relation);
                new_chain = [obj.chained_graph.chain, z];
                f.Initialize(state, new_chain);
            end
            
        end
        
        % Update the subgraph chain in response to receiving new links
        function ChainUpdate(obj, update)
            
            % Last link in update needs to be shifted to local base
            obj.chained_graph.chain = update;            
            obj.chained_graph.UpdateLink();                        
            
            % Update local position estimates
            obj.BuildEstimates(obj.chained_graph.chain);
            
            % We leave some overlap when contracting            
            if ~isempty(obj.chained_graph.parent)
                parent_scale = obj.chained_graph.parent.time_scale;
            else
                parent_scale = obj.chained_graph.time_scale;
            end            
            alpha = obj.chained_graph.time_overlap;
            start_time = update(end).target_time - alpha*parent_scale;
            obj.chained_graph.Contract(start_time);
            
        end
        
        % Translates all buffered measurements to the most recent
        % representative - should be the current base since we only call
        % this right before we transmit representatives
        function TranslateMeasurements(obj)
            
            all_meas = obj.translate_buff.PopAll();
            
            for i = 1:numel(all_meas)
            
                m = all_meas{i};
                
                % Find reciprocal role
                reciprocal = FindReciprocal(obj, m.target_id);
                lca = FindLCA(obj, reciprocal);
                
                if mod(m.observer_time, lca.chained_graph.time_scale) ~= 0
                    continue;
                end
                
                local_rel = obj.chained_graph.CreateRelation('base', [m.observer_id, m.observer_time]);                
                rep_to_local = obj.chained_graph.Extract(local_rel);
                recip_rel = reciprocal.chained_graph.CreateRelation([m.target_id, m.target_time], 'base');                
                local_to_recip = reciprocal.chained_graph.Extract(recip_rel);
                
                m = rep_to_local.Compose(m);
                m = m.Compose(local_to_recip);
                messg = MeasurementUpdateMessage(obj.leader.commID, obj.commID, m);
                messg.sendTime = obj.time;
                obj.comms.Deposit(obj.commID, {messg});
            
            end
        end
        
        % Process internal functions. Should be called once every time
        % step.
        function Step(obj)                        
            
            % Split messages
            all_messages = obj.comms.Withdraw(obj.commID);
            local_measurements = {};
            if ~isempty(all_messages)
                measurement_messages = Message.GetType(all_messages, 'MeasurementUpdate');
                measurement_messages = Message.SortSendTime(measurement_messages);
                chain_updates = Message.GetType(all_messages, 'ChainUpdate');
                chain_updates = Message.SortSendTime(chain_updates);
            else
                measurement_messages = {};
                chain_updates = {};
            end
            
            times = [];
            team_ids = obj.GetTeam();
            for i = 1:numel(measurement_messages)
                m = measurement_messages{i};
                z = m.contents;
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    %oos_measurements = [oos_measurements, {z}];
                    obj.translate_buff.Push(z);
                else
                    times = [times, z.observer_time, z.target_time];
                    local_measurements = [local_measurements, {z}];
                end
            end
            latest_t = max(times);
            
            if ~isempty(local_measurements)
                
                % Extend to cover new times, then incorporate and optimize
                obj.chained_graph.Extend(latest_t);
                
                % Incorporate and optimize                
                obj.chained_graph.Incorporate(local_measurements);
                obj.chained_graph.Optimize();
                
                % Update chain
                obj.chained_graph.SetBase(obj.chained_graph.base_id, latest_t);
                obj.chained_graph.UpdateLink();  
                
                if obj.chained_graph.depth == 0
                   obj.ChainUpdate(obj.chained_graph.chain); 
                end
                
            end
            
            if ~isempty(chain_updates)
                z = chain_updates{end}.contents(end);
                fprintf(['\tChain update received. s_id: ', num2str(z.observer_id), ...
                    ' s_t: ', num2str(z.observer_time), ' e_id: ', num2str(z.target_id), ...
                    ' e_t: ', num2str(z.target_time), '\n']);
                obj.ChainUpdate(chain_updates{end}.contents);
            end                                   
            
        end
        
        % This step called after all other graphs have stepped
        function CommStep(obj)
            
            % Chain transmit
            if ~isempty(obj.followers)
                time_passed = obj.time - obj.last_chain_transmit >= obj.chained_graph.time_scale;
                interesting = obj.chained_graph.time_scope(end) ~= obj.last_sent_chain_time;
                if time_passed && interesting
                    fprintf('\tCG(%d,%d) transmitting down\n', obj.ownerID, ...
                        obj.chained_graph.depth);
                    obj.PushChains();
                    obj.comms.Transmit(obj.commID);
                    obj.last_chain_transmit = obj.time;
                    obj.last_sent_chain_time = obj.chained_graph.time_scope(end);
                end
            end
               
            % Representative & measurement transmit              
            if ~isempty(obj.leader)
                time_passed = obj.time - obj.last_rep_transmit >= obj.leader.chained_graph.time_scale;
                interesting = obj.chained_graph.time_scope(end) - obj.last_sent_rep_time ...
                    >= obj.leader.chained_graph.time_scale;
                if time_passed && interesting
                    fprintf('\tCG(%d,%d) transmitting up\n', obj.ownerID, ...
                        obj.chained_graph.depth);
                    obj.PushRepresentatives();
                    obj.TranslateMeasurements();
                    obj.comms.Transmit(obj.commID);
                    obj.last_rep_transmit = obj.time;
                    obj.last_sent_rep_time = obj.chained_graph.time_scope(end);
                end
            end                        
                                    
            obj.time = obj.time + 1;
            
        end
        
    end
    
    methods(Access = private)
        
        % Builds this agent's root position in all higher reference frames
        % given a chain of relative local_measurements. The chain should be
        % ordered with local_measurements(1) corresponding to root->f1, and
        % local_measurements(end) corresponding to fn->obj
        function BuildEstimates(obj, local_measurements)
            
            if numel(local_measurements) ~= obj.chained_graph.depth + 1
                error(['Wrong number of local_measurements given to BuildEstimates: ', ...
                    'level: %d, #meas: %d'], obj.chained_graph.depth, ...
                    numel(local_measurements));
            end
            
            % Initialize all as last link
            for i = 1:obj.chained_graph.depth + 1
                obj.estimates(i) = local_measurements(end);
            end
            
            % Build chain by composing local_measurements in reverse-order
            for i = obj.chained_graph.depth:-1:1
                m = local_measurements(i);
                for j = i:-1:1
                    obj.estimates(j) = m.Compose(obj.estimates(j));
                end
            end
            
        end
        
        % Extract and send new links to followers
        function PushChains(obj)
            
            relation = obj.chained_graph.CreateRelation('base', 'base');
            
            beta = obj.chained_graph.chain_holdoff;
            ages = obj.time - obj.chained_graph.time_scope;
            t_ind = find(ages >= beta, 1, 'last');
            if ~isempty(t_ind)
                relation.target_time = obj.chained_graph.time_scope(t_ind);
            end
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                relation.target_id = f.ownerID;
                
                z = obj.chained_graph.Extract(relation);
                new_chain = [obj.chained_graph.chain, z];
                m = ChainUpdateMessage(f.commID, obj.commID, new_chain);
                m.sendTime = obj.time;
                obj.comms.Deposit(obj.commID, {m});
            end
            
        end
        
        function PushRepresentatives(obj)
            
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
            N = numel(representative_times) - 1;
            messages = cell(1,N);
            relation = obj.chained_graph.CreateRelation('base', 'base');
            
            for i = 1:numel(representative_times) - 1
                
                relation.observer_time = representative_times(i);
                relation.target_time = representative_times(i + 1);
                
                m = MeasurementUpdateMessage(obj.leader.commID, obj.commID, ...
                    obj.chained_graph.Extract(relation));
                m.sendTime = obj.time;
                messages{i} = m;
                
            end
            
            obj.comms.Deposit(obj.commID, messages);
            
        end
        
    end
    
end