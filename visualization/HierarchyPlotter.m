% Extends a Plotter2D to visualize hierarchies

classdef HierarchyPlotter < Plotter2D
    
    properties
        
    end
    
    methods
        
        function [obj] = HierarchyPlotter(dims)
            
            obj = obj@Plotter2D(dims);
            obj.Clear();
            
            zlabel(obj.axe, 'k');
            
        end
        
        function Clear(obj)
            
            Clear@Plotter2D(obj);
            
        end
        
        % Update to take root as argument instead
        function PlotHierarchy(obj, robots, state)
            
            [obj.idMap, ~] = state.BuildMaps();
            obj.z_scale = 0.5;
            mdepth = 0;
            
            for i = 1:numel(robots)
                
                jobs = robots(i).roles;
                
                for j = 1:numel(jobs)
                    
                    job = jobs(j);
                    mdepth = max(mdepth, job.level);
                    
                    if isempty(job.followers)
                        continue;
                    end
                    
                    if isempty(job.leader)
                        pose = robots(obj.idMap.Forward(job.ownerID)).pose;
                        obj.PlotRobot(pose, job.level, 'r', 1.0);
                        obj.PlotLabel(pose, job.level, num2str(job.ownerID));
                    end
                    
                    obj.PlotFollowers(robots, job);
                    
                end
                
            end
            
            set(obj.axe, 'ZTick', 0:mdepth);
            
        end
        
        function PlotBeliefs(obj, root)
            
            if isa(root, 'HierarchyRole')
                root = root.chained_graph;
            end
            
            r = root;
            hold(obj.axe, 'on');
            
            while(~isempty(r))
                curr = r(1);
                r(1) = [];
                curr = curr.ApplyLinks(curr.depth+1);
                graph = curr.subgraph;
                c = obj.colors(curr.depth+1, :);
                obj.PlotGraph(graph, c);
                r = [r, curr.children];
            end
            
        end
        
        function PlotLatestBeliefs(obj, root)
            
            if isa(root, 'HierarchyRole')
                root = root.chained_graph;
            end
            
            r = root;
            hold(obj.axe, 'on');
            
            while(~isempty(r))
                curr = r(1);
                r(1) = [];
                if isempty(curr.children)
                    curr = curr.ApplyLinks(curr.depth+1);
                    graph = curr.subgraph(end);
                    c = [0,0,0]; % black
                    obj.PlotGraph(graph, c);
                else
                    r = [r, curr.children];
                end
            end
            
        end
        
        function PlotTruthOverlay(obj, root, truth)
            
            D = size(obj.colors,1); 
            curr = root;
            
            [tidMap, ttMap] = truth.BuildMaps();
                        
            obj.PlotLatestBeliefs(root);
            obj.PlotGraph(truth(end), [1,0,1]);            
            
            for i = 1:D
                
                curr_cg = curr.chained_graph;
                gframe_curr_cg = curr_cg.ApplyLinks(curr_cg.depth+1);
                
                [idMap, tMap] = curr_cg.subgraph.BuildMaps();
                curr_idind = idMap.Forward(curr_cg.base_id);
                curr_tind = tMap.Forward(curr_cg.base_time);
                curr_bpose = gframe_curr_cg.subgraph(curr_tind).poses(:,curr_idind);
                
                t_idind = tidMap.Forward(curr_cg.base_id);
                t_tind = ttMap.Forward(curr_cg.base_time);
                t_bpose = truth(t_tind).poses(:,t_idind);
                
                dx = curr_bpose(1:2) - t_bpose(1:2);
                da = wrapToPi(curr_bpose(3) - t_bpose(3));
                shifted_truth = truth.Shift(-t_bpose(1:2));
                shifted_truth = shifted_truth.Rotate(da);
                shifted_truth = shifted_truth.Shift(t_bpose(1:2));
                shifted_truth = shifted_truth.Shift(dx);
                                
                obj.PlotGraph(shifted_truth(end), obj.colors(i,:));
                
                if ~isempty(curr.followers)
                    curr = curr.followers(1);
                end
                
            end
        end
        
    end
    
    methods(Access = protected)
        
        function PlotGraph(obj, sequence, c)
            
            [idMap, tMap] = sequence.BuildMaps();
            
            for t = 1:numel(sequence)
                
                state = sequence(t);
                true_t = tMap.Backward(t);
                
                % Plot robot positions
                for i = 1:state.GetDimension()
                    p = state.poses(:,i);
                    id = state.ids(i);
                    obj.PlotRobot(p, true_t, c, 1.0);
                    obj.PlotLabel(p, true_t, num2str(id));
                end
                
                % Plot measurements
                for i = 1:numel(state.measurements)
                    m = state.measurements{i};
                    obs_t = tMap.Forward(m.observer_time);
                    obs_id = idMap.Forward(m.observer_id);
                    if isempty(obs_t) || isempty(obs_id)
                        continue
                    end
                    obs_p = sequence(obs_t).poses(:,obs_id);
                    tar_p = m.ToPose(obs_p);
                    obj.PlotLine([obs_p(1:2); m.observer_time], ...
                        [tar_p(1:2); m.target_time], {'Color', c});
                end
                
            end
            
        end
        
        function PlotFollowers(obj, robots, role)
            
            lpose = robots(obj.idMap.Forward(role.ownerID)).pose;
            lloc = lpose;
            lloc(3) = role.level;
            
            for i = 1:numel(role.followers)
                
                f = role.followers(i);
                fpose = robots(obj.idMap.Forward(f.ownerID)).pose;
                floc = fpose;
                floc(3) = f.level;
                
                obj.PlotLine(lloc, floc);
                obj.PlotRobot(fpose, f.level, 'r', 1.0);
                obj.PlotLabel(fpose, f.level, num2str(f.ownerID));
                
            end
            
        end
        
    end
    
    
end