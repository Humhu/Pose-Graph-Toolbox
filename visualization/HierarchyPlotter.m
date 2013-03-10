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
%                 graph = curr.subgraph;
%                 graph_root_pos = curr.estimates(1);                
%                 graph = graph.Rotate(graph_root_pos.rotation);
%                 graph = graph.Shift(graph_root_pos.displacement);
                obj.PlotGraph(graph);
                r = [r, curr.children];
            end
            
        end
        
    end
    
    methods(Access = protected)
        
        function PlotGraph(obj, sequence)
            
            [idMap, tMap] = sequence.BuildMaps();
                     
            for t = 1:numel(sequence)
                
                state = sequence(t);
                true_t = tMap.Backward(t);
                
                % Plot robot positions
                for i = 1:state.GetDimension()
                    p = state.poses(:,i);
                    id = state.ids(i);
                    %c = obj.colors(idMap.Forward(id),:); % FIX!!
                    c = obj.colors(id + 1, :);
                    obj.PlotRobot(p, true_t, c, 1.0);
                    obj.PlotLabel(p, true_t, num2str(id));
                end
                
                % Plot measurements
                for i = 1:numel(state.measurements)
                   m = state.measurements{i};
                   obs_t = tMap.Forward(m.observer_time);
                   obs_id = idMap.Forward(m.observer_id);                   
                   
                   obs_p = sequence(obs_t).poses(:,obs_id);
                   tar_p = m.ToPose(obs_p);
                   color = obj.colors(obs_id,:);
                   obj.PlotLine([obs_p(1:2); m.observer_time], ...
                       [tar_p(1:2); m.target_time], {'Color', color});
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