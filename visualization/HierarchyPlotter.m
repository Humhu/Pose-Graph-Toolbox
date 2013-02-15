% Extends a Plotter2D to visualize hierarchies

classdef HierarchyPlotter < Plotter2D
    
    properties
        
        idMap;
        
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
           
            
            
        end
        
    end
    
    methods(Access = protected)
        
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