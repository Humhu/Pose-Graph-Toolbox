% Represents all information at a particular time
classdef WorldState2D
    
    properties
        
        poses;          % Poses at this time step
        ids;            % IDs corresponding to poses
        measurements;   % Measurements at this time step
        time;           % Time step or time
        
    end
    
    methods
        
        function [obj] = WorldState2D()
            
            if nargin == 0
                return
            end
            
        end
        
        % Returns
        function [d] = GetDimension(obj)
            if isempty([obj(1).poses])
                d = 0;
            else
                d = size(obj(1).poses, 2);
            end
        end
        
        % Returns time span
        function [l] = GetLength(obj)
            
            if numel(obj) < 2
                l = 0;
                return
            end
            times = [obj.time];
            l = max(times) - min(times);
            
        end
        
        % Build ID and time to index SearchMap objects
        function [idMap, tMap] = BuildMaps(obj)
            
            % Map ID-indices
            pinds = 1:numel(obj(1).ids);
            idMap = SearchMap(obj(1).ids, pinds);
            
            % Map time-indices
            times = [obj.time];
            tinds = 1:numel(times);
            tMap = SearchMap(times, tinds);
            
        end
        
        function [rotated] = Zero(obj)
           
            rotated = obj;
            zero_pose = obj(1).poses(:,1);
            a = zero_pose(3);
            R = [cos(a), sin(a);
                 -sin(a), cos(a)];
             
            N = rotated.GetDimension();
            T = numel(rotated);
            p = [rotated.poses];
            p = bsxfun(@minus, p, zero_pose);
            p(1:2,:) = R*p(1:2,:);
            for i = 1:T
               rotated(i).poses = p(:,(N*(i-1) + 1):(N*i));
            end
            
        end
        
    end
    
end