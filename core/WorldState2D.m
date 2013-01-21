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
        
        function [d] = GetDimension(obj)
            if isempty(obj.poses)
                d = 0;
            else
                d = size(obj.poses, 2);
            end
        end
        
    end
    
end