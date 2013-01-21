% A hierarchical role object
% Contains all information required to perform role's duties
classdef HierarchyRole < handle
    
    properties
    
        level;      % Rank of role (root level = 0)
        time_scale; % Time step size
        
        leader;     % Reference to leader HierarchyRole object
        followers;  % Array of follower HierarchyRole objects
        
        beliefs;    % Sequence2D object representing set of beliefs
        
    end
    
    methods
        
        function [obj] = HierarchyRole(level)
            obj.level = level;
        end
        
        % Process new data
        function Update(obj, meaurements)
            
            
            
        end
        
    end
    
end