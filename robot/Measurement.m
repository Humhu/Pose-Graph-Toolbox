% Superclass container for measurements
classdef (Abstract) Measurement
    
    properties (Abstract)
        
        observer_id;    % World ID of agent that produced this observation.
        target_id;      % World ID of agent being observed. -1 if unknown.
        observer_time;  % Time step for observer
        target_time;    % Time step for target
        
    end
    
    methods
       
        % Checks to see if two measurements involve the same IDs
        function [match] = SameAgents(obj, other)
            match = obj.observer_id == other.target_id && ...
                    obj.target_id == other.target_id;
        end
        
        % Checks to see if two measurements are at the same times
        function [match] = SameTimes(obj, other)
            match = obj.observer_time == other.observer_time && ...
                    obj.target_time == other.target_time;
        end
        
        % Checks to see if two measurements are at the same time and
        % involve the same IDs
        function [match] = SameRelation(obj, other)
           match = obj.SameAgents(other) && obj.SameTimes(other);
        end
        
    end
    
end