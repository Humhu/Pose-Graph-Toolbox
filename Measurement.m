% Superclass container for measurements
classdef (Abstract) Measurement
    
    properties (Abstract)
        
        observer_id;    % World ID of agent that produced this observation.
        target_id;      % World ID of agent being observed. -1 if unknown.
        
    end
    
end