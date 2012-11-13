% Sensor superclass
classdef Sensor < handle
    
    properties(Abstract)
        
        ownerID;       % ID of agent this sensor corresponds to
        mean;           % Sensor noise characteristics
        covariance;
        
    end
    
    methods(Abstract)
        
        % Produce measurements of the world
        [measurements] = GenerateMeasurements(obj, state);        
        
    end
    
end