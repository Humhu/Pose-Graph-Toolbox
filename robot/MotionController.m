% Represents a robot's motion controller
classdef MotionController < handle
   
    properties(Abstract)              
        
        ownerID;    % ID of agent this controller belongs to
        ref;        % Controller reference
        
    end
    
    methods(Abstract)
       
        % Produce control outputs 
        [u] = GenerateOutputs(obj, beliefs); 
       
        % Initialize if needed
        Initialize(obj, state);
        
    end
    
end