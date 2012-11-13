% Represents a robot's motion controller
classdef MotionController < handle
   
    properties(Abstract)              
        
        ref;        % Controller reference
        
    end
    
    methods(Abstract)
       
        % Produce control outputs 
        [u] = GenerateOutputs(obj, beliefs); 
        
    end
    
end