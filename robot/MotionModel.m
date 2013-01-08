% Represents a robot's physical motion model
classdef MotionModel < handle
   
    properties(Abstract)
        
        ownerID;        % ID of agent this model belongs to
        
    end
    
    methods(Abstract)
        
        % Apply control inputs to current pose to generate motion
        [newPose] = GenerateMotion(obj, currentPose, inputs);
        
    end
    
end