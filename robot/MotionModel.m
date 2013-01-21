% Represents a robot's physical motion model
classdef MotionModel < handle
   
    properties(Abstract)
        
        ownerID;        % ID of agent this model belongs to
        input_dims;     % Input space dimensionality M
        output_dims;    % Output space dimensionality N
        input_limits;   % Input space limits [upper, lower] M x 2
        output_limits;  % Output space limits
        input_wrapping; % Input space wrapping (boolean)
        output_wrapping;% Output space wrapping (boolean)
        
    end
    
    methods(Abstract)
        
        % Apply control inputs to current pose to generate motion
        % Should apply necessary limits and wrappings
        [newPose] = GenerateMotion(obj, currentPose, inputs);
        
    end
    
end