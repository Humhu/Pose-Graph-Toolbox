% Controller that moves the robot randomly
classdef RandomMotionController < handle & MotionController
    
    properties
        
        ref;
        mean;
        covariance;
        motionGain;
        
    end
    
    methods
        
        function [obj] = RandomMotionControl(rmc)
            
            if nargin == 0
                return;
            end
            
            obj.ref = rmc.ref;
            obj.motionGain = rmc.motionGain;
            
        end
        
        function [u] = GenerateOutputs(obj, ~)
           
            u = mvnrnd(obj.mean, obj.covariance);
            u = obj.motionGain*(u/norm(u));
            
        end
        
    end
    
end