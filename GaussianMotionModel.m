% Standard Gaussian motion model
% TODO: Implement transition matrix
classdef GaussianMotionModel < handle & MotionModel
   
    properties
       
        ownerID;
        transitionMatrix; %Currently unused?
        mean;
        covariance;
        
    end
    
    methods
       
        function [obj] = GaussianMotionModel(gmm)
           
            if nargin == 0
                return;
            end
            
            obj.transitionMatrix = gmm.transitionMatrix;
            obj.mean = gmm.mean;
            obj.covariance = gmm.covariance;
            
        end
        
        function [newPose] = GenerateMotion(obj, currentPose, inputs)
           
            noise = mvnrnd(obj.mean, obj.covariance)';
            newPose = currentPose + reshape(inputs + noise, 1, 1, 3);
            
        end
        
        function [newObj] = Copy(obj)
           
            newObj = GaussianMotionModel(obj);
            
        end
        
    end
    
end