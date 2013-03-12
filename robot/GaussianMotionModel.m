% Standard Gaussian motion model
% TODO: Implement transition matrix
classdef GaussianMotionModel < handle & MotionModel
    
    properties
        
        ownerID;
        transitionMatrix; %Currently unused?
        mean;
        covariance;
        
        input_dims;     % Input space dimensionality M
        output_dims;    % Output space dimensionality N
        input_limits;   % Input space limits [upper, lower] M x 2
        output_limits;  % Output space limits
        input_wrapping; % Input space wrapping (boolean)
        output_wrapping;% Output space wrapping (boolean)
        
    end
    
    methods
        
        function [obj] = GaussianMotionModel(gmm)
            
            if nargin == 0
                return;
            end
            
            obj.transitionMatrix = gmm.transitionMatrix;
            obj.mean = gmm.mean;
            obj.covariance = gmm.covariance;
            
            obj.input_dims = gmm.input_dims;
            obj.output_dims = gmm.output_dims;
            obj.input_limits = gmm.input_limits;
            obj.output_limits = gmm.output_limits;
            obj.input_wrapping = gmm.input_wrapping;
            obj.output_wrapping = gmm.output_wrapping;
            
        end
        
        function [newPose] = GenerateMotion(obj, currentPose, inputs)
            
            % Input limits
            over = inputs > obj.input_limits(:,1);
            inputs(over) = obj.input_limits(over,1);
            under = inputs < obj.input_limits(:,2);
            inputs(under) = obj.input_limits(under,2);
            
            % TODO: Input wrapping?
            input_mag = norm(inputs);
            noise = mvnrnd(obj.mean, sqrt(input_mag)*obj.covariance)';
            a = currentPose(3);
            R = [cos(a), -sin(a);
                sin(a), cos(a)];
            noise(1:2) = R*noise(1:2);
            newPose = currentPose + inputs + noise;
            
            % Output limits
            over = newPose > obj.output_limits(:,1);
            newPose(over) = obj.output_limits(over,1);
            under = newPose < obj.output_limits(:,2);
            newPose(under) = obj.output_limits(under,2);
            
            % TODO: Output wrapping
            newPose(obj.output_wrapping) = wrapToPi(newPose(obj.output_wrapping));
            
        end
        
        function [newObj] = Copy(obj)
            
            newObj = GaussianMotionModel(obj);
            
        end
        
    end
    
end