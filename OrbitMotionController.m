% Controller that makes the robot orbit around the world origin
% Currently assuming robot is holonomic, so control outputs correspond to
% state changes
% TODO: Make outputs in robot frame, not global frame?
% TODO: Move copy method to abstract superclass?
classdef OrbitMotionController < handle & MotionController
    
    properties
        
        ownerID;
        ref;
        motionGain;
        
    end
    
    methods
   
        function [obj] = OrbitMotionController(omc)
            
            if nargin == 0
                return;
            end
            
            obj.ref = omc.ref;
            obj.motionGain = omc.motionGain;
            
        end
        
        % TODO: For now, beliefs is robot pose. Needs to be updated to be more general!
        function [u] = GenerateOutputs(obj, beliefs)
           
            radius = norm(beliefs(1:2));
            da = obj.motionGain/radius;
            R = [cos(da), -sin(da);
                sin(da), cos(da)];       
            
            movement = (R - eye(2))*beliefs(1:2);            
            u = [movement; da];
            
        end
        
        function [newObj] = Copy(obj)
           
            newObj = OrbitMotionController(obj);
            
        end
        
    end
    
end