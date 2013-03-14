classdef StraightMotionController < handle & MotionController
    
    properties
        
        ownerID;
        
        ref; % ref start        
        ref_direction; % line angle in world coordinates
        
        k_normal;
        velocity;
        
        
    end
    
    methods
        
        function [obj] = StraightMotionController(smc)
            
            if nargin == 0
                return;
            end
            
            if ~isa(smc, 'StraightMotionController')
                return
            end
            
            obj.ownerID = smc.ownerID;
            obj.ref = smc.ref;
            obj.ref_direction = smc.ref_direction;
            obj.k_normal = smc.k_normal;
            obj.velocity = smc.velocity;
            
        end
        
        function Initialize(obj, state)
           
            [idMap, ~] = state.BuildMaps;
            pose = state(1).poses(:, idMap.Forward(obj.ownerID));
            obj.ref = pose;
            
        end       
        
        function [u] = GenerateOutputs(obj, beliefs)
            
            bel_x = beliefs(1:2);
            bel_a = beliefs(3);
            
            e_x = bel_x - obj.ref(1:2);
            R = [cos(obj.ref_direction), -sin(obj.ref_direction);
                 sin(obj.ref_direction), cos(obj.ref_direction)];
            e_x = R*e_x;            
            v = [1; -obj.k_normal*e_x(2)];             
            v = R'*v;
            
            v(3) = -wrapToPi(bel_a - obj.ref(3));                        
            v = obj.velocity*v/norm(v);            
            
            R = [cos(bel_a), sin(bel_a);
                -sin(bel_a), cos(bel_a)];
            u = R*v(1:2);
            u(3) = v(3);
            
        end
        
        function [newObj] = Copy(obj)
           
            newObj = StraightMotionController(obj);
            
        end
        
    end
    
end
