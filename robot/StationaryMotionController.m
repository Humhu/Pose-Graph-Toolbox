classdef StationaryMotionController < handle & MotionController
    
    properties
        
        ownerID;
        ref;
        
    end
    
    methods
        
        function [obj] = StationaryMotionController(smc)
            
            if nargin == 0
                return;
            end               
            
            if ~isa(smc, 'StationaryMotionController')
                return
            end
            
            obj.ownerID = smc.ownerID;
            obj.ref = smc.ref;
            
        end
        
        function [u] = GenerateOutputs(obj, beliefs)
            
            u =  obj.ref;
            
        end
       
        function [newObj] = Copy(obj)
            
            newObj = StationaryMotionController(obj);
            
        end
        
    end
    
end