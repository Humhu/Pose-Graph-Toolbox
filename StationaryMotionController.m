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
            
        end
        
        function [u] = GenerateOutputs(obj, beliefs);
            
            u =  zeros(3,1);
            
        end
       
        function [newObj] = Copy(obj)
            
            newObj = StationaryMotionController(obj);
            
        end
        
    end
    
end