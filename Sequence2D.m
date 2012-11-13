% Represents a time sequence of poses and measurements
classdef Sequence2D < handle
    
    properties
        
        fullStates; % Full state data
        times;      % Times corresponding to states - currently unsupported
        
    end
    
    methods
   
        function [obj] = Sequence2D()                        
            
            if nargin == 0
                return
            end
            
        end
        
        function [] = Append(obj, fs)
           
            if ~isa(fs, 'WorldState2D')
                return
            end
            
            obj.fullStates = [obj.fullStates, fs];
            
        end
        
        function [seq] = GetStates(obj, range)
           
            seq = obj.fullStates(range);
            
        end
        
    end
    
end