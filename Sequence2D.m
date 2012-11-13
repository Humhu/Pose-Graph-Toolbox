% Represents a time sequence of poses and measurements
% Currently fixed sized for better efficiency
% 
% Sequences are intended to be generated for known-length runs of a
% simulator. Sequences can be appended to each other for unknown-length
% runs.
classdef Sequence2D < handle
    
    properties
        
        states;     % World state data        
        i;          % State writing index
        
    end
    
    methods
   
        function [obj] = Sequence2D(N)                        
            
            if nargin == 0
                return
            end
            
            obj.i = 1;
            obj.states = WorldState2D;
            obj.states(1,N) = WorldState2D;
            
        end
        
        function [] = Write(obj, fs)                       
            
            % Stop writing if full
            if obj.i > size(obj.states,2)
                return
            end
            obj.states(obj.i) = fs;
            obj.i = obj.i + 1;
            
        end                
        
        %TODO Make more efficient?
        function [newSeq] = Append(obj, seq)
           
            if isempty(seq)
                newSeq = obj;
                return
            end
            
            n = size(obj.states,2);
            m = size(seq.states,2);
            newSeq = Sequence2D(n + m);
            newSeq.states(1:n) = obj.states;
            newSeq.states(n + 1: n + m) = seq.states;
            
        end        
        
        function [errs] = Difference(obj, seq)
           
            if size(obj.states) ~= size(seq.states)
                error('Sequences must be of same length and dimension');                
            end
            
            N = size(obj.states,1); 
            T = size(obj.states,2);
            
            dp = [obj.states.poses] - [seq.states.poses];            
            errs = double(dp);
            
        end
        
    end
    
end