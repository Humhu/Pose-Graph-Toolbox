% Represents a time sequence of poses and measurements
% Currently fixed sized for efficiency
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
        
        function [newObj] = Copy(obj)
            
            L = obj.GetLength();
            newObj = Sequence2D(L);
            newObj.states(1:L) = obj.states(1:L);
            newObj.i = obj.i;
            
        end
        
        % Add a state entry to last available slot
        function [] = Write(obj, fs)                       
            
            % Stop writing if full
            if obj.i > size(obj.states,2)
                return
            end
            obj.states(obj.i) = fs;
            obj.i = obj.i + 1;
            
        end                
        
        function [] = Clear(obj)
           obj.i = 1; 
        end
        
        % Number of agents recorded
        function [d] = GetDimension(obj)
           d = size(obj.states(1).poses, 2); 
        end
        
        % Number of time steps recorded
        function [l] = GetLength(obj)            
            l = obj.i - 1;
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
            newSeq.i = n + m + 1;
            
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
        
        function [pD] = GetPosesDouble(obj)
           
            pD = double([obj.states.poses]);
            
        end
        
    end
    
end