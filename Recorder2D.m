% Time sequence recording object
classdef Recorder2D < handle
    
    properties
        
        poses;
        times;
        
    end
    
    methods
        
        function obj = Recorder2D(N)
            if nargin == 0
                return
            end
            obj.poses = Pose2D.empty(N,0);
            
        end
        
        function [] = Append(obj, p)
           
            if size(p,1) ~= size(obj.poses,1)
                return
            end
            
            obj.poses = [obj.poses, p];
            
        end                
        
        function [p] = GetPoses(obj)
            
            p = obj.poses;
            
        end
        
    end
    
end