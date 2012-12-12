% Represents a simulated, bounded planar world populated by robots
% Time is discrete
classdef World2D < handle
    
    properties
        
        dims;           % World size centered at 0
        robots;         % Robot objects handles   
        state           % Current state info
        
    end
    
    methods
        
        function obj = World2D(a)
            
            if nargin == 0
                return
            end
            
            obj.state = WorldState2D;
            obj.state.time = 1;
            
            if isa(a, 'double')
                obj.dims = reshape(a, 2, 1);
                return
            end
            
            if ~isa(a, 'World2D')
                display('Invalid argument type');
                return
            end
            
            obj.state = a.state;
            obj.dims = a.dims;
            N = numel(a.robots);
            obj.robots = Robot;
            obj.robots(N,1) = Robot;
            
            for i = 1:N
               obj.robots(i) = Robot(a.robots(i)); 
            end
            
        end
        
        function AddRobots(obj, robs)
           
            if ~isa(robs, 'Robot')
                return
            end
            
            for i = 1:numel(robs)
               obj.robots = [obj.robots; robs(i)]; 
            end
            
            obj.state.ids = obj.GetIDs();
            obj.state.poses = obj.GetPoses();
            obj.GenerateMeasurements();
            
        end
        
        function [] = Step(obj)
           
            N = numel(obj.robots);
            for i = 1:N
               
                r = obj.robots(i);
                r.Step(obj.state);
                
            end                        
            
            obj.state.time = obj.state.time + 1;
            obj.state.poses = obj.GetPoses();
            obj.GenerateMeasurements();
            
        end
        
        function [state] = GetState(obj)
           
            state = obj.state;
            
        end
        
        function [measurements] = GetMeasurements(obj)
            
            measurements = obj.state.measurements;
            
        end
        
        function [IDs] = GetIDs(obj)
           
            IDs = reshape([obj.robots.ID], size(obj.robots));
            
        end
        
        function [poses] = GetPoses(obj)
            
            poses = reshape([obj.robots.pose], 3, numel(obj.robots));
            
        end
       
        function [err] = GetErrors(obj, w)
            
            if ~isa(w, 'World2D')
                display('Invalid argument type')
            end
            
            err = double([obj.robots.pose] - [w.robots.pose]);
        end
        
    end
    
    methods(Access = private)
        
        function [] = GenerateMeasurements(obj)
           
            obj.state.measurements = {};                        
            for i = 1:numel(obj.robots)
                
                m = obj.robots(i).GetMeasurements(obj.state);
                obj.state.measurements = [obj.state.measurements; m];
                
            end
            
        end
        
    end
    
end