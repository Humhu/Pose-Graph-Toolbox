% Relative pose sensor
% Corresponds to Lidar scan matching outputs
classdef RelativePoseSensor < handle & Sensor
    
    properties
        
        ownerID;
        maxRange;
        mean;
        covariance;
        
    end
    
    methods
        
        function [obj] = RelativePoseSensor(rps)
            
            if nargin == 0
                return;
            end
            
            obj.ownerID = rps.ownerID;
            obj.maxRange = rps.maxRange;
            obj.mean = rps.mean;
            obj.covariance = rps.covariance;
            
        end
        
        function [newObj] = Copy(obj)
            
            newObj = RelativePoseSensor(obj);
            
        end
        
        % TODO: Vectorize?
        function [measurements] = GenerateMeasurements(obj, state)
            
            N = size(state.poses, 2);
            % TODO: Add ID search instead of index
            [idMap, ~] = state.BuildMaps();
            p = state.poses(:, idMap.Forward(obj.ownerID));
            measurements = {};
            
            for i = 1:N
                
                target_id = state.ids(i);
                if target_id == obj.ownerID
                    continue
                end
                target_pose = state.poses(:, i);
                rel = target_pose - p;
                rel(3) = wrapToPi(rel(3));
                range = norm(rel(1:2));
                if range > obj.maxRange
                    continue
                end
                
                cov = range*obj.covariance;
                m = MeasurementRelativePose(p, target_pose, cov);
                m.observer_id = obj.ownerID;
                m.target_id = target_id;
                m.observer_time = state.time;
                m.target_time = state.time;
                measurements = [measurements; {m}];
                
            end
            
        end
        
        
    end
    
end