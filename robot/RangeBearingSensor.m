% Range bearing sensor
% Corresponds to visual localization
classdef RangeBearingSensor < handle & Sensor
    
    properties
    
        ownerID;
        maxRange;
        mean;
        covariance;
        
    end
    
    methods
        
        function [obj] = RangeBearingSensor(rbs)
            
            if nargin == 0
                return;
            end
            
            obj.ownerID = rbs.ownerID;
            obj.maxRange = rbs.maxRange;
            obj.mean = rbs.mean;
            obj.covariance = rbs.covariance;
            
        end
        
        function [newObj] = Copy(obj)
            
            newObj = RangeBearingSensor(obj);
            
        end
        
        function [measurements] = GenerateMeasurements(obj, state)
           
            N = numel(state.poses);
            % TODO: Add ID search instead of indexing directly
            p = state.poses(obj.owenerID);
            measurements = {};
            
            for i = 1:N
                
                target_id = state.ids(i);                
                if target_id == obj.ownerID
                    continue
                end
                target_pose = state.poses(i);
                rel = target_pose - p;
                if norm(rel.position) > obj.maxRange
                    continue
                end                              
                
                m = MeasurementRangeBearing(p, target_pose, obj.covariance);
                m.observer_id = obj.ownerID;
                m.target_id = target_id;
                m.observer_time = state.time;
                m.target_time = state.time;
                measurements = [measurements; {m}];
                
            end
            
        end
        
    end
    
end