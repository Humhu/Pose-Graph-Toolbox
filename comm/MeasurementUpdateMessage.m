% Measurement communication message
classdef MeasurementUpdateMessage < Message
    
    properties
        
        contents;
        
    end
    
    methods
        
        function [obj] = MeasurementUpdateMessage(to, from, meas)
            
            if nargin == 0
                return;
            end
            
            obj.senderID = from;
            obj.recipientID = to;
            obj.contents = meas;
            obj.content_type = 'MeasurementUpdate';
            
        end
        
    end
    
end