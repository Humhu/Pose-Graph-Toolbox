% Chain update message
classdef ChainUpdateMessage < Message
    
    properties
   
        contents;
        
    end
    
    methods
        
        function [obj] = ChainUpdateMessage(to, from, chain)
           
            if nargin == 0
                return;
            end
            
            obj.senderID = from;
            obj.recipientID = to;
            obj.contents = chain;
            obj.content_type = 'ChainUpdate';
            
        end
        
    end
    
end