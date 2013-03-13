% Abstract message class
classdef (Abstract) Message
    
    properties
        
        senderID;       % From
        recipientID;    % To
        content_type;   % What's in it
        sendTime;
        receiveTime;
        
    end    
    
    methods(Static)
       
        % Sorts by send time, oldest first
        function [sorted] = SortSendTime(messages)
           
            N = numel(messages);
            times = zeros(1, N);
            for i = 1:N
                times(i) = messages{i}.sendTime;
            end
            [~, inds] = sort(times, 'ascend');
            sorted = messages(inds);
            
        end
        
        function [matches] = GetType(messages, type)
           
            matches = {};
            for i = 1:numel(messages)
               
                m = messages{i};
                if strcmp(m.content_type, type)
                   matches = [matches, {m}];
                end
                
            end
            
        end
        
    end
    
end