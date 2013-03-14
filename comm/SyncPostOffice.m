% Handles synchronous communications
classdef SyncPostOffice < handle
    
    properties
        
        inboxes;
        outboxes;
        box_size;
        
    end
    
    properties(Access=private)
        
        pendingTransactions;  % Messages that have been pushed to send
        idCounter;  % Next available ID
        commlog; %Format is columns [time, from, to] in commIDs
        
    end
    
    methods
        
        function [obj] = SyncPostOffice(buff_size)
            
            obj.box_size = buff_size;
            obj.inboxes = BinBuffer.empty(1,0);
            obj.outboxes = BinBuffer.empty(1,0);
            obj.idCounter = 1;
            obj.commlog;
            
        end
        
        function [clog] = GetCommlog(obj)
            
            clog = obj.commlog;
            
        end
        
        function [id] = Register(obj)
            
            id = obj.idCounter;
            obj.inboxes(id) = BinBuffer(obj.box_size);
            obj.outboxes(id) = BinBuffer(obj.box_size);
            obj.idCounter = obj.idCounter + 1;
            
        end
        
        function Deposit(obj, id, messages)
            
            if ~obj.CheckID(id)
                error('SyncPostOffice only has %d outboxes, invalid ID %d\n', ...
                    numel(obj.outboxes), id);
            end
            
            obj.outboxes(id).Push(messages);
            
        end
        
        function DirectDeposit(obj, messages)
           
            for i = 1:numel(messages)
                m = messages{i};
                to = m.recipientID;
                if ~obj.CheckID(to)
                    continue
                end
                obj.inboxes(to).Push(m);
            end
            
        end
        
        function [items] = Withdraw(obj, id)
            
            if ~obj.CheckID(id)
                error('SyncPostOffice only has %d inboxes, invalid ID %d\n', ...
                    numel(obj.inboxes), id);
            end
            items = obj.inboxes(id).PopAll();
            
        end
        
        function Transmit(obj, id)
            
            toSend = obj.outboxes(id).PopAll();
            obj.pendingTransactions = [obj.pendingTransactions, toSend];
            
        end
        
        % Need time from world to log correctly
        function ProcessTransactions(obj, time)
            
            N = numel(obj.pendingTransactions);
            log_entry = zeros(N, 3);
            for i = 1:N
                
                m = obj.pendingTransactions{i};
                to = m.recipientID;
                if ~obj.CheckID(to)
                    continue
                end
                log_entry(i,:) = [time, m.senderID, to];
                obj.inboxes(to).Push(m);
                
            end
            obj.commlog = [obj.commlog; log_entry];
            obj.pendingTransactions = {};
            
        end
        
    end
    
    methods(Access = private)
       
        function [val] = CheckID(obj, id)
           
            val = id > 0 && id <= numel(obj.inboxes);
            
        end
        
    end
    
    
end
