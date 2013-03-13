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
        
    end
    
    methods
        
        function [obj] = SyncPostOffice(buff_size)
            
            obj.box_size = buff_size;
            obj.inboxes = BinBuffer.empty(1,0);
            obj.outboxes = BinBuffer.empty(1,0);
            obj.idCounter = 1;
            
        end
        
        function [id] = Register(obj)
            
            id = obj.idCounter;
            obj.inboxes(id) = BinBuffer(1, obj.box_size);
            obj.outboxes(id) = BinBuffer(1, obj.box_size);
            obj.idCounter = obj.idCounter + 1;
            
        end
        
        function Deposit(obj, id, messages)
            
            if ~obj.CheckID(id)
                error('SyncPostOffice only has %d outboxes, invalid ID %d\n', ...
                    numel(obj.outboxes), id);
            end
            
            obj.outboxes(id).Push(1, messages);
            
        end
        
        function DirectDeposit(obj, messages)
           
            for i = 1:numel(messages)
                m = messages{i};
                to = m.recipientID;
                if ~obj.CheckID(to)
                    continue
                end
                obj.inboxes(to).Push(1, m);
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
        
        function ProcessTransactions(obj)
            
            for i = 1:numel(obj.pendingTransactions)
                
                m = obj.pendingTransactions{i};
                to = m.recipientID;
                if ~obj.CheckID(to)
                    continue
                end
                obj.inboxes(to).Push(1, m);
                
            end
            
            obj.pendingTransactions = {};
            
        end
        
    end
    
    methods(Access = private)
       
        function [val] = CheckID(obj, id)
           
            val = id > 0 && id <= numel(obj.inboxes);
            
        end
        
    end
    
    
end
