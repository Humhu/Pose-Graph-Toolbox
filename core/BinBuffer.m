% A collection of indexable buffers
classdef BinBuffer < handle
    
    properties
        
        buff;       % Array of cells storing items
        inds;       % Array indices
        
    end
    
    methods
        
        function [obj] = BinBuffer(capacity)
            
            if nargin == 0
                return
            elseif nargin == 1
                capacity = 100;
            end
            
            obj.buff = cell(1, capacity);
            obj.inds = 1;
            
        end
        
        function [new] = Copy(obj)
           
            new = BinBuffer(size(obj.buff,1), size(obj.buff,2));
            
        end
        
        function Push(obj, item)
            
            item = FlattenCell(item);
            l = numel(item);
            obj.buff(obj.inds:obj.inds + l - 1) = item(:);
            obj.inds = obj.inds + l;
            
        end
        
        function [item] = Pop(obj)
            
            if obj.inds == 1
                item = [];
                return
            end
            
            item = obj.buff{obj.inds - 1};
            obj.inds = obj.inds - 1;
            
        end
        
        function [items] = PopAll(obj)
            
            items = obj.buff(1:obj.inds - 1);                                  
            obj.inds = 1;                        
            
        end
        
        function [flags] = IsEmpty(obj)
           
            flags = (obj.inds == 1);
            
        end
        
    end
    
end