% A collection of indexable buffers
classdef BinBuffer < handle
    
    properties
        
        buff;       % Array of cells storing items
        inds;       % Array indices
        
    end
    
    methods
        
        function [obj] = BinBuffer(num_bins, capacity)
            
            if nargin == 0
                return
            elseif nargin == 1
                capacity = 100;
            end
            
            obj.buff = cell(num_bins, capacity);
            obj.inds = ones(1, num_bins);
            
        end
        
        function [new] = Copy(obj)
           
            new = BinBuffer(size(obj.buff,1), size(obj.buff,2));
            
        end
        
        function Push(obj, bin, item)
            
            item = FlattenCell(item);
            l = numel(item);
            obj.buff(bin, obj.inds(bin):obj.inds(bin) + l - 1) = item(:);
            obj.inds(bin) = obj.inds(bin) + l;
            
        end
        
        function [item] = Pop(obj, bin)
            
            if obj.inds(bin) == 1
                item = [];
                return
            end
            
            item = obj.buff{bin, obj.inds(bin) - 1};
            obj.inds(bin) = obj.inds(bin) - 1;
            
        end
        
        function [items] = PopAll(obj)
            
            items = cell(1, sum(obj.inds - 1));
            ind = 1;            
            for i = 1:numel(obj.inds)
                cnt = obj.inds(i) - 1;
                obj.inds(i) = 1;
                items(ind:ind + cnt - 1) = obj.buff(i, 1:cnt);
                ind = ind + cnt;
            end            
            
        end
        
        function [flags] = IsEmpty(obj)
           
            flags = (obj.inds == 1);
            
        end
        
    end
    
end