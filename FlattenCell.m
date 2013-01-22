% TODO: Make work for the general case
function [flat] = FlattenCell(data)
    
    if ~iscell(data)
        flat = { data(:)' };
        return;
    end        
    
    flat = cell(1,0);
    for i = 1:numel(data)
        flat = [flat, FlattenCell(data{i})];
    end
    
end