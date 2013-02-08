% Takes in fractal generation parameters to create a corresponding grouping
function [grouping] = GenerateGrouping(d, b)

    grouping = cell(d, 1);
    
    base = 1:b^(d-1);    
    
    for i = d:-1:2
       
        grouping{i} = mat2cell(base, 1, b*ones(1, ceil(numel(base)/b)));
        
        base = base(1:b:end);
        
    end
    
    grouping{1} = {1};

end