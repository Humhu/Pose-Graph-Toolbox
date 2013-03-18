function [match] = FindReciprocal(start, ID)

level = start.chained_graph.depth;
prev = start;

match = [];

% Search by going up the tree and checking descendants
while(level > 0)
   
    up = prev.leader;
    level = level - 1;
    ids = GetDescendants(up);
    if ismember(ID, ids)
       break 
    end
    prev = up;
    
end

% Once a common ancestor is found, go down the tree 
prev = up;
for i = 1:start.chained_graph.depth - level - 1
   
    downs = prev.followers();
    for j = 1:numel(downs)        
        f = downs(j);
        ids = f.GetDescendants();
        if ismember(ID, ids)
            break
        end
    end
    prev = f;
    
end

for i = 1:numel(prev.followers)
   f = prev.followers(i);
   if ismember(ID, f.chained_graph.robot_scope)
       match = f;
       break
   end
end