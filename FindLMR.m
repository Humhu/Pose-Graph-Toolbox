% Finds the a matching role in the tree at the same level with a given ID
% TODO: Make more efficient
function [match] = FindLMR(start, ID)

level = start.level;
prev = start;

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
for i = 1:start.level - level - 1
   
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
   if f.ownerID == ID;
       match = f;
       break
   end
end