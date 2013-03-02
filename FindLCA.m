% Finds the lowest common ancestor of two HieararchyRoles. The inputs do
% not have to be at the same depth.
%
% Input:
%   r1 - Hierarchy role 1
%   r2 - Hierarchy role 2
%
% Output:
%   ancestor - Ancestor HierarchyRole object
%   k - Level of the common ancestor
%   
function [ancestor, k] = FindLCA(r1, r2)

k1 = r1.chained_graph.depth + 1;
k2 = r2.chained_graph.depth + 1;

pred1 = cell(k1, 1);
pred2 = cell(k2, 1);

pred1{k1} = r1;
for i = k1-1:-1:1
    pred1{i} = pred1{i+1}.leader;    
end

pred2{k2} = r2;
for i = k2-1:-1:1
    pred2{i} = pred2{i+1}.leader;    
end

km = min(k1, k2);
pred1 = pred1(1:km);
pred2 = pred2(1:km);

ancestor = [];
k = -1;

for i = km:-1:1
   l1 = pred1{i};
   l2 = pred2{i};
   if l1.ownerID == l2.ownerID
       ancestor = l1;
       k = i - 1;
       break
   end
end
