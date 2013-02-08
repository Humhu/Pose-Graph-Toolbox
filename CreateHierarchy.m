% Organizes robots into a specified hierarchy, or if no grouping is
% specified, groups automatically based on distance
%
% Inputs:
%   robots - Array of Robot objects
%   grouping - K x 1 cell array, where grouping{k}{j} is a matrix of robot
%   indices in a group at depth k-1. The first index in the array is the
%   leader.

function [robots] = CreateHierarchy(robots, grouping)

if nargin == 1
   return;
   % Some sorting algorithm here
end

for k = 0:numel(grouping) - 1
   
    level = grouping{k + 1};
    
    for i = 1:numel(indices)
       
        group = level{i};
        followers = HierarchyRole();
        
        for j = 1:numel(group)
            
            ind = group(j);
            
            role = HierarchyRole(k);            
            followers(j) = role;
            
            robot = robots(ind);
            robot.RegisterRole(role);
        
        end
        
        if k == 0
            continue
        end
        leader = robots(group(1)).roles(k);
        leader.AssignFollowers(followers);
        
    end
    
end

