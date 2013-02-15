% Organizes robots into a specified hierarchy
%
% Inputs:
%   robots - Array of Robot objects
%   grouping - K x 1 cell array, where grouping{k}{j} is a matrix of robot
%   indices in a group at depth k-1. The first index in the array is the
%   leader.
%   time_scales - K x 1 array, where time_scales(k) is the time scale at
%   depth k -1

function [robots] = AssignGrouping(robots, grouping, time_scales)

for k = 0:numel(grouping) - 1
   
    level = grouping{k + 1};
    
    for i = 1:numel(level)
       
        group = level{i};
        followers = HierarchyRole.empty(numel(group),0);
        
        for j = 1:numel(group)
            
            ind = group(j);
            
            role = HierarchyRole(k);            
            followers(j) = role;
            role.time_scale = time_scales(k + 1);
            
            robot = robots(ind);
            robot.RegisterRole(role);
        
        end
        
        if k == 0
            continue
        end
        leader = robots(group(1)).roles(end - 1);
        leader.AssignFollowers(followers);
        
    end
    
end

