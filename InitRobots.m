% Initializes robots based off template at specified positions.
% 
% Inputs:
%   templates - Array of example Robots.
%   positions - Cell array of 3 X ? matrices. Robot templates{i} will be
%   copied and initialized to each pose in positions{i}.
%
% Outputs:
%   robots - Array of Robots initialized and ID'd consecutively.
function [robots] = InitRobots(templates, positions)

robots = Robot.empty(1,0);
id = templates(1).ID;
acc = 1;

for i = 1:numel(templates)
   
    example = templates(i);
    poses = positions{i};
    
    for j = 1:size(poses, 2)
       
        r = Robot(example);
        r.pose = poses(:,j);
        r.ID = id;
        r = Robot(r); % HACK!!!
        
        id = id + 1;
        robots(acc) = r;
        acc = acc + 1;
        
    end
    
end

