% Represents all information at a particular time
classdef WorldState2D
    
    properties
        
        poses;          % Poses at this time step
        ids;            % IDs corresponding to poses
        measurements;   % Measurements at this time step
        time;           % Time step or time
        
    end
    
    methods
        
        function [obj] = WorldState2D()
            
            if nargin == 0
                return
            end
            
        end
        
        % Returns
        function [d] = GetDimension(obj)
            if isempty([obj(1).poses])
                d = 0;
            else
                d = size(obj(1).poses, 2);
            end
        end
        
        % Returns time span
        function [l] = GetLength(obj)
            
            if numel(obj) < 2
                l = 0;
                return
            end
            times = [obj.time];
            l = max(times) - min(times);
            
        end                
                     
        % Build ID and time to index SearchMap objects
        function [idMap, tMap] = BuildMaps(obj)
            
            % Map ID-indices
            pinds = 1:numel(obj(1).ids);
            idMap = SearchMap(obj(1).ids, pinds);
            
            % Map time-indices
            times = [obj.time];
            tinds = 1:numel(times);
            tMap = SearchMap(times, tinds);
            
        end
        
        function [sub] = GetSubset(obj, ids, times)
            
            if isempty(times)
                times = [obj.time];
            end
            if isempty(ids)
                ids = obj(1).ids;
            end
            
            [idMap, tMap] = obj.BuildMaps();
            id_inds = idMap.Forward(ids);
            t_inds = tMap.Forward(times);
            
            sub = obj(t_inds);
            for i = 1:numel(sub)
                sub(i).poses = sub(i).poses(:,id_inds);
                sub(i).ids = ids;
            end
            
        end
        
        % Rotates the entire sequence
        function [rotated] = Rotate(obj, theta)
            
            rotated = obj;                        
            
            R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
            p = [rotated.poses];            
            p(1:2,:) = R*p(1:2,:);
            p(3,:) = bsxfun(@plus, p(3,:), theta);
            
            N = rotated.GetDimension();
            T = numel(rotated);            
            for i = 1:T
                rotated(i).poses = p(:,(N*(i-1) + 1):(N*i));
            end
            
        end
        
        % Translates the entire sequence
        % Shift is a 2 by 1 [x; y] vector
        function [shifted] = Shift(obj, shift)
            
            shifted = obj;
            p = [shifted.poses];
            p(1:2,:) = bsxfun(@plus, p(1:2,:), shift);
            
            N = shifted.GetDimension();
            T = numel(shifted);
            for i = 1:T
                shifted(i).poses = p(:,(N*(i-1) + 1):(N*i));
            end
        end
        
        function [zeroed] = Zero(obj, base_id, base_time)
            
            if nargin == 1
                base_id = obj(1).ids(1);
                base_time = obj(1).time;
            end
            
            zeroed = obj;
            [idMap, tMap] = zeroed.BuildMaps();
            id_ind = idMap.Forward(base_id);
            t_ind = tMap.Forward(base_time);
            
            if isempty(id_ind) || isempty(t_ind)
                return
            end
            
            zero_pose = obj(t_ind).poses(:,id_ind);
            
            zeroed = zeroed.Shift(-zero_pose(1:2));
            zeroed = zeroed.Rotate(-zero_pose(3));
            
%             a = zero_pose(3);
%             R = [cos(a), sin(a);
%                 -sin(a), cos(a)];
%             
%             N = zeroed.GetDimension();
%             T = numel(zeroed);
%             p = [zeroed.poses];
%             p = bsxfun(@minus, p, zero_pose);
%             p(1:2,:) = R*p(1:2,:);
%             for i = 1:T
%                 zeroed(i).poses = p(:,(N*(i-1) + 1):(N*i));
%             end
            
        end
       
    end
    
    methods(Static)

        % Returns the difference between the overlapping poses with another
        % graph. We assume the IDs and times are sorted in ascending order.
        function [diff] = Compare(a, b)            
            
            % Get subset of other graph corresponding to local times
            t_matches = ismember([b.time], [a.time]);
            if sum(t_matches) ~= numel(a)
               error(['Comparison graph does not contain local subgraph.\n', ...
                   'Local ID: ', num2str(a(1).ids), '\n', ...
                   'Local t: ', num2str([a.time]), '\n', ...
                   'Other ID: ', num2str(b(1).ids), '\n', ...
                   'Other t: ', num2str([b.time])]);                    
            end
            b = b(t_matches);                      
            
            % Get subset of other graph that matches local
            id_matches = ismember(b(1).ids, a(1).ids);            
            T = numel(a);            
            
            %[idMap, tMap] = b.BuildMaps();
            %id_ind = idMap.Forward(obj.base_id);
            %t_ind = tMap.Forward(obj.base_time);            
            %b_pose = b(t_ind).poses(:,id_ind);
            %b = b.Shift(-b_pose(1:2));
            %b = b.Rotate(-b_pose(3));
            
            other_poses = [b.poses];
            other_poses = other_poses(:, repmat(id_matches, 1, T));
            our_poses = [a.poses];
            diff = other_poses - our_poses;
            diff(3,:) = wrapToPi(diff(3,:));                    
        
        end
        
    end
    
end