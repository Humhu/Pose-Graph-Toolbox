% Represents physical position in a 2D (flat) world
classdef Pose2D
    
    properties
        position;       % [x;y] position
        orientation;    % [theta] orientation
    end
    
    methods
        
        % Create a new Pose2D from a position vector and orientation
        % The orientation can be either a double or an Orientation1D
        %
        % An i x j x 2 position matrix returns an i x j matrix of poses
        % The orientation matrix should also then be i x j
        function obj = Pose2D(position, orientation)
            if nargin == 0
                return
            end
            
            if(isa(orientation, 'double'))
                orientation = Orientation1D(orientation);
            end
            
            m = size(position,1);
            n = size(position,2);
            obj(m,n) = Pose2D;
            for i = 1:m
                for j = 1:n
                    obj(i,j).position = reshape(position(i,j,1:2),2,1);
                    obj(i,j).orientation = orientation(i,j);
                end
            end
        end
        
        function D = double(obj)
            pos = [obj.position];
            D = reshape(pos(1,:), size(obj,1), size(obj,2));
            D(:,:,2) = reshape(pos(2,:), size(obj,1), size(obj,2));
            D(:,:,3) = reshape(double([obj.orientation]), size(obj,1), size(obj,2));
        end
        
        function C = plus(A, B)
            Ad = double(A);
            Bd = double(B);
            Cd = Ad + Bd;
            C = Pose2D(Cd(:,:,1:2), Cd(:,:,3));
        end
        
        function C = minus(A, B)
            Ad = double(A);
            Bd = double(B);
            Cd = Ad - Bd;
            C = Pose2D(Cd(:,:,1:2), Cd(:,:,3));
        end
        
        % TODO: Fix!
        function C = mtimes(A, B)
            if(isa(A,'Pose2D'))
                C = Pose2D(A.position*B, A.orientation*B);
            else
                C = Pose2D(A*B.position, A*B.orientation);
            end
        end
        
    end
    
end