% Represents physical position in a 2D (flat) world
classdef Pose2D
    
    properties
        position;       % [x;y] position
        orientation;    % [theta] orientation
    end
    
    methods
        
        % Create a new Pose2D from a position vector and orientation
        % The orientation can be either a double or an Orientation1D
        function obj = Pose2D(position, orientation)
            if nargin == 0
                return
            end
            
            if(isa(orientation, 'Orientation1D'))
                orientation = double(orientation);
            end
            
            m = size(position,2);
            n = size(position,3);
            obj(m,n) = Pose2D;
            for i = 1:m
                for j = 1:n
                    obj(i,j).position = position(1:2,i,j);
                    obj(i,j).orientation = Orientation1D(orientation(i,j));
                end
            end
        end
        
        function D = double(obj)            
            D = [reshape([obj.position], 2, size(obj,1), size(obj,2));
                reshape(double([obj.orientation]), 1, size(obj,1), size(obj,2))];
        end
        
        function C = plus(A, B)
            [Ap, Ao] = A.getElements;
            [Bp, Bo] = B.getElements;
            C = Pose2D(Ap + Bp, Ao + Bo);
        end
        
        function C = minus(A, B)
            C = Pose2D(A.position - B.position, A.orientation - B.orientation);
        end
        
        function C = mtimes(A, B)
            if(isa(A,'Pose2D'))
                C = Pose2D(A.position*B, A.orientation*B);
            else
                C = Pose2D(A*B.position, A*B.orientation);
            end
        end
        
        % Convert into a double representation
        % Returned arrays are same size as original object initialization
        function [P,O] = getElements(obj)
            P = reshape([obj.position], 2, size(obj,1), size(obj,2));
            O = reshape([obj.orientation], size(obj,1), size(obj,2));
        end
        
    end
    
end