% Represents a single angle in range [-pi, pi]
% TODO: Optimize! The double for loop in the constructor is slow as hell.
classdef Orientation1D
    
    properties
        angle;
    end
    
    methods
        
%         function obj = Orientation1D(angle)
%             if nargin == 0
%                 return
%             end
%             
%             m = size(angle,1);
%             n = size(angle,2);
%             obj(m,n) = Orientation1D;
%             for i = 1:m
%                 for j = 1:n
%                     obj(i,j).angle = angle(i,j);
%                 end
%             end
%         end        

        function obj = Orientation1D(angle)

            if nargin == 0
                return
            end
            
            N = numel(angle);
            obj(N,1) = Orientation1D;
            for i = 1:N
               obj(i).angle = angle(i); 
            end
            obj = reshape(obj, size(angle));
            
        end

        function obj = set.angle(obj, a)
            obj.angle = wrapToPi(a);
        end
        
        function D = double(obj)
            D = reshape([obj.angle], size(obj,1), size(obj,2));
        end
        
        function C = plus(A, B)
            if(isa(A, 'Orientation1D'))
                A = double(A);
            end
            if(isa(B, 'Orientation1D'))
                B = double(B);
            end
            C = Orientation1D(A + B);
        end
        
        function C = minus(A, B)
            if(isa(A, 'Orientation1D'))
                A = double(A);
            end
            if(isa(B, 'Orientation1D'))
                B = double(B);
            end
            C = Orientation1D(A - B);
        end
        
        function C = mtimes(A, B)
            if(isa(A, 'Orientation1D'))
                A = double(A);
            end
            if(isa(B, 'Orientation1D'))
                B = double(B);
            end
            C = Orientation1D(A*B);
        end
        
        function C = times(A, B)
            if(isa(A, 'Orientation1D'))
                A = double(A);
            end
            if(isa(B, 'Orientation1D'))
                B = double(B);
            end
            C = Orientation1D(A.*B);
        end
        
        function C = rdivide(A, B)
            if(isa(A, 'Orientation1D'))
                A = double(A);
            end
            if(isa(B, 'Orientation1D'))
                B = double(B);
            end
            C = Orientation1D(A./B);
        end        
        
    end
    
end