classdef Quaternion2D
    
    properties
        q;
    end
    
    methods
        
        function [obj] = Quaternion2D(angles)
            if nargin == 0
                return
            end
            obj.q = [cos(angles(:)');
                sin(angles(:)')];
        end
        
        function [c] = plus(a, b)            
            c = Quaternion2D();
            c.q = [a.q(1,:).*b.q(1,:) - a.q(2,:).*b.q(2,:);
                   a.q(2,:).*b.q(1,:) + a.q(1,:).*b.q(2,:)];
                
        end
        
        function [c] = minus(a, b)
            c = a + (-b);
        end
        
        function [a] = uminus(a)
            a.q(2,:) = -a.q(2,:);
        end
        
        function [d] = double(a)
            d = atan2(a.q(2,:), a.q(1,:));
        end
        
    end
    
end