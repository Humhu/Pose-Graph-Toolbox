% Test for 2D quaternions
function [c] = Quat2Add(q1, q2)

q1_f = [q1(2,:);
        q1(1,:)];
q1_n = [q1(1,:);
        -q1(2,:)];
c = [q1_n'*q2;
    q1_f'*q2];
   

end