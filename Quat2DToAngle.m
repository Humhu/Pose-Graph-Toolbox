function [a] = Quat2DToAngle(o)

a = atan2(o(2,:), o(1,:));