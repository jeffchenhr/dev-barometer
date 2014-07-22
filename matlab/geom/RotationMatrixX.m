function [R] = RotationMatrixX(a)
% takes the euler angles a b c that
% ratate around the axis, x, y, and z
% and computes the corresponding
% 3x3 rotation matrix R

R = [   1       0     0     ];
R = [R; 0    cos(a)   sin(a)];
R = [R; 0  (-sin(a))  cos(a)];
