function [R] = RotationMatrixY(b)
% takes the euler angles a b c that
% ratate around the axis, x, y, and z
% and computes the corresponding
% 3x3 rotation matrix R

R = [   cos(b)  0   (-sin(b)) ];
R = [R; 0       1   0         ];
R = [R; sin(b)  0   cos(b)    ];
