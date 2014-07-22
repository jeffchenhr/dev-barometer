function [R] = RotationMatrixZ(c)
% takes the euler angles a b c that
% ratate around the axis, x, y, and z
% and computes the corresponding
% 3x3 rotation matrix R

R = [   cos(c)    sin(c)  0 ];
R = [R; (-sin(c)) cos(c)  0 ];
R = [R; 0         0       1 ];
