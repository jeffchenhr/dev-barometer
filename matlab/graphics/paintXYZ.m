function objh = paintXYZ(P, R, objh)

w = 0.125;
b = 1;
a = 1;
c = 1;

h = 0.125;

d = b - w/2;
e = c - h/2;

X =  [ w  w w w;  d  a a d;  d  a  a d;  w  w  w w;  w  w -w -w;  w  w -w -w;  w  w -w -w;  w  w -w -w;  w  w -w -w;     w  w -w -w; w  w -w -w;   w  w -w -w; w  w -w -w;   w  w -w -w; w  w -w -w ]';
Y = -[-w -w w w; -w -w w w; -w -w  w w; -w -w  w w; -w -w  w  w; -w -w -w -w; -d -b -b -d; -d -b -b -d; -w -w -w -w;     0 -w -w  0; 0 -w -w  0;   0 -w -w  0; 0  w  w  0;   0  w  w  0; 0  w  w  0 ]';
Z =  [ 0  h h 0;  0  h h 0;  0 -h -h 0;  0 -h -h 0;  0 -h -h  0;  0 -h -h  0;  0 -h -h  0;  0  h  h  0;  0  h  h  0;     0  h  h  0; h  h  h  h;   e  c  c  e; e  c  c  e;   h  h  h  h; 0  0  0  0 ]';

zf = 1.5;
X = zf*X;
Y = zf*Y;
Z = zf*Z;

C = zeros(15,4,3);
C(11:13,:,3) = ones(3,4);
C(6:8,:,2) = ones(3,4);
C(1:3,:,1) = ones(3,4);

Q(1,:,:) = X';
Q(2,:,:) = Y';
Q(3,:,:) = Z';
Q(:,:,1) = R * Q(:,:,1);
Q(:,:,2) = R * Q(:,:,2);
Q(:,:,3) = R * Q(:,:,3);
Q(:,:,4) = R * Q(:,:,4);
X = squeeze(Q(1,:,:)) + P(1);
Y = squeeze(Q(2,:,:)) + P(2);
Z = squeeze(Q(3,:,:)) + P(3);

if (nargin > 2)
    set(objh, 'XData', X, 'YData', Y, 'ZData', Z);
else
    objh = surface(X, Y, Z, C);
    %objh = surface(X, Y, Z);
end

end

function [R] = rotMat(v)
R = exp(1)^[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end