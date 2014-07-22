function objh = paintQuadshot(P, R, objh);

l = 0.15; % wing half-length (x-dim)
lf = 0.08; % wing thick front-length (x-dim)
lb = 0.02; % wing thick back-length (x-dim)
w = 0.50; % wing half-width
h = 0.05; % wing half-height

a = 0.20; % position top motors
b = 0.30; % position bottom motors
c = 0.03; % motor width
d = 0.25; % motor height

ac = a+c;
am = a+c/2;
bc = b+c;
bm = b+c/2;
dd = d - c/2;

wh = w; %w - h;


%%% Quadshot in ENU frame (x is right/east, y is forward/north, z is up)
ZL = [l; lf; -lb; -l];
Z = repmat(ZL, 1, 23);
X = [0 0 0 0; am a a am; am a a am; am ac ac am; am ac ac am;  wh w w wh; wh w w wh;  bm bc bc bm; bm bc bc bm; bm b b bm; bm b b bm;   0 0 0 0; -bm -b -b -bm; -bm -b -b -bm; -bm -bc -bc -bm; -bm -bc -bc -bm;  -wh -w -w -wh; -wh -w -w -wh; -am -ac -ac -am; -am -ac -ac -am; -am -a -a -am; -am -a -a -am; 0 0 0 0]';
%Y = [0 0 0 0; am a a am; am a a am; am ac ac am; am ac ac am;  wh w w wh; wh w w wh;  bm bc bc bm; bm bc bc bm; bm b b bm; bm b b bm;   0 0 0 0; -bm -b -b -bm; -bm -b -b -bm; -bm -bc -bc -bm; -bm -bc -bc -bm;  -wh -w -w -wh; -wh -w -w -wh; -am -ac -ac -am; -am -ac -ac -am; -am -a -a -am; -am -a -a -am; 0 0 0 0]';
Y = -[0 h h 0; 0 h h 0; dd d d dd; dd d d dd; 0 h h 0; 0 h h 0; 0 -h -h 0;  0 -h -h 0; -dd -d -d -dd; -dd -d -d -dd; 0 -h -h 0;   0 -h -h 0; 0 -h -h 0; -dd -d -d -dd; -dd -d -d -dd; 0 -h -h 0; 0 -h -h 0; 0 h h 0;   0 h h 0; dd d d dd; dd d d dd; 0 h h 0;    0 h h 0]';

%%%ZL = -[l; lf; -lb; -l];
%%%Z = repmat(ZL, 1, 23);
%%%Y = [0 0 0 0; am a a am; am a a am; am ac ac am; am ac ac am;  wh w w wh; wh w w wh;  bm bc bc bm; bm bc bc bm; bm b b bm; bm b b bm;   0 0 0 0; -bm -b -b -bm; -bm -b -b -bm; -bm -bc -bc -bm; -bm -bc -bc -bm;  -wh -w -w -wh; -wh -w -w -wh; -am -ac -ac -am; -am -ac -ac -am; -am -a -a -am; -am -a -a -am; 0 0 0 0]';
%%%%Y = [0 0 0 0; am a a am; am a a am; am ac ac am; am ac ac am;  wh w w wh; wh w w wh;  bm bc bc bm; bm bc bc bm; bm b b bm; bm b b bm;   0 0 0 0; -bm -b -b -bm; -bm -b -b -bm; -bm -bc -bc -bm; -bm -bc -bc -bm;  -wh -w -w -wh; -wh -w -w -wh; -am -ac -ac -am; -am -ac -ac -am; -am -a -a -am; -am -a -a -am; 0 0 0 0]';
%%%X = -[0 h h 0; 0 h h 0; dd d d dd; dd d d dd; 0 h h 0; 0 h h 0; 0 -h -h 0;  0 -h -h 0; -dd -d -d -dd; -dd -d -d -dd; 0 -h -h 0;   0 -h -h 0; 0 -h -h 0; -dd -d -d -dd; -dd -d -d -dd; 0 -h -h 0; 0 -h -h 0; 0 h h 0;   0 h h 0; dd d d dd; dd d d dd; 0 h h 0;    0 h h 0]';

zf = 4;
X = zf*X;
Y = zf*Y;
Z = zf*Z;

%X = [w w -w -w; w w -w -w; w w -w -w; w w -w -w; w w -w -w; w w -w -w; w w -w -w]';
%Y = [0 0 0 0; ah a a ah; ah a a ah; 0 0 0 0; -ah -a -a -ah; -ah -a -a -ah; 0 0 0 0]';
%Z = [0 h h 0; 0 h h 0; 0 -h -h 0; 0 -h -h 0; 0 -h -h 0; 0 h h 0; 0 h h 0]';


C = zeros(23,4,3);
C([1 5 6 7 11 12 16 17 18 22],:,3) = 0.8*ones(10,4);
C([1 5 6 7 11 12 16 17 18 22],:,2) = 0.8*ones(10,4);
C([1 5 6 7 11 12 16 17 18 22],:,1) = 0.8*ones(10,4);

C([2 3 4  8 9 10],:,1) = 0.8*ones(6,4);
C([13 14 15  19 20 21],:,2) = 0.8*ones(6,4);

%C(1:3,:,1) = ones(3,4);

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