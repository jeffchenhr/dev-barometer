function [ output_args ] = testtrig( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

phi = 0
theta = pi/2
psi = 0

q = INT32_QUAT_OF_EULERS(phi, theta, psi)
rm = RMAT_OF_QUAT(q)


q1 = INT32_QUAT_OF_EULERS(pi/2, 0, 0)
rm1 = RMAT_OF_QUAT(q1)
q2 = INT32_QUAT_OF_EULERS(0, -pi/2, 0)
rm2 = RMAT_OF_QUAT(q2)

q12 = INT32_QUAT_COMP(q1, q2)
rm12 = RMAT_OF_QUAT(q12)
rm1*rm2

q21 = INT32_QUAT_COMP(q2, q1)
rm21 = RMAT_OF_QUAT(q21)
rm2*rm1

syms a b c;
v_out = INT32_QUAT_VMULT(q12, [a b c])

vin = [0 a b c];
ql = q12;
qr = [ql(1) -ql(2) -ql(3) -ql(4)];
wiki = INT32_QUAT_COMP(ql, INT32_QUAT_COMP(vin, qr))
papa = INT32_QUAT_COMP(qr, INT32_QUAT_COMP(vin, ql))

RMAT_OF_QUAT([-0.5 0.5 0.5 0.5])


q = INT32_QUAT_OF_EULERS(pi/2, 0, pi/2)
q = INT32_QUAT_OF_EULERS(-pi/2, -pi/2, 0)
RMAT_OF_QUAT(q)

end


% the paparazzi implementation is the other way round than the wikipedia
% implementation:
% wiki: p' = q*p*conj(q)
% papa: p' = conj(q)*p*q
function v_out = INT32_QUAT_VMULT(q, v_in)

v2qi2_m1 = 2*q(1)^2 - 1;
v2qx2    = 2*q(2)^2;
v2qy2    = 2*q(3)^2;
v2qz2    = 2*q(4)^2;
v2qiqx   = 2*q(1)*q(2);
v2qiqy   = 2*q(1)*q(3);
v2qiqz   = 2*q(1)*q(4);
m01 = 2*q(2)*q(3) + v2qiqz;
m02 = 2*q(2)*q(4) - v2qiqy;
m12 = 2*q(3)*q(4) + v2qiqx;
v_out(1) = (v2qi2_m1*v_in(1) + v2qx2 * v_in(1) + m01 * v_in(2) +  m02 * v_in(3));
v_out(2) = (v2qi2_m1*v_in(2) + m01 * v_in(1) -2*v2qiqz*v_in(1) + v2qy2 * v_in(2) + m12 * v_in(3));
v_out(3) = (v2qi2_m1*v_in(3) + m02 * v_in(1) +2*v2qiqy*v_in(1)+ m12 * v_in(2) -2*v2qiqx*v_in(2)+ v2qz2 * v_in(3));
end

function r = INT32_QUAT_COMP(a, b)
r(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
r(2) = a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
r(3) = a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
r(4) = a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1);
end
    
function rm = RMAT_OF_QUAT(q)    

v2qi2_m1 = 2*q(1)^2 - 1;
v2qiqx   = 2*q(1)*q(2);
v2qiqy   = 2*q(1)*q(3);
v2qiqz   = 2*q(1)*q(4);

v2qxqy   = 2*q(2)*q(3);
v2qxqz   = 2*q(2)*q(4);
v2qyqz   = 2*q(3)*q(4);

rm = [2*q(2)^2+v2qi2_m1  v2qxqy-v2qiqz  v2qxqz+v2qiqy; ...
      v2qxqy+v2qiqz  2*q(3)^2+v2qi2_m1  v2qyqz-v2qiqx; ...
      v2qxqz-v2qiqy  v2qyqz+v2qiqx  2*q(4)^2+v2qi2_m1];
% elements arrangement inside paparazzi:
% rm[0] rm[3] rm[6]
% rm[1] rm[4] rm[7]
% rm[2] rm[5] rm[8]
end

function vb = INT32_RMAT_VMULT(m_a2b, va)
% note that paparazzi flips rotation operations, thus R'
vb = R' * a;
end

function vo = INT32_VECT3_CROSS_PRODUCT(v1, v2)
vo(1) = v1(2)*v2(3) - v1(3)*v2(2);
vo(2) = v1(3)*v2(1) - v1(1)*v2(3);
vo(3) = v1(1)*v2(2) - v1(2)*v2(1);
end

function q = INT32_QUAT_OF_EULERS(phi, theta, psi)

phi2   = phi   / 2;
theta2 = theta / 2;
psi2   = psi   / 2;

s_phi2 = sin(phi2);
c_phi2 = cos(phi2);
s_theta2 = sin(theta2);
c_theta2 = cos(theta2);
s_psi2 = sin(psi2);
c_psi2 = cos(psi2);

c_th_c_ps = c_theta2 * c_psi2;
c_th_s_ps = c_theta2 * s_psi2;
s_th_s_ps = s_theta2 * s_psi2;
s_th_c_ps = s_theta2 * c_psi2;

q(1) =  c_phi2 * c_th_c_ps + s_phi2 * s_th_s_ps;

q(2) = -c_phi2 * s_th_s_ps + s_phi2 * c_th_c_ps;
q(3) =  c_phi2 * s_th_c_ps + s_phi2 * c_th_s_ps;
q(4) =  c_phi2 * c_th_s_ps - s_phi2 * s_th_c_ps;
end