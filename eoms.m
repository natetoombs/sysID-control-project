clear; clc;

syms sphi stheta spsi cphi ctheta cpsi x y z u v w phi theta psi p q r m g...
    Cd_u Cd_v Cd_w Cd_p Cd_q Cd_r f_thrust tau_roll tau_pitch tau_yaw Ixx...
    Iyy Izz 

R_BI = [ctheta*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi;
        ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi;
        -stheta, sphi*ctheta, cphi*ctheta];

A = [1 0 -stheta;
     0 cphi ctheta*sphi;
     0 -sphi ctheta*cphi];
 
pos_I = [x; y; z];
vel_B = [u; v; w];
angles = [phi; theta; psi];
om_BI = [p; q; r];

Cd_F = [Cd_u Cd_v Cd_w];
Cd_M = [Cd_p Cd_q Cd_r];

F_aero = -f_thrust*Cd_F*vel_B + f_thrust*[0;0;-1];
F_g = m*g*[0;0;1];

I = [Ixx 0 0;
     0 Iyy 0;
     0 0 Izz];

M_aero = [tau_roll; tau_pitch; tau_yaw] + -[Cd_p Cd_q Cd_r]*om_BI;

x_dot = R_BI*vel_B
v_dot = 1/m*(R_BI.'*F_g + F_aero) - skew(om_BI)*vel_B
angles_dot = inv(A)*om_BI
pqr_dot = inv(I)*(-skew(om_BI)*I*om_BI + M_aero)

x_dot_lin = subs(x_dot,[ctheta stheta cphi sphi cpsi spsi],[1 theta 1 phi 1 psi])
v_dot_lin = subs(v_dot,[ctheta stheta cphi sphi cpsi spsi],[1 theta 1 phi 1 psi])
angles_dot_lin = subs(angles_dot,[ctheta stheta cphi sphi cpsi spsi],[1 theta 1 phi 1 psi])
pqr_dot_lin = subs(pqr_dot,[ctheta stheta cphi sphi cpsi spsi],[1 theta 1 phi 1 psi])
 
function y = skew(x)
y = [0 -x(3) x(2);
     x(3) 0 -x(1);
     -x(2) x(1) 0];
end


