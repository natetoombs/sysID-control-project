%% Feedback linearization WITHOUT perfect tracking for constant yd(t)
clear all; close all; clc

g_coeff = 6.158;
g = 9.81;
Cd_vg = 0.06057;
Cd_v = Cd_vg/g_coeff;
Cd_ug = Cd_vg;

% 4x4 State Space

A = [0 0 1 0;
    0 0 0 1;
    0 0 -Cd_ug 0;
    0 0 0 -Cd_vg];

B = [0 0;
    0 0;
    0 -g;
    g 0];


d_prim_poles = [-0.5 + 1.5*j;-0.5 - 1.5*j; -0.2; -1]

K = place(A,B,d_prim_poles)
A_CL = A - B*K

G_s = ss(A,B,eye(4),zeros(4,2))
H_s = ss(K)
L_s = series(H_s, G_s)
T_s = feedback(L_s,eye(4))
step(T_s)
S_s = feedback(eye(4),L_s)
R_s = feedback(H_s,G_s);


%% Feedback linearization WITH perfect tracking for constant yd(t)
clear all; close all; clc

s = tf('s')

g_coeff = 6.158;
g = 9.81;
Cd_vg = 0.06057;
Cd_v = Cd_vg/g_coeff;
Cd_ug = Cd_vg;


A = [0 0 1 0;
    0 0 0 1;
    0 0 -Cd_ug 0;
    0 0 0 -Cd_vg];

B = [0 0;
    0 0;
    0 -g;
    g 0];

C = eye(4);

D = zeros(4,2);

b_s = 1/s;
B_s = ss(eye(2)*b_s);
A_b = B_s.A
B_b = B_s.B
C_b = B_s.C
D_b = B_s.D

G_s = ss(A,B,C,D);
G_s_a = series(b_s,G_s);
A_a_2 = G_s_a.A
B_a_2 = G_s_a.B
C_a_2 = G_s_a.C
D_a_2 = G_s_a.D

A_a = [A_b,zeros(2,4);B*C_b, A]
B_a = [B_b; zeros(4,2)]
C_a = [zeros(4,2),C]
D_a = zeros(2,2)

delta = 1e-6;
p_1 = -0.5 + 1.5*j;
p_2 = -0.5 - 1.5*j;
p_3 = -0.2;
p_4 = -1;
p_5 = 3*p_4;

delta = 1e-6;
d_prim_poles = [p_1; p_2; p_3;p_4; p_4 - delta; p_4 - 2*delta]
d_scnd_poles = [p_5; p_5-delta; p_5-2*delta;p_5-3*delta;p_5-4*delta;p_5-5*delta]

K_a = place(A_a,B_a,d_prim_poles)
L_a = place(A_a.',C_a.',d_scnd_poles).'

A_H_a = A_a - L_a*C_a - B_a*K_a
B_H_a = L_a
C_H_a = K_a
H_s_a = ss(A_H_a,B_H_a,C_H_a,0)
H_s = series(H_s_a,b_s);
L_s = series(H_s,G_s);
T_s = feedback(L_s,eye(4));
S_s = feedback(eye(4), L_s);
R_s = feedback(H_s,G_s);

figure
step(T_s)

%% LQR WITHOUT perfect tracking for constant yd(t)
clear all; close all; clc

g_coeff = 6.158;
g = 9.81;
Cd_vg = 0.06057;
Cd_v = Cd_vg/g_coeff;
Cd_ug = Cd_vg;


A = [0 0 1 0;
    0 0 0 1;
    0 0 -Cd_ug 0;
    0 0 0 -Cd_vg];

B = [0 0;
    0 0;
    0 -g;
    g 0];

C = eye(4);

D = zeros(4,2);

u_max = 15*pi/180;
r = (1/u_max)^2;

x_max = 0.25;
x_dot_max = 1;

q1 = (1/x_max)^2;
q2 = (1/x_dot_max)^2;

Q = diag([q1,q1,q2,q2]);
R = r*eye(2);


[K_lqr,S_lqr,clp_lqr] = lqr(A,B,Q,R)

G_s = ss(A,B,C,D);
H_s = ss(K_lqr);
L_s = series(H_s,G_s);
T_s = feedback(L_s,eye(4));
S_s = feedback(eye(4), L_s);
R_s = feedback(H_s,G_s);

step(T_s)

figure
sigma(T_s)

figure
sigma(S_s)

%% LQR WITH perfect tracking for constant yd(t)

clear all; close all; clc

s = tf('s')

g_coeff = 6.158;
g = 9.81;
Cd_vg = 0.06057;
Cd_v = Cd_vg/g_coeff;
Cd_ug = Cd_vg;


A = [0 0 1 0;
    0 0 0 1;
    0 0 -Cd_ug 0;
    0 0 0 -Cd_vg];

B = [0 0;
    0 0;
    0 -g;
    g 0];

C = eye(4);

D = zeros(4,2);


b_s = 1/s;
B_s = ss(eye(2)*b_s);
A_b = B_s.A
B_b = B_s.B
C_b = B_s.C
D_b = B_s.D

G_s = ss(A,B,C,D);
G_s_a = series(b_s,G_s);
A_a_2 = G_s_a.A
B_a_2 = G_s_a.B
C_a_2 = G_s_a.C
D_a_2 = G_s_a.D

A_a = [A_b,zeros(2,4);B*C_b, A]
B_a = [B_b; zeros(4,2)]
C_a = [zeros(4,2),C]
D_a = zeros(2,2)

u_max = 15*pi/180;
r = (1/u_max)^2;

x_max = 0.25;
x_dot_max = 1;

q1 = 1*(1/x_max)^2;
r1 = 1*(1/u_max)^2;

q2 = 1;
r2 = 1;

Q1 = q1*eye(6);
R1 = r1*eye(2);
Q2 = q2*eye(6);
R2 = r2*eye(4);

[K_lqr,~,clp_plant] = lqr(A_a,B_a,Q1,R1)
[L_lqr,~,clp_obsrv] = lqr(A_a.',C_a.',Q2,R2)
L_lqr = L_lqr.';

A_H_a = A_a - L_lqr*C_a - B_a*K_lqr
B_H_a = L_lqr
C_H_a = K_lqr
H_s_a = ss(A_H_a,B_H_a,C_H_a,0)
H_s = series(H_s_a,b_s)

L_s = series(H_s,G_s);
T_s = feedback(L_s,eye(4));
S_s = feedback(eye(4), L_s);
R_s = feedback(H_s,G_s);

figure
step(T_s)

figure
sigma(T_s)

figure
sigma(S_s)

zpk(H_s)

e_ss = evalfr(S_s,0)