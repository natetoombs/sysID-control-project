% Control Stuff

clear


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
  
  
  d_poles = [-0.5 + 1.5*j;-0.5 - 1.5*j; -0.2; -1]
  
  K = place(A,B,d_poles)
  A_CL = A - B*K
  
  G_s = ss(A,B,eye(4),zeros(4,2))
  H_s = ss(K)
  L_s = series(H_s, G_s)
  T_s = feedback(L_s,eye(4))
  step(T_s)
  S_s = feedback(eye(4),L_s)
  R_s = feedback()
  
  [K_lqr,S_lqr,clp_lqr] = lqr(A,B,eye(4),eye(2))
  