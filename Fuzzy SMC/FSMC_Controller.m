function [u1,u2,u3,u4,t1,t2] = FSMC_Controller(t, x, y, F, pqr)

global slideSurfs
global Tilt1 Tilt2
global m g l Ix Iy Iz 
global C1 C2 C3 C1p C2p C3p

global eps1 eps2 eps3 eps4
global eta1 eta2 eta3 eta4

global fis1 fis2 fis3 fis4
global u_prev

u1_k_2 = u_prev(1,1);u1_k_1 = u_prev(1,2);
u2_k_2 = u_prev(2,1);u2_k_1 = u_prev(2,2);
u3_k_2 = u_prev(3,1);u3_k_1 = u_prev(3,2);
u4_k_2 = u_prev(4,1);u4_k_1 = u_prev(4,2);

eps1 = 0.2;eps2 = 0.2;eps3 = 0.2;eps4 = 0.2;
eta1 = 2;eta2 = 2;eta3 = 2;eta4 = 2;

desired = X_desired(t);

Xd    = desired(1);
Yd    = desired(4);
Zd    = desired(7);
Phid  = desired(10);
Tetad = desired(13);
Psid  = desired(16);

Xd_dot    = desired(2);
Yd_dot    = desired(5);
Zd_dot    = desired(8);
Phid_dot  = desired(11);
Tetad_dot = desired(14);
Psid_dot  = desired(17);

Xd_ddot    = desired(3);
Yd_ddot    = desired(6);
Zd_ddot    = desired(9);
Phid_ddot  = desired(12);
Tetad_ddot = desired(15);
Psid_ddot  = desired(18);

X_ddot    = y(2);
Y_ddot    = y(4);
Z_ddot    = y(6);
Phi_ddot  = y(8);
Teta_ddot = y(10);
Psi_ddot  = y(12);

p = pqr(1);
q = pqr(2);
r = pqr(3);

F1 = F(1);F2 = F(2);F3 = F(3);F4 = F(4);
[t1,t2] = PD_Controller(x(7), x(8), Phid, Phid_dot,...
                        x(9), x(10), Tetad, Tetad_dot);

% t1 = 0;t2 = 0;
Tilt1 = [Tilt1 t1];
Tilt2 = [Tilt2 t2];

o_z   = 0.2;
o_psi = 0.2;

s1     = o_z*(Zd-x(5))+(Zd_dot-x(6));
s1_dot = o_z*(Zd_dot-x(6))+(Zd_ddot-Z_ddot);

%% u1 
% u1_fuzzy = evalfis(fis1, [Kp*s1 0.3*Kp*s1_dot Z_ddot u1_k_2 u1_k_1]);
Kp_u1 = 6;
u1_fuzzy = evalfis(fis1, [Kp_u1*s1 Kp_u1*s1_dot x(6) Z_ddot Zd_dot Zd_ddot u1_k_2 u1_k_1]);
u1       = 1/(c(x(7))*c(x(9)))*(s1_dot+Z_ddot+...
        1/m*(F2+F4)*s(t2)*s(x(9))+1/m*(F1+F3)*c(x(9))*s(x(7))*s(t1)+...
        g+C3*x(6)+eps1*SignApprox(s1, 3, 2)+eta1*s1);
    
o1 = 11*m/(c(x(7))*c(x(11))*m*u1-(F1+F3)*s(t1)*c(x(11))*s(x(7)));
o2 = 6/11*o1;
o3 = 1;o4 = 6;

o5 = 11*m/(c(x(11))*m*u1+(F1+F3)*s(t1)*s(x(11))*s(x(9)));
o6 = 6/11*o5;
o7 = 1;o8 = 6;

s2     = o_psi*(Psid-x(11))+(Psid_dot-x(12));
s2_dot = o_psi*(Psid_dot-x(12))+(Psid_ddot-Psi_ddot);

%% u4
Kp_u4  = 0.34;
Kd_u4  = 0.01;
Kdd_u4 = -0.004;
% u4_fuzzy = evalfis(fis4, [s2 s2_dot Psi_ddot u4_k_2 u4_k_1]);
u4_fuzzy = evalfis(fis4, [0 Kp_u4*x(11) Kd_u4*x(12) Kdd_u4*Psi_ddot Psid Psid_dot Psid_ddot u4_k_2 u4_k_1]);
u4       = -p*q*(Ix-Iy)/Iz + l*C3p*x(12)/Iz + o_psi*(Psid-x(11))...
        + Psid_ddot + eta2*s2 + eps2*SignApprox(s2, 3, 2);


s3     = o1*(Xd_dot-x(2))+o2*(Xd-x(1))+o3*(Tetad_dot-x(10))+o4*(Tetad-x(9));
s3_dot = o1*(Xd_ddot-X_ddot)+o2*(Xd_dot-x(2))...
    +o3*(Tetad_ddot-Teta_ddot)+o4*(Tetad_dot-x(10));

%% u3
Kp_u3 = 0.15;
Kp_X = 5;

% Kp_u3 = 4;
% Kp_X = 1;
% 
% if t<0.5
%     Kp_u3 = 0;
% end
    
u3_fuzzy = evalfis(fis3, [Kp_u3*s3 Kp_u3*s3_dot Kp_X*X_ddot u3_k_2 u3_k_1]);
% u3_fuzzy = evalfis(fis3, [s3 s3_dot x(2) 0 Xd_dot Xd_ddot u3_k_2 u3_k_1]);
u3 = 1/o3*(s3_dot + o3*Teta_ddot + eta3*s3...
    + eps3*SignApprox(s3, 3, 2) + o3*l*C2p*x(10)/Iy-o3*p*r*(Iz-Ix)/Iy);

s4 = o5*(Yd_dot-x(4))+o6*(Yd-x(3))+o7*(Phid_dot-x(8))+o8*(Phid-x(7));
s4_dot = o5*(Yd_ddot-Y_ddot)+o6*(Yd_dot-x(4))...
        +o7*(Phid_ddot-Phi_ddot)+o8*(Phid_dot-x(8));

%% u2
% u2_fuzzy = evalfis(fis2, [4*s4 4*s4_dot Y_ddot u2_k_2 u2_k_1]);
% Kp_u2 = 0.6;
% Kp_Y  = 5;

Kp_u2 = 2;
Kp_Y  = 3;

if t<0.5
    Kp_u2 = 0;
end

u2_fuzzy = evalfis(fis2, [Kp_u2*s4 Kp_u2*s4_dot x(4) Kp_Y*Y_ddot Yd_dot Yd_ddot u2_k_2 u2_k_1]);
u2 = 1/o7*(s4_dot+o7*Phi_ddot+eps4*SignApprox(s4, 3, 2)...
     + eta4*s4 + o7*l*C1p*x(8)/Ix-o7*q*r/Ix*(Iy-Iz));

%% Fuzzy-SMC
fuzzy = 0;
if fuzzy 
    u1 = u1_fuzzy;
    u2 = u2_fuzzy;
    u3 = u3_fuzzy;
    u4 = u4_fuzzy; 
end
%% Supervisory Control ---> SMC + Fuzzy-SMC 
supervisory = 0;

maxX = 2;maxY = 2;maxZ = 2;
cond1 = abs(x(1)) < maxX && abs(x(3)) < maxY && (x(5) < maxZ && x(5) > 0) ;

maxPhi = deg2rad(2);maxTheta = deg2rad(2);maxPsi = deg2rad(2);
cond2 = abs(x(7)) < maxPhi && abs(x(9)) < maxTheta && abs(x(11)) < maxPsi;

if supervisory && cond1 && cond2 && t > 1
    u1 = u1_fuzzy;
    u2 = u2_fuzzy;
    u3 = u3_fuzzy;
    u4 = u4_fuzzy; 
end

u_prev = [u1_k_1 u1;u2_k_1 u2;u3_k_1 u3;u4_k_1 u4];
slideSurfs = [slideSurfs;[s1 s1_dot s2 s2_dot s3 s3_dot s4 s4_dot]];
end
function cos_ = c(Teta)
cos_ = cos(Teta);
end
function sin_ = s(Teta)
sin_ = sin(Teta);
end
function out = SignApprox(S, Type, a)

if nargin<2
    Type = 1;
end

switch Type
    case 1
        out = sign(S);
    case 2
        phi = a;
        out = Saturation(S/phi, [-1 1]);
    case 3
        coef = a;
        out = tanh(coef*S);
end

end
function out = Saturation(S, bound)

if S > max(bound)
    out = max(bound);
elseif S < min(bound)
    out = min(bound);
else
    out = S;
end

end
function out = Normalization(s, OldBound, NewBound)
    m = (s-min(OldBound))/(max(OldBound)-min(OldBound));
    out = min(NewBound) + m * (max(NewBound)-min(NewBound));
end
