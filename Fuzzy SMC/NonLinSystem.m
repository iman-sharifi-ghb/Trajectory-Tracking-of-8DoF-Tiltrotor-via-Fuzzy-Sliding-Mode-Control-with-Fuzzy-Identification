function y = NonLinSystem(t, x, Omega, y_previous)

global m g l Ix Iy Iz
global C1 C2 C3 C1p C2p C3p
global Kf Km
global u1s u2s u3s u4s

% X=x(1);X-dot=x(2);Y=x(3);Y-dot=x(4);Z=x(5);Z-dot=x(6);
% Phi=x(7);Phi-dot=x(8);Teta=x(9);Teta-dot=x(10);Psi=x(11);Psi-dot=x(12);

Phi  = x(7);
Teta = x(9);
Psi  = x(11);
% t1 = Tilt(1);t2 = Tilt(2);
% Tilts = [Tilt(1);Tilt(2)];
 
F1 = Kf*Omega(1)^2;F2 = Kf*Omega(2)^2;
F3 = Kf*Omega(3)^2;F4 = Kf*Omega(4)^2;
F = [F1;F2;F3;F4];

M1 = Km*Omega(1)^2;M2 = Km*Omega(2)^2;
M3 = Km*Omega(3)^2;M4 = Km*Omega(4)^2;

% u1 = 1/m*(F1*c(t1) + F2*c(t2) + F3*c(t1) + F4*c(t2));
% u2 = 1/Ix*(M2+M4)*s(t2) + l/Ix*(F2-F4)*c(t2);
% u3 = 1/Iy*(M1+M3)*s(t1) + l/Iy*(F1-F3)*c(t1);
% u4 = -1/Iz*(M1*c(t1)-M2*c(t2)+M3*c(t1)-M4*cos(t2))...
%      -l/Iz*(F1*s(t1)-F2*s(t2)-F3*s(t1)-F4*s(t2));
 
R1 = [1 s(Psi)*tan(Teta) c(Phi)*tan(Teta);
      0    c(Phi)        -s(Phi)         ;
      0 s(Phi)*sec(Teta) c(Phi)*sec(Teta)];
% det(R1)
pqr = R1\[x(8);x(10);x(12)];
p = pqr(1);
q = pqr(2);
r = pqr(3);

[u1,u2,u3,u4,t1,t2] = FSMC_Controller(t, x, y_previous, F, pqr);
u1s=[u1s u1]; u2s=[u2s u2]; u3s=[u3s u3]; u4s=[u4s u4];

% Equations
y(1) = x(2);
y(2) = 1/m*(F1+F3)*s(t1)*(c(Phi)*s(Psi)-c(Psi)*s(Phi)*s(Teta))...
    -C1*x(2)+(s(Phi)*s(Psi)+c(Phi)*c(Psi)*s(Teta))*u1...
    + 1/m*(F2+F4)*(s(t2)*c(Psi)*c(Teta));
y(3) = x(4);
y(4) = -1/m*(F1+F3)*s(t1)*(c(Phi)*c(Psi)-s(Psi)*s(Phi)*s(Teta))...
    -C2*x(4)+(s(Phi)*c(Psi)-c(Phi)*s(Psi)*s(Teta))*u1 ...
    + 1/m*(F2+F4)*(s(t2)*s(Psi)*c(Teta));
y(5) = x(6);
y(6) = -1/m*(F2+F4)*s(t2)*s(Teta)-1/m*(F1+F3)*c(Teta)*s(Phi)*s(t1)...
    -g-C3*x(6)+c(Phi)*c(Teta)*u1;
y(7)  = x(8);
y(8)  = u2+q*r*(Iy-Iz)/Ix-l*C1p*x(8)/Ix;
y(9)  = x(10);
y(10) = u3+p*r*(Iz-Ix)/Iy-l*C2p*x(10)/Iy;
y(11) = x(12);
y(12) = u4+p*q*(Ix-Iy)/Iz-l*C3p*x(12)/Iz;

end
function cos_ = c(Teta)
cos_ = cos(Teta);
end
function sin_ = s(Teta)
sin_ = sin(Teta);
end

