function [tilt1,tilt2]=PD_Controller(Phi,Phi_dot,Phid,Phid_dot,Teta,Teta_dot,Tetad,Tetad_dot)

Kp1 = 100;%0.8*Ku;
Kd1 = 5;%Ku*Tu/10;

Ku = 20;
Tu = 2.06;
Kp2 = -0.8*Ku;
Kd2 = -Ku*Tu/10;

Ku_roll = 20;
Kph_roll  = Ku_roll;
Kdh_roll  = 5;

Ku_pitch = 5;
Kph_pitch = Ku_pitch;
Kdh_pitch = 0;

Phidh  = Phid;
Tetadh = Tetad;

e_Teta     = Tetad-Teta;
e_dot_Teta = Tetad_dot-Teta_dot;

e_Phi      = Phid-Phi;
e_dot_Phi  = Phid_dot-Phi_dot; 

delta_Tetad = Kp1*e_Teta + Kd1*e_dot_Teta;
delta_Phid  = Kp2*e_Phi  + Kd2*e_dot_Phi ;

if Phidh~=0
    t1_hover = -(2*Phidh + Kph_roll*(Phidh-Phi) +...
                Kdh_roll*(Phid_dot-Phi_dot));
    t1_roll  = 0;
else
    t1_roll  = delta_Phid;
    t1_hover = 0;
end

if Tetadh~=0
    t2_hover = +(2*Tetadh + Kph_pitch*(Tetadh-Teta) +...
                Kdh_pitch*(Tetad_dot - Teta));
    t2_pitch = 0;
else
    t2_pitch = delta_Tetad;
    t2_hover = 0;
end

u5 = t1_hover+t1_roll;
u6 = t2_hover+t2_pitch;

tilt1 = -u5;tilt2 = -u6;

% b = deg2rad(75);
% if abs(tilt1)>b
%     tilt1 = b*sign(tilt1);
% end
% if abs(tilt2)>b
%     tilt2 = b*sign(tilt2);
% end

end
