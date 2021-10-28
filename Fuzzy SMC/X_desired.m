function out = X_desired(t)
global desiredStates
global Type
% Type = 1;
% Type = 2; % Train
% Type = 3; % Test

switch Type
    case 1
       
        Xd = 1;
        Xd_dot = 0;
        Xd_ddot = 0;

        Yd = 1;
        Yd_dot = 0;
        Yd_ddot = 0;

        Zd = 1;
        Zd_dot = 0;
        Zd_ddot = 0;

        a = deg2rad(0);
        Phid = a;%a*sin(t);
        Phid_dot = 0;%a*cos(t);
        Phid_ddot = 0;%-a*sin(t);

        b = deg2rad(0);
        Tetad = b;
        Tetad_dot = 0;
        Tetad_ddot = 0;

        c = deg2rad(1);
        Psid = c;
        Psid_dot = 0;
        Psid_ddot = 0;
    
    case 2

        r = 6;
        f1 = 2*pi/10;
        Xd = r*cos(f1*t);
        Xd_dot = -r*f1*sin(f1*t);
        Xd_ddot = -r*f1^2*cos(f1*t);

        Yd = r*sin(f1*t);
        Yd_dot = r*f1*cos(f1*t);
        Yd_ddot = -r*f1^2*sin(f1*t);

        %Helix
    %         b = 0.25;
    %         Zd = b*t;
    %         Zd_dot = b;
    %         Zd_ddot = 0;

        Zd = 4*(1+cos(f1*t));
        Zd_dot = -5*f1*sin(f1*t);
        Zd_ddot = -5*f1^2*cos(f1*t);

        a = deg2rad(10);
        f2 = 2*pi/15;
        Phid = a*sin(f2*t);
        Phid_dot = a*f2*cos(f2*t);
        Phid_ddot = -a*f2^2*sin(f2*t);

        Tetad = a*sin(f2*t);
        Tetad_dot = a*f2*cos(f2*t);
        Tetad_ddot = -a*f2^2*sin(f2*t);

        c = deg2rad(30); 
        Psid = c*sin(f1*t);
        Psid_dot = c*f1*cos(f1*t);
        Psid_ddot = -c*f1^2*sin(f1*t);

    case 3

        r = 1;
        f1 = 2*pi/10;
        Xd = r*sin(f1*t);
        Xd_dot = r*f1*cos(f1*t);
        Xd_ddot = -r*f1^2*sin(f1*t);

        Yd = r*cos(f1*t);
        Yd_dot = -r*f1*sin(f1*t);
        Yd_ddot = -r*f1^2*cos(f1*t);

        %Helix
        b = 0.1;
        Zd = b*t;
        Zd_dot = b;
        Zd_ddot = 0;

%         Zd = 0.2*(1+sin(f1*t));
%         Zd_dot = 1*f1*cos(f1*t);
%         Zd_ddot = -1*f1^2*sin(f1*t);

        a = deg2rad(0);
        f2 = 2*pi/15;
        Phid = a*cos(f2*t);
        Phid_dot = -a*f2*sin(f2*t);
        Phid_ddot = -a*f2^2*cos(f2*t);

        Tetad = a*cos(f2*t);
        Tetad_dot = -a*f2*sin(f2*t);
        Tetad_ddot = -a*f2^2*cos(f2*t);

        c = deg2rad(0); 
        Psid = c*cos(f1*t);
        Psid_dot = -c*f1*sin(f1*t);
        Psid_ddot = -c*f1^2*cos(f1*t);
end

out = [Xd;Xd_dot;Xd_ddot;Yd;Yd_dot;Yd_ddot;Zd;Zd_dot;Zd_ddot;
       Phid;Phid_dot;Phid_ddot;Tetad;Tetad_dot;Tetad_ddot;
       Psid;Psid_dot;Psid_ddot]; 

desiredStates = [desiredStates;out.'];

end

