function [Vbdot,Omdot,QuatsDot,LLRdot] =External_AircraftEOM_OffsetFromCG(Fb,Mb,mass,Ib,Ibdot,r_cg,v_cg,a_cg,Om,Quats,Vb,LLR,hb,hbdot)



% r_cg = [0;0;0];


%% CONSTANTS
OmEarth = 7.292115e-5;



%% Split input vectors
Fx = Fb(1);
Fy = Fb(2);
Fz = Fb(3);

p = Om(1);
q = Om(2);
r = Om(3);
% phi = PTS(1);
% theta = PTS(2);
% psi = PTS(3);
u = Vb(1);
v = Vb(2);
w = Vb(3);


Ixx = Ib(1,1);
Ixy = Ib(1,2);
Ixz = Ib(1,3);
Iyy = Ib(2,2);
Iyz = Ib(2,3);
Izz = Ib(3,3);

Lat = LLR(1);
%Long = LLR(2);
Rgeo = LLR(3);


e0 = Quats(1);
e1 = Quats(2);
e2 = Quats(3);
e3 = Quats(4);


%% Rotation tensors involving Euler angles
% LBV = [...
%     cos(theta)*cos(psi),                                cos(theta)*sin(psi),                                -sin(theta);
%     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     sin(phi)*cos(theta);
%     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),     cos(phi)*cos(theta)];


LBV = [...
    e0^2+e1^2-e2^2-e3^2,                2*(e1*e2+e0*e3),                2*(e1*e3-e0*e2);
    2*(e1*e2-e0*e3),                    (e0^2-e1^2+e2^2-e3^2),          2*(e2*e3+e0*e1);
    2*(e1*e3+e0*e2),                    2*(e2*e3-e0*e1),                e0^2-e1^2-e2^2+e3^2];



LVB = LBV';


%% Derivatives of Position
Xedot = LVB * [u;v;w];

% Etkin 5.3.6, rpg 72
% RHS = LVB [u;v;w];
% LatDot = RHS(1)/Rgeo;
% LongDot = RHS(2)/(Rgeo*cos(Lat))
% RDot = - RHS(3)

LatDot = Xedot(1)/Rgeo;
LongDot = Xedot(2)/(Rgeo*cos(Lat));
RgeoDot = - Xedot(3);

LLRdot = [LatDot;LongDot;RgeoDot];


%% RELATIVE RATES ACCOUNTING FOR EARTH CURVATURE AND ROTATION
% Etkin, 5.2,10, rpg 70
PQR = [p;q;r] - LBV * [(OmEarth + LongDot)*cos(Lat);    -LatDot;    -(OmEarth + LongDot)*sin(Lat)];
P = PQR(1); Q = PQR(2); R = PQR(3);


%% Derivatives of Euler Angles
% phi_dot     = P + Q*sin(phi)*tan(theta) + R*cos(phi)*tan(theta);
% theta_dot   = Q*cos(phi) - R*sin(phi);
% psi_dot     = Q*sin(phi)*sec(theta) + R*cos(phi)*sec(theta);
% 
% PTSdot = [phi_dot,theta_dot,psi_dot]';



C = 1 - Quats'*Quats;


QuatMat = [...
    C,  -P, -Q, -R;
    P,   C,  R, -Q;
    Q,  -R,  C,  P;
    R,   Q, -P,  C];


QuatsDot = 0.5*QuatMat*Quats;





% 
%% The Force Equations
lambda_x = Fx/mass - q*w + r*v - a_cg(1) - 2*q*v_cg(3)...
    + 2*r*v_cg(2) + r_cg(1)*(q^2+r^2) - r_cg(2)*p*q - r_cg(3)*p*r;
lambda_y = Fy/mass - r*u + p*w - a_cg(2) - 2*r*v_cg(1)...
    + 2*p*v_cg(3) + r_cg(1)*p*q + r_cg(2)*(p^2+r^2) - r_cg(3)*q*r;
lambda_z = Fz/mass - p*v + q*u - a_cg(3) - 2*p*v_cg(2)...
    + 2*q*v_cg(1) - r_cg(1)*p*r - r_cg(2)*q*r + r_cg(3)*(p^2+q^2);

%% The Moment Equations


% Trying out in matrix form:
Omtilde = [0, -r, q;  r, 0, -p;  -q, p, 0];
r_cgtilde = [0, -r_cg(3), r_cg(2);   r_cg(3), 0, -r_cg(1);  -r_cg(2), r_cg(1), 0];



mu = Mb - Ibdot*Om - Omtilde * Ib * Om - hbdot - Omtilde*hb - mass*r_cgtilde*Omtilde*Vb;


%% Put Force and Moments together
right = [lambda_x;lambda_y;lambda_z;mu];
D = [1,0,0,0,r_cg(3),-r_cg(2);...
    0,1,0,-r_cg(3),0,r_cg(1);...
    0,0,1,r_cg(2),-r_cg(1),0;...
    0,-mass*r_cg(3),mass*r_cg(2),Ixx,Ixy,Ixz;...
    mass*r_cg(3),0,-mass*r_cg(1),Ixy,Iyy,Iyz;...
    -mass*r_cg(2),mass*r_cg(1),0,Ixz,Iyz,Izz];


left = D \ right;



Vbdot = [left(1),left(2),left(3)]';
Omdot = [left(4),left(5),left(6)]';

end

% x_ECEF_dot_v3(1,1) = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))...
%     + w*(cos(phi)*sin(theta)*sin(psi)+sin(phi)*sin(psi));
% x_ECEF_dot_v3(2,1) = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))...
%     + w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
% x_ECEF_dot_v3(3,1) = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

% Xedot = [...
%     u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))+ w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
%     u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))+ w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
%     -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta)];


% mu_x = Mx - (Idotxx*P+Idotxy*Q+Idotxz*R)...
%     - (P*(Q*Ixz-R*Ixy)+Q*(Q*Iyz-R*Iyy)+R*(Q*Izz-R*Iyz)) - hbdot(1)...
%     - (Q*hb(3)-R*hb(2))...
%     - mass*(Vb(1)*(-r_cg(3)*R-r_cg(2)*Q)+Vb(2)*r_cg(2)*P+Vb(3)*r_cg(3)*Q);
% 
% mu_y = My - (Idotxy*P+Idotyy*Q+Idotyz*R)...
%     - (P*(R*Ixx-P*Ixz)+Q*(R*Ixy-P*Iyz)+R*(R*Ixz-P*Izz)) - hbdot(2)...
%     - (R*hb(2)-Q*hb(1))...
%     - mass*(Vb(1)*r_cg(1)*Q+Vb(2)*(-r_cg(3)*R-r_cg(1)*P)+Vb(3)*r_cg(3)*Q);
% 
% mu_z = Mz - (Idotxz*P+Idotyz*Q+Idotzz*R)...
%     - (P*(P*Ixz-Q*Ixx)+Q*(P*Iyy-Q*Ixz)+R*(P*Iyz-Q*Ixz)) - hbdot(3)...
%     - (P*hb(2)-Q*hb(1))...
%     - mass*(Vb(1)*r_cg(1)*R+Vb(2)*r_cg(2)*R+Vb(3)*(-r_cg(2)*Q-r_cg(1)*Q));
% 
% mu = [mu_x;mu_y;mu_z];