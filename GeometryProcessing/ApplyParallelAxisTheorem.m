function [Ixx_bf_rp,Iyy_bf_rp,Izz_bf_rp,Ixy_bf_rp,Iyz_bf_rp,Izx_bf_rp] = ApplyParallelAxisTheorem(Mass,CG,Ica,R)

Mass = reshape(Mass,[length(Mass),1]);


% capture elements of inertia tensor about body CG and centroidal axes
Ixx = Ica(:,1);
Iyy = Ica(:,2);
Izz = Ica(:,3);
Ixy = Ica(:,4);
Iyz = Ica(:,5);
Ixz = Ica(:,6);

R11 = R(:,1);
R12 = R(:,2);
R13 = R(:,3);
R21 = R(:,4);
R22 = R(:,5);
R23 = R(:,6);
R31 = R(:,7);
R32 = R(:,8);
R33 = R(:,9);

% transform inertia tensor
Icm_bf_11 = R11.*(Ixx.*R11 + Ixy.*R12 + Ixz.*R13) + R12.*(Ixy.*R11 + Iyy.*R12 + Iyz.*R13) + R13.*(Ixz.*R11 + Iyz.*R12 + Izz.*R13);
Icm_bf_12 = R21.*(Ixx.*R11 + Ixy.*R12 + Ixz.*R13) + R22.*(Ixy.*R11 + Iyy.*R12 + Iyz.*R13) + R23.*(Ixz.*R11 + Iyz.*R12 + Izz.*R13);
% Icm_bf_13 = R31.*(Ixx.*R11 + Ixy.*R12 + Ixz.*R13) + R32.*(Ixy.*R11 + Iyy.*R12 + Iyz.*R13) + R33.*(Ixz.*R11 + Iyz.*R12 + Izz.*R13);

% Icm_bf_21 = R11.*(Ixx.*R21 + Ixy.*R22 + Ixz.*R23) + R12.*(Ixy.*R21 + Iyy.*R22 + Iyz.*R23) + R13.*(Ixz.*R21 + Iyz.*R22 + Izz.*R23);
Icm_bf_22 = R21.*(Ixx.*R21 + Ixy.*R22 + Ixz.*R23) + R22.*(Ixy.*R21 + Iyy.*R22 + Iyz.*R23) + R23.*(Ixz.*R21 + Iyz.*R22 + Izz.*R23);
Icm_bf_23 = R31.*(Ixx.*R21 + Ixy.*R22 + Ixz.*R23) + R32.*(Ixy.*R21 + Iyy.*R22 + Iyz.*R23) + R33.*(Ixz.*R21 + Iyz.*R22 + Izz.*R23);

Icm_bf_31 = R11.*(Ixx.*R31 + Ixy.*R32 + Ixz.*R33) + R12.*(Ixy.*R31 + Iyy.*R32 + Iyz.*R33) + R13.*(Ixz.*R31 + Iyz.*R32 + Izz.*R33);
% Icm_bf_32 = R21.*(Ixx.*R31 + Ixy.*R32 + Ixz.*R33) + R22.*(Ixy.*R31 + Iyy.*R32 + Iyz.*R33) + R23.*(Ixz.*R31 + Iyz.*R32 + Izz.*R33);
Icm_bf_33 = R31.*(Ixx.*R31 + Ixy.*R32 + Ixz.*R33) + R32.*(Ixy.*R31 + Iyy.*R32 + Iyz.*R33) + R33.*(Ixz.*R31 + Iyz.*R32 + Izz.*R33);
 

% MassProp = table();
% MassProp.Name = {Name};
% MassProp.Mass = Mass;
% MassProp.xcg = CG(1);
% MassProp.ycg = CG(2);
% MassProp.zcg = CG(3);


% phi = Angles(1);
% theta = Angles(2);
% psi = Angles(3);


% % formulate rotation tensor rotating body-fixed basis to centroidal basis
% Rpsi = [cosd(psi),-sind(psi),0;   sind(psi), cosd(psi), 0;   0,0,1];
% Rtheta = [cosd(theta), 0, sind(theta);  0, 1, 0;  -sind(theta), 0, cosd(theta)];
% Rphi = [1,0,0; 0, cosd(phi),-sind(phi); 0, sind(phi), cosd(phi)];
% 
% Rnet = Rpsi * Rtheta * Rphi;

% % capture elements of inertia tensor about body CG and centroidal axes
% Ixx_ca = Ica(1);
% Iyy_ca = Ica(2);
% Izz_ca = Ica(3);
% Ixy_ca = Ica(4);
% Iyz_ca = Ica(5);
% Izx_ca = Ica(6);
% 
% % Form the inertia tensor about the elements center of mass, and
% % referred to the elements centroidal axes
% Icm_ca = [Ixx_ca, Ixy_ca, Izx_ca; Ixy_ca, Iyy_ca, Iyz_ca; Izx_ca, Iyz_ca, Izz_ca];
% 
% % Rotate this inertia tensor to obtain the inertia tensor of the
% % element about the element's center of mass, but with respect to the global body-fixed
% % axes
% Icm_bf = Rnet*Icm_ca*Rnet';

% Apply the parallel axis theorem to compute the moment of inertia
% tensor of the element about the global body-fixed origin and with
% respect to the global body-fixed axes

xcm = CG(:,1);
ycm = CG(:,2);
zcm = CG(:,3);

% 1. moments of inertia
Ixx_bf_rp = Icm_bf_11 + Mass.*(ycm.^2+zcm.^2);
Iyy_bf_rp = Icm_bf_22 + Mass.*(xcm.^2+zcm.^2);
Izz_bf_rp = Icm_bf_33 + Mass.*(xcm.^2+ycm.^2);

% 2. products of inertia
Iyz_bf_rp = Icm_bf_23 - Mass.*ycm.*zcm;
Izx_bf_rp = Icm_bf_31 - Mass.*xcm.*zcm;
Ixy_bf_rp = Icm_bf_12 - Mass.*xcm.*ycm;

