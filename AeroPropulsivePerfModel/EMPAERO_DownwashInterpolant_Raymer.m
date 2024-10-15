function [aero_out] = EMPAERO_DownwashInterpolant_Raymer(forward_LS_aero,forward_LS_geom,aft_LS_geom,Vehicle)
% half_wing geometry
plotFlag = 0;
showResult = 0;
% alpha = wing_aero.AOA
% CL = wing_aero.CL


% Canard characteristics
AR = 2*forward_LS_geom.AspectRatio; 
TR = forward_LS_geom.TaperDefn(2,2);
% sweep = forward_LS_geom.Sweep_qc_r2t;
inc_wing = forward_LS_geom.RootIncidence;
% dihedral = forward_LS_geom.Dihedral(2,1);                                      % dihedraL angle, +ve tip up
b = 2* forward_LS_geom.Span;
% S = 2* forward_LS_geom.PlanformArea;
MAC = forward_LS_geom.MAC;
xLE_MAC = forward_LS_geom.LE_MAC(1);

Cr = forward_LS_geom.Stn.c(1);          
xTE_rootchord = forward_LS_geom.Stn.xTE(1);                                      % distance of the root chord AC from reference point 
xAC_tipchord = forward_LS_geom.Stn.xQC(end);

% Horizontal tail characteristics 
bH = 2*aft_LS_geom.Span;                                                      % span of the horizontal tail 
bfor = b;
if bH>bfor
    bH = bfor;
end

% Additional Characteristics 
% l2 = xTE_rootchord - aft_LS_geom.LocMAC(1);                                   % aft end of the wing root chord to the quarter-chord point of hte MAC of the horizontal tail 
% l3 = xLE_MAC-xTE_rootchord;
% leff = xAC_tipchord-aft_LS_geom.LocMAC(1);
% Mach = 0.17;

% refer to the Fig 16.12 on Raymer (2018) reader pg. 634
% horizontal distance betwen the forward and after surface MACs
Lt_horizontal = abs(aft_LS_geom.LocMAC(1)-xLE_MAC);

% vertical distance betwen the forward and after surface MACs
zHS =  aft_LS_geom.LocMAC(3);
zW =  forward_LS_geom.LocMAC(3);
zHS_from_W = zHS-zW; % FD coordinate ( +ve down) 

zero_lift_AOA = forward_LS_aero.zero_lift_AOA(1); % degree 
root_angle = inc_wing-zero_lift_AOA;
% 
z1 = Lt_horizontal * tand(root_angle);
z2 = z1 + abs(zHS_from_W); % vertical distance of the aft surface from zero-lift line 
Lt_1 = z2 * sind(root_angle);
% 
% Lenght of aft surface from forward surface along the zero-lift line
Lt = Lt_horizontal/cosd(root_angle) - Lt_1;
zHS_from_zero_lift_line = z2*cosd(root_angle);
Zt = zHS_from_zero_lift_line ;    
%%
m = Zt/ (b/2);
r = Lt/ (b/2);

x_test =  r;
dim = length(x_test);
m_vec = m* ones(dim,1);
TR_vec = TR *  ones(dim,1);
AR_vec = AR * ones(dim,1);
depsilon_dalpha = Vehicle.Aero.GI_deda_raymer(x_test,m_vec,TR_vec,AR_vec);
%%
% Epsilon at first step
% Epsilon after frist time step
dA = 2;
max_lift_AOA = forward_LS_aero.max_lift_AOA; % degree
alpha = [zero_lift_AOA:dA:max_lift_AOA]';
epsilon_0 = 0;
max = size(alpha,1);
epsilon = zeros(max,1);
epsilon(1) = epsilon_0;
for i=2:max
    epsilon(i) = epsilon(i-1) + depsilon_dalpha * dA;
end

forward_LS_aero.downwash.GI_epsilon_raymer = griddedInterpolant(alpha,epsilon,'linear','linear');
aero_out = forward_LS_aero;
%%
if showResult == 1
   table (depsilon_dalpha)
   table(alpha,epsilon)
end 
end 

