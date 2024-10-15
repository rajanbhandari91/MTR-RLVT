function [aero_out] = EMPAERO_DownwashInterpolant(forward_LS_aero,forward_LS_geom,aft_LS_geom,Vehicle)
% half_wing geometry
plotFlag = 0;
showResult = 0;
% alpha = wing_aero.AOA
% CL = wing_aero.CL


% Wing Characteristics
AR = 2*forward_LS_geom.AspectRatio; 
TR = forward_LS_geom.TaperDefn(2,2);
sweep = forward_LS_geom.Sweep_qc_r2t;
inc_wing = forward_LS_geom.RootIncidence;
dihedral = forward_LS_geom.Dihedral(2,1);                                      % dihedraL angle, +ve tip up
b = 2* forward_LS_geom.Span;
S = 2* forward_LS_geom.PlanformArea;
MAC = forward_LS_geom.MAC;
xLE_MAC = forward_LS_geom.LE_MAC(1);

Cr = forward_LS_geom.Stn.c(1);          
xTE_rootchord = forward_LS_geom.Stn.xTE(1);                                      % distance of the root chord AC from reference point 
xAC_tipchord = forward_LS_geom.Stn.xQC(end);
% Airfoil Characterisitcs 

% Horizontal tail Characteristics 
bH = 2*aft_LS_geom.Span;                                                      % span of the horizontal tail 
bfor = 2* forward_LS_geom.Span;
if bH>bfor
    bH = bfor;
end

% Additional Characteristics 
l2_h = xTE_rootchord - aft_LS_geom.LocMAC(1);                                   % aft end of the wing root chord to the quarter-chord point of hte MAC of the horizontal tail 
l2 = l2_h/cosd(inc_wing);                                                       % l2 should be measured parallel to winf root chord
l3 = xLE_MAC-xTE_rootchord;
leff_h = xAC_tipchord-aft_LS_geom.LocMAC(1);
leff = leff_h/cosd(inc_wing);
% Mach = 0.17;
zero_lift_AOA = forward_LS_aero.zero_lift_AOA; % degree
max_lift_AOA = forward_LS_aero.max_lift_AOA; % degree
%%
% horizontal distance betwen the forward and after surface MACs
Lt_horizontal = abs(aft_LS_geom.LocMAC(1)-xLE_MAC);
z1 = Lt_horizontal * tand(inc_wing); % +ve when incidence angle is +ve

zHS =  aft_LS_geom.LocMAC(3);
zW =  forward_LS_geom.LocMAC(3);
zHS_from_W = -zW+zHS;
zHS_from_wingRC = -z1 + zHS_from_W;
hH = - zHS_from_wingRC ;                                                          % height of the horizontal tail MAC quarter-chord point +ve above (contrasts with FD coordinate) from wing-level 
%%
dA = 2;
 alpha = [zero_lift_AOA:dA:max_lift_AOA]';
 stripLen = forward_LS_aero.LS.stripLen;

 for i =1:length(alpha)
     G = forward_LS_aero.LS.G_to_alpha*(alpha(i)-zero_lift_AOA)/57.3;                                 % G_to_alpha is in radian
     CL(i) = 4*b/S* sum(G.*stripLen);
 end 
CL = CL(:);
%% 
% step 1
% x1 = AOA_parameter 
% x2 = Sweep angle 


AOA_param = (alpha - zero_lift_AOA+0.0001)/(max_lift_AOA-zero_lift_AOA);
sweep_param = sweep * ones(size(AOA_param));
inter1 = Vehicle.Aero.GI_EffectiveWingAR_S1(AOA_param,sweep_param);

% x1 = inter1
% x2 = Taper ratio
TR_param = TR * ones(size(inter1));
AR_eff_to_AR = Vehicle.Aero.GI_EffectiveWingAR_S2(inter1,TR_param);
AR_eff = AR * AR_eff_to_AR;

% x1 = AR_eff_to_AR
% x2 = Taper ratio
TR_param = TR * ones(size(AR_eff_to_AR));
b_eff_to_b = Vehicle.Aero.GI_EffectiveWingSpan(AR_eff_to_AR,TR_param);
b_eff = b * b_eff_to_b;
%% Step2: Downwash at the plane of symmetry and height of vortex core
length_norm = l2/(b/2);
% depsilon_dalpha_inf_calc = (2* 57.3)/ (pi *AR)*C_L_alpha;

% x1 = l2/(b/2) Tail length in semispans 
% x2 = AR_eff
% Output is some intermediate value

length_param = length_norm * ones (size(AR_eff));
depsilon_dalpha_inf_1 =Vehicle.Aero.GI_deda_S1 (length_param,AR_eff);

% Digitization of DATCOM Fig. 4.4.1-67 (quadrant III), pg. 4.4.1-67 (Reader pg. 1269)
% x1 = AR_eff
% x2 = Sweep
% Output is downwash greadient at infinity
% second independent variable
  
depsilon_dalpha_inf_2 = Vehicle.Aero.GI_deda_S2 (AR_eff,sweep_param);
height =(0.8-depsilon_dalpha_inf_1).*(1-depsilon_dalpha_inf_2);
depsilon_dalpha_vortex = depsilon_dalpha_inf_2+ height ;
%% Step3: Vertical position of the vortex core based on dy 
% delta y as percentage of chord length( DATCOM pg. 2.2.1-8)
% calculated from airfoil half_wingetry in this implementation

y1 = forward_LS_geom.AirfoilGeomGI.GI_AF_xy_top(0.15/100);
y2 = forward_LS_geom.AirfoilGeomGI.GI_AF_xy_top(6/100);
dy = (y2-y1)*100;
dy_max = 3;
if dy>dy_max
    dy = dy_max;
end 
 
% Digitization of DATCOM Fig. 4.4.1-68a, pg. 4.4.1-68a (Reader pg. 1270)
% based on value of dy, find out whether 
% LE seperation is predominant(Region: 2) or 
% TE seperation is predominant (Region: 3)

P = [sweep,dy];
Region = EMPAERO_identifySeperationType(P,Vehicle.Aero.FlowSepType,plotFlag);
% negative value of CL will result in complex number in b_v calculation 
% negative CL can result when angle of attack is very high and negative.
CL (CL<0) = 0;

temp = (0.41*CL)./(pi*AR_eff);
if Region == 1 || Region == 2
    a= hH-(l2+l3)*(alpha/57.3-temp)-b_eff/2* tan(dihedral);
end
if Region == 3
    a= hH-(leff)*(alpha/57.3-temp)-b_eff/2* tan(dihedral);
end
if isnan(Region)
    disp("The query point outside the scope of data")
end
%% Step 4: Span of the vortices at the longitudinal location of the quarter-chord point of hte horizontal-tail MAC
b_v_ru = (0.78+0.10*(TR-0.4)+0.003*sweep)* b_eff;                           % sweep in degree
C_ru = 0.56 * AR./CL;
b_v = b_eff - (b_eff-b_v_ru).*sqrt((2*leff)./(b*C_ru));
%% step 5: Average downwash gradient acting on the tail 
% x1
x1 = bH./b_v;
% x2
x2 = 2*a./b_v;
depsilon_dalpha_ratio =Vehicle.Aero.GI_deda_ratio(x1,abs(x2));
depsilon_dalpha = depsilon_dalpha_ratio .* depsilon_dalpha_vortex;
deda = depsilon_dalpha;

% step 6 
%%
% step 7 
% Epsilon at first step
% Epsilon after frist time step
epsilon_0 = 0;
K1 = depsilon_dalpha;
K2 = [depsilon_dalpha(2:end);0];
m = (K1+K2)/2;
max = size(m,1);
epsilon = zeros(max,1);
epsilon(1) = epsilon_0;
for i=2:max
    epsilon(i) = epsilon(i-1) + m(i-1) * dA;
end
forward_LS_aero.downwash.GI_deda = griddedInterpolant(alpha,deda,'linear','linear');
forward_LS_aero.downwash.GI_epsilon = griddedInterpolant(alpha,epsilon,'linear','linear');
aero_out = forward_LS_aero;
%%
if showResult == 1
    solution1 = table (alpha,AOA_param,AR_eff_to_AR,b_eff_to_b,AR_eff,b_eff,depsilon_dalpha_vortex,CL)
    solution2 = table (alpha,temp,a,C_ru,b_v_ru,b_v)
    solution3 = table(alpha,x2,x1,depsilon_dalpha_ratio,depsilon_dalpha,epsilon)
end 
end 

