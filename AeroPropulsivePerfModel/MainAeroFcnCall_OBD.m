function [dFx_AC_b,dFy_AC_b,dFz_AC_b,dMx_AC_b,dMy_AC_b,dMz_AC_b,AOA_eff] = MainAeroFcnCall_OBD(a,rho,Vehicle,V_s,AOA_dir,Controls,th_ROM)


%%%% STRIP THEORY IMPLEMENTATION
% STRIP AFFILIATIONS
%% CAPTURE STRIP INDICES and PROPERTIES
% Left wing, root to tip
LWIndices = Vehicle.Aero.Indices.LWing;
% Right wing, root to tip
RWIndices = Vehicle.Aero.Indices.RWing;
% Left horizontal stab, root to tip
LHTIndices = Vehicle.Aero.Indices.LHTail;
% Right horizontal stab, root to tip
RHTIndices = Vehicle.Aero.Indices.RHTail;
% Left Vertical stab, root to tip
VTIndices = Vehicle.Aero.Indices.VS;


% CONTROL SURFACE LOCATIONS
OBDLWFlaperon = Vehicle.Aero.Indices.OBD_Flaperon_on_LWing;
MBDLWFlaperon = Vehicle.Aero.Indices.MBD_Flaperon_on_LWing;
IBDLWFlaperon = Vehicle.Aero.Indices.IBD_Flaperon_on_LWing;
IBDRWFlaperon = Vehicle.Aero.Indices.IBD_Flaperon_on_RWing;
MBDRWFlaperon = Vehicle.Aero.Indices.MBD_Flaperon_on_RWing;
OBDRWFlaperon = Vehicle.Aero.Indices.OBD_Flaperon_on_RWing;
RudIndices = Vehicle.Aero.Indices.Rudder_on_VS;


FlaperonFCR = Vehicle.Aero.ControlChordFracs.IBD_Flaperon_on_RWing;
RudderFCR = Vehicle.Aero.ControlChordFracs.Rudder_on_VS;


% % % Phi Rotations of the Components for ROM data collection 
% PhiROM(LCanIndices,:) = 0;
% PhiROM(RCanIndices,:) = 0;
PhiROM(LWIndices,:) = 0;
PhiROM(RWIndices,:) = 0;
PhiROM(LHTIndices,:) = 0;
PhiROM(RHTIndices,:) = 0;
PhiROM(VTIndices,:) = -90;


R_BR_11 = cosd(th_ROM);
R_BR_12 = 0;
R_BR_13 = sind(th_ROM);
R_BR_21 = sind(PhiROM).*sind(th_ROM);
R_BR_22 = cosd(PhiROM);
R_BR_23 = -cosd(th_ROM).*sind(PhiROM);
R_BR_31 = -cosd(PhiROM).*sind(th_ROM);
R_BR_32 = sind(PhiROM);
R_BR_33 = cosd(PhiROM).*cosd(th_ROM);


% Computing velocities in strip frame from the total velocity and AOA_dir
% 
ws_s = V_s.*sind(AOA_dir);
us_s = V_s.*cosd(AOA_dir);

dA = Vehicle.Aero.StripDefn.Area;      % m2

c_strip = Vehicle.Aero.StripDefn.Chord; % m


%% CAPTURE Control vector

% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left MBD flaperon deflection, deg, positive TED
% 3. Left IBD flaperon deflection, deg, positive TED
% 4. Right IBD flaperon deflection, deg, positive TED
% 5. Right MBD flaperon deflection, deg, positive TED
% 6. Right OBD flaperon deflection, deg, positive TED
% 7. Stabilator deflection, deg, positive TED
% 8. Rudder deflection, deg, positive TER

% 9.  dt1, Prop deflection angle, deg, [0 - 90]
% 10. dt2, Prop deflection angle, deg, [0 - 90]
% 11. dt3, Prop deflection angle, deg, [0 - 90]
% 12. dt4, Prop deflection angle, deg, [0 - 90]

% 13. n1, Prop 1 RPM
% 14. n2, Prop 2 RPM
% 15. n3, Prop 3 RPM
% 16. n4, Prop 4 RPM
% 17. n5, Prop 5 RPM
% 18. n6, Prop 6 RPM


% 19. c1,  Prop 1 Pitch
% 20. c2,  Prop 2 Pitch
% 21. c3,  Prop 3 Pitch
% 22. c4,  Prop 4 Pitch
% 23. c5,  Prop 5 Pitch
% 24. c6,  Prop 6 Pitch





% capture flaperon deflections
dF1 = Controls(1);
dF2 = Controls(2);
dF3 = Controls(3);
dF4 = Controls(4);
dF5 = Controls(5);
dF6 = Controls(6);

% Capture stabilator deflections
dS = Controls(7);

% capture rudder deflections
dR = Controls(8);

% capture prop deflection angle
dt1 = Controls(9);
dt2 = Controls(10);
dt3 = Controls(11);
dt4 = Controls(12);


% Blade RPMs
CPRPMs = Controls(13:16) ;
LPRPMs = Controls(17:18) ;

% Blade Pitch
CPBladePitch = Controls(19:22);
LPBladePitch = Controls(23:24);


LiftPropCutoffRPM = 300;
PropCutoffRPM = 300;






%% AOA calculation
AOA_LocFree = AOA_dir - Vehicle.Aero.StripDefn.dth;

% AOA_LocFree

qbar_s = 0.5*rho.*(us_s.^2 + ws_s.^2);

%% Calculate incremental lift coefficient due to canardvator 
% the evaluation of the control surface preceeds the parent lifting surface
% so that if we wanted to calculate the total induced AOA, the circulation
% due to control surface goes to the EMPAERO_CalcInducedAOA

% Syntax
% [deltaCl_LS, detlaCir_LS] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.LS, [def1(deg) def2(deg) ... ], V_s(LCIndices), Chord(LCIndices), Vehicle);
% if multiple control surfaces on a lifting surface, the order of the control surface deflection angles should be in the same
% order of CSNmaes in VehicleDefiniton_Baseline

Vehicle.Aero.ControlSurfaceType = 'Plain';

[deltaCl_LWing, detlaCir_LWing] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.LWing, [dF3, dF2, 0], V_s(LWIndices), c_strip(LWIndices), Vehicle);
[deltaCl_RWing, detlaCir_RWing] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.RWing, [dF4, dF5, 0], V_s(RWIndices), c_strip(RWIndices), Vehicle);


[deltaCl_VT,     detlaCir_VT] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.VS, [dR], V_s(VTIndices), c_strip(VTIndices), Vehicle);


% Calling Induced Drag Calculation Function
symm = 0; Root = 0; Tip = 0;
[AOAi_LWing, InducedDragLWing] = CalcInducedAOA(Vehicle.Aero.LWing.ROM,AOA_LocFree(LWIndices),V_s(LWIndices),a,symm,Root,Tip, rho, detlaCir_LWing);

symm = 0; Root = 0; Tip = 0;
[AOAi_RWing, InducedDragRWing] = CalcInducedAOA(Vehicle.Aero.RWing.ROM,AOA_LocFree(RWIndices),V_s(RWIndices),a,symm,Root,Tip, rho, detlaCir_RWing);

symm = 1; Root = 0; Tip = 1;
[AOAi_LHTail, InducedDragLHTail] = CalcInducedAOA(Vehicle.Aero.LHTail.ROM,AOA_LocFree(LHTIndices),V_s(LHTIndices),a,symm,Root,Tip, rho, 0);

symm = 1; Root = 0; Tip = 1;
[AOAi_RHTail, InducedDragRHTail] = CalcInducedAOA(Vehicle.Aero.RHTail.ROM,AOA_LocFree(RHTIndices),V_s(RHTIndices),a,symm,Root,Tip, rho, 0);


symm = 0; Root = 0; Tip = 1;
[AOAi_VT, InducedDragVT] = CalcInducedAOA(Vehicle.Aero.VS.ROM,AOA_LocFree(VTIndices),V_s(VTIndices),a,symm,Root,Tip, rho, detlaCir_VT);


AOA_ind = [ AOAi_LWing;  AOAi_RWing; AOAi_LHTail; AOAi_RHTail; AOAi_VT];

flag = 1;
InducedDrag = [InducedDragLWing*flag; InducedDragRWing*flag; InducedDragLHTail*flag; InducedDragRHTail; InducedDragVT*flag];



CSType = Vehicle.Aero.ControlSurfaceType;
%%  Calculate incremental profile drag due to all control surface deflection
% choose control surface type 
% only drag data of plain and single-slotted flap available in DATCOM 
if strcmp(CSType,'Plain')
  delta_cdfpGI = Vehicle.Aero.PlainTEFlapGI.delta_cdfp;
end 

if strcmp(CSType,'SingleSlotted')
  delta_cdfpGI = Vehicle.Aero.SingleSlottedTEFlapGI.delta_cdfp;
end 

% The abs(deflection) is used to ensure
% symmetric drag increment for -ve and +ve deflection cases

% The increment in 3D profile drag coefficient is computed based on DATCOM Reader pg. 2223
% the drag is then distributed over the control surface span based on aera
% ratio 
% CDp = cdp * Kb (Kb :  Span Factor For Inboard Flaps)
% Dp = 0.5 * rho * V^2 * Sref * CDp 

Sref = 2 * Vehicle.Geom.LWing.PlanformArea;

deltaCD_p = zeros(size([LWIndices';RWIndices';LHTIndices';RHTIndices';VTIndices']));

% left wing

% OBD flaperon
% 2-D profile drag increment
% delcd_p_Flap_LOBD = delta_cdfpGI(FlaperonFCR,abs(dF1));
% % 2-D to 3-D conversion (also normalized by reference area)
% delCD_p_Flap_LOBD = delcd_p_Flap_LOBD* Vehicle.Aero.LWing.OBD_Flaperon.Kb * Vehicle.Geom.LWing.PlanformArea/Sref;
% % indices of the lifting surface on which the control surface lies
% ind = LWIndices(OBDLWFlaperon);
% % distribution of the 3-D drag value to each of the strip proportional to the strip area 
% deltaCD_p(ind) = delCD_p_Flap_LOBD * dA(ind)/ sum(dA(ind));

% MBD flaperon
% 2-D profile drag increment
delcd_p_Flap_LMBD = delta_cdfpGI(FlaperonFCR,abs(dF2));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_Flap_LMBD = delcd_p_Flap_LMBD* Vehicle.Aero.LWing.MBD_Flaperon.Kb * Vehicle.Geom.LWing.PlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
ind = LWIndices(MBDLWFlaperon);
% distribution of the 3-D drag value to each of the strip proportional to the strip area 
deltaCD_p(ind) = delCD_p_Flap_LMBD * dA(ind)/ sum(dA(ind));

% IBD flaperon
% 2-D profile drag increment
delcd_p_Flap_LIBD = delta_cdfpGI(FlaperonFCR,abs(dF3));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_Flap_LIBD = delcd_p_Flap_LIBD* Vehicle.Aero.LWing.IBD_Flaperon.Kb * Vehicle.Geom.LWing.PlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
ind = LWIndices(IBDLWFlaperon);
% distribution of the 3-D drag value to each of the strip proportional to the strip area 
deltaCD_p(ind) = delCD_p_Flap_LIBD * dA(ind)/ sum(dA(ind));


% right wing

% IBD flaperon
% 2-D profile drag increment
delcd_p_Flap_RIBD = delta_cdfpGI(FlaperonFCR,abs(dF4));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_Flap_RIBD = delcd_p_Flap_RIBD* Vehicle.Aero.RWing.IBD_Flaperon.Kb * Vehicle.Geom.RWing.PlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
ind = RWIndices(IBDRWFlaperon);
% distribution of the 3-D drag value to each of the strip proportional to the strip area 
deltaCD_p(ind) = delCD_p_Flap_RIBD * dA(ind)/ sum(dA(ind));

% MBD flaperon
% 2-D profile drag increment
delcd_p_Flap_RMBD = delta_cdfpGI(FlaperonFCR,abs(dF5));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_Flap_RMBD = delcd_p_Flap_RMBD* Vehicle.Aero.RWing.MBD_Flaperon.Kb * Vehicle.Geom.RWing.PlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
ind = RWIndices(MBDRWFlaperon);
% distribution of the 3-D drag value to each of the strip proportional to the strip area 
deltaCD_p(ind) = delCD_p_Flap_RMBD * dA(ind)/ sum(dA(ind));


% OBD flaperon
% 2-D profile drag increment
% delcd_p_Flap_ROBD = delta_cdfpGI(FlaperonFCR,abs(dF6));
% % 2-D to 3-D conversion (also normalized by reference area)
% delCD_p_Flap_ROBD = delcd_p_Flap_ROBD* Vehicle.Aero.RWing.OBD_Flaperon.Kb * Vehicle.Geom.RWing.PlanformArea/Sref;
% % indices of the lifting surface on which the control surface lies
% ind = RWIndices(OBDRWFlaperon);
% % distribution of the 3-D drag value to each of the strip proportional to the strip area 
% deltaCD_p(ind) = delCD_p_Flap_ROBD * dA(ind)/ sum(dA(ind));


% rudder
delcd_p_Rudder = delta_cdfpGI(RudderFCR,abs(dR));
delcd_p_Rudder = delcd_p_Rudder* Vehicle.Aero.VS.Rudder.Kb * Vehicle.Geom.VS.PlanformArea/Sref;
ind = VTIndices(RudIndices);
deltaCD_p(ind) = delcd_p_Rudder * dA(ind)/sum(dA(ind));



%% Query sectional aero
% Query wing sectional aero


% Change in AOA because of viscocity effects
AOA_visc(LWIndices,:)    = Vehicle.Aero.LWing.AOA_visc;
AOA_visc(RWIndices,:)    = Vehicle.Aero.RWing.AOA_visc;
AOA_visc(LHTIndices,:)    = Vehicle.Aero.LHTail.AOA_visc;
AOA_visc(RHTIndices,:)    = Vehicle.Aero.RHTail.AOA_visc;
AOA_visc(VTIndices,:)    = Vehicle.Aero.VS.AOA_visc;




% compute the effective angle of attack --> Effective AOA = Geom AOA - Induced AOA
AOA_eff = AOA_LocFree - AOA_ind + AOA_visc;

AOA_eff_LW = AOA_eff(LWIndices);
Cl_LW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_LW)*Vehicle.Aero.LWing.Cl_3DCorr;
Cd_LW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_LW)*Vehicle.Aero.LWing.Cd0_3DCorr;
Cm_LW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_LW);


AOA_eff_RW = AOA_eff(RWIndices);
Cl_RW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_RW)*Vehicle.Aero.RWing.Cl_3DCorr;
Cd_RW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_RW)*Vehicle.Aero.RWing.Cd0_3DCorr;
Cm_RW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_RW);

% Query horizontal stabilizer sectional aero
AOA_eff_LHT = AOA_eff(LHTIndices);
Cl_LHT = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_LHT)*Vehicle.Aero.LHTail.Cl_3DCorr;
Cd_LHT = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_LHT)*Vehicle.Aero.LHTail.Cd0_3DCorr;
Cm_LHT = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_LHT);

AOA_eff_RHT = AOA_eff(RHTIndices);
Cl_RHT = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_RHT)*Vehicle.Aero.RHTail.Cl_3DCorr;
Cd_RHT = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_RHT)*Vehicle.Aero.RHTail.Cd0_3DCorr;
Cm_RHT = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_RHT);

% Query Vertical stabilizer sectional aero
AOA_eff_VT = AOA_eff(VTIndices);
Cl_VT = Vehicle.Aero.GI_Cl_AOA_VT(AOA_eff_VT)*Vehicle.Aero.VS.Cl_3DCorr;
Cd_VT = Vehicle.Aero.GI_Cd_AOA_VT(AOA_eff_VT)*Vehicle.Aero.VS.Cd0_3DCorr;
Cm_VT = Vehicle.Aero.GI_Cm_AOA_VT(AOA_eff_VT);




Cl = [Cl_LW*flag; Cl_RW*flag; Cl_LHT*flag; Cl_RHT; Cl_VT*flag; ];
Cd = [Cd_LW*flag; Cd_RW*flag; Cd_LHT*flag; Cd_RHT; Cd_VT*flag; ];
Cm = [Cm_LW*flag; Cm_RW*flag; Cm_LHT*flag; Cm_RHT; Cm_VT*flag; ];

% Additional of incremental lift due to control surface
Cl = Cl + [deltaCl_LWing; deltaCl_RWing; zeros(length(LHTIndices),1); zeros(length(RHTIndices),1); deltaCl_VT];

%% Compute strip lift, drag, and pitching moment loads

dA = Vehicle.Aero.StripDefn.Area;      % m2
c_strip = Vehicle.Aero.StripDefn.Chord; % m



% % 
dL_s                = qbar_s .* dA .* Cl ;

dD_s_profile        = qbar_s .* dA .* Cd .* Vehicle.Aero.KCD0;
dD_s_Profile_CS     = qbar_s.* Sref .* deltaCD_p;
dD_s                = (dD_s_profile + dD_s_Profile_CS)  +  InducedDrag;

dM_s                = qbar_s .* dA .* c_strip .* Cm;               



%%

% convert these loads back from local wind axes to ROM axis system

dFx_r =  dL_s.*sind(AOA_LocFree) - dD_s.*cosd(AOA_LocFree);
dFy_r =  0;
dFz_r = -dL_s.*cosd(AOA_LocFree) - dD_s.*sind(AOA_LocFree); 

dMx_r = 0;
dMy_r = dM_s;
dMz_r = 0;


% AOA_LocFree
% 
% % convert these loads from ROM basis [R] to body-fixed basis [B] using
% the Phi Rotation
% note: loads act at quarter chord points of strips
%%

dFx_AC_b =  R_BR_11.*dFx_r + R_BR_12.*dFy_r  + R_BR_13.*dFz_r;
dFy_AC_b =  R_BR_21.*dFx_r + R_BR_22.*dFy_r  + R_BR_23.*dFz_r;
dFz_AC_b =  R_BR_31.*dFx_r + R_BR_32.*dFy_r  + R_BR_33.*dFz_r;

dMx_AC_b =  R_BR_11.*dMx_r + R_BR_12.*dMy_r  + R_BR_13.*dMz_r;
dMy_AC_b =  R_BR_21.*dMx_r + R_BR_22.*dMy_r  + R_BR_23.*dMz_r;
dMz_AC_b =  R_BR_31.*dMx_r + R_BR_32.*dMy_r  + R_BR_33.*dMz_r;


end
