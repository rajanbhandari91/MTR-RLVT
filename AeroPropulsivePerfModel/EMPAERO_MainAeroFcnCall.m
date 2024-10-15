function [dFx_AC_b,dFy_AC_b,dFz_AC_b,dMx_AC_b,dMy_AC_b,dMz_AC_b,AOA_eff,ppData] = EMPAERO_MainAeroFcnCall(rho,Vehicle,V_s,AOA_dir,Controls,R_BS)

% Extract control surface type
CSType = Vehicle.Aero.ControlSurfaceType;                                   % Control Surface type: Plain or single slotted

% Downwash check boxes 
ActivateWingDownwash = 1; 
ActivateIndAOA = 1;

% capture reference area to normalize drag coefficients 
Sref = Vehicle.Aero.RefArea;

%%%% STRIP THEORY IMPLEMENTATION
% STRIP AFFILIATIONS
% CONTROL SURFACE LOCATIONS

%% CAPTURE STRIP INDICES and PROPERTIES

% Left wing, root to tip
LWIndices = Vehicle.Aero.Indices.LWing;
% Right wing, root to tip
RWIndices = Vehicle.Aero.Indices.RWing;
% Left horizontal stab, root to tip
LHTIndices = Vehicle.Aero.Indices.LHTail;
% Right horizontal stab, root to tip
RHTIndices = Vehicle.Aero.Indices.RHTail;
% Vertical stab, root to tip
VTIndices = Vehicle.Aero.Indices.VTail;


%% Addition to evaluate a single lifting surface
% isolate a lfiting surface for other surfaces for analysis

% makes a vector with entries equal to 1 corresponding to desired
% lifting surface and 0 for all other
% removes the need for manually 'zeroing out' the coefficients
% starts here
All_indices = [LWIndices';RWIndices';LHTIndices';RHTIndices';VTIndices'];
if ~isempty(Vehicle.Aero.DesSurf)
    DesSurf = Vehicle.Aero.DesSurf;
    zeroOutVec = zeros(size(All_indices));
    ActivateWingDownwash = 0;

    if strcmp(DesSurf,'LWing')
        zeroOutVec(LWIndices)=1;
    end

    if strcmp(DesSurf,'LHTail')
        zeroOutVec(LHTIndices)=1;
    end
else
    zeroOutVec = ones(size(All_indices));
end
% ends here

%% CONTROL SURFACE LOCATIONS

IBDLWFlaperon = Vehicle.Aero.Indices.Inboard_Flaperon_on_LWing;
OBDLWFlaperon = Vehicle.Aero.Indices.Outboard_Flaperon_on_LWing;
MBDLWFlaperon = Vehicle.Aero.Indices.Midboard_Flaperon_on_LWing;
IBDRWFlaperon = Vehicle.Aero.Indices.Inboard_Flaperon_on_RWing;
OBDRWFlaperon = Vehicle.Aero.Indices.Outboard_Flaperon_on_RWing;
MBDRWFlaperon = Vehicle.Aero.Indices.Midboard_Flaperon_on_RWing;
LeftElev = Vehicle.Aero.Indices.Elevator_on_LHTail;
RightElev = Vehicle.Aero.Indices.Elevator_on_RHTail;
RudIndices = Vehicle.Aero.Indices.Rudder_on_VTail;


% Flap-to-chord ratio
IBDFlaperonFCR = Vehicle.Aero.ControlChordFracs.Inboard_Flaperon_on_LWing;
OBDFlaperonFCR = Vehicle.Aero.ControlChordFracs.Outboard_Flaperon_on_LWing;
MBDFlaperonFCR = Vehicle.Aero.ControlChordFracs.Midboard_Flaperon_on_LWing;

RudderFCR = Vehicle.Aero.ControlChordFracs.Rudder_on_VTail;
ElevatorFCR = Vehicle.Aero.ControlChordFracs.Elevator_on_LHTail;

% Extract area from StripDef 
dA = Vehicle.Aero.StripDef.Area;

% R_BS = Vehicle.Aero.StripDefn.R_BS;
[R_BS_11, R_BS_12, R_BS_13, R_BS_21, R_BS_22, R_BS_23, R_BS_31, R_BS_32, R_BS_33]...
    = deal(R_BS(:,1),R_BS(:,2),R_BS(:,3),R_BS(:,4),R_BS(:,5),R_BS(:,6),R_BS(:,7),R_BS(:,8),R_BS(:,9));


% Computing velocities in strip frame from the total velocity and AOA_dir
ws_s = V_s.*sind(AOA_dir);
us_s = V_s.*cosd(AOA_dir);


%% CAPTURE Control vector

% 1. Left OBD + MBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD + MBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Rudder deflection, deg, positive TER

% 8. Cruise prop throttle, [0-1]
% 9. Cruise prop pitch, deg
% 10. Cruise prop RPM

% 11. Lift prop 1 RPM
% 12. Lift prop 2 RPM
% 13. Lift prop 3 RPM
% 14. Lift prop 4 RPM
% 15. Lift prop 5 RPM
% 16. Lift prop 6 RPM
% 17. Lift prop 7 RPM
% 18. Lift prop 8 RPM

% 19. Lift prop 1 pitch
% 20. Lift prop 2 pitch
% 21. Lift prop 3 pitch
% 22. Lift prop 4 pitch
% 23. Lift prop 5 pitch
% 24. Lift prop 6 pitch
% 25. Lift prop 7 pitch
% 26. Lift prop 8 pitch

% capture flaperon deflections
dF1 = Controls(1);
dF2 = Controls(2);
dF3 = Controls(3);
dF4 = Controls(4);
dF5 = Controls(5);
dF6 = Controls(6);

% Capture elevator deflections
dE1 = Controls(7);
dE2 = Controls(8);

% capture rudder deflections
dR = Controls(9);


%% AOA calculation
AOA_LocFree = AOA_dir;

% AOA_LocFree

qbar_s = 0.5*rho.*(us_s.^2 + ws_s.^2);

Chord = Vehicle.Aero.StripDef.Chord;

%% Downwash angle initialization 
% epsilon will collect downwash angle seen by the strips due to relevant
% forward lifting surfaces 
epsilon = zeros(size(All_indices));

%% Calculate incremental lift coefficient  
% the evaluation of the control surface preceeds the parent lifting surface
% so that if we wanted to calculate the total induced AOA, the circulation
% due to control surface goes to the EMPAERO_CalcInducedAOA

% Syntax
% [deltaCl_LS, detlaCir_LS] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.LS, [def1(deg) def2(deg) ... ], V_s(LCIndices), Chord(LCIndices), Vehicle);
% if multiple control surfaces on a lifting surface, the order of the control surface deflection angles should be in the same
% order of CSNames in Vehicle.Geom.(LSName).Controls.Name
%% WING 
% Order: [IBD, MBD, BBD]
dF_LWing = [dF3 dF2 dF1];
dF_RWing = [dF4 dF5 dF6];
%% Calculate incremental lift coefficient due to flaperon
[deltaCl_LWing, detlaCir_LWing] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.LWing, dF_LWing, V_s(LWIndices), Chord(LWIndices), Vehicle);
[deltaCl_RWing, detlaCir_RWing] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.RWing, dF_RWing, V_s(RWIndices), Chord(RWIndices), Vehicle);

%% Calculate indcued AOA for wing and take additional outputs for analysis
symm = 0; 
[AOAi_LWing,ADDL_LWing] = EMPAERO_CalcInducedAOA(Vehicle.Aero.LWing,AOA_LocFree(LWIndices),V_s(LWIndices),symm, rho,detlaCir_LWing);

symm = 0; 
[AOAi_RWing,ADDL_RWing] = EMPAERO_CalcInducedAOA(Vehicle.Aero.RWing,AOA_LocFree(RWIndices),V_s(RWIndices),symm, rho,detlaCir_RWing);

%% Compute downwash due to wing on horizontal tail 
% the downwash angle is zero at wing's airfoil's zero-lift-AOA and
% symmetric about it
% Left Wing
WingRootAOA = AOA_LocFree(LWIndices(1));
GI_epsilon_LWing2LHTail = Vehicle.Aero.LWing.downwash.GI_epsilon_raymer;
epsilon_on_LHTail = GI_epsilon_LWing2LHTail(WingRootAOA);

% Right Wing
GI_epsilon_RWing2RHTail = Vehicle.Aero.RWing.downwash.GI_epsilon_raymer;
epsilon_on_RHTail = GI_epsilon_RWing2RHTail(WingRootAOA);

% store the value of downwash angle at horizontal tail strips 
epsilon(LHTIndices) = ActivateWingDownwash * epsilon_on_LHTail;
epsilon(RHTIndices) = ActivateWingDownwash * epsilon_on_RHTail;

%% Compute AOA as seen by the horizontal tail strips on wing downwash 
AOA_LocFree(LHTIndices) = AOA_LocFree(LHTIndices) - epsilon(LHTIndices);
AOA_LocFree(RHTIndices) = AOA_LocFree(RHTIndices) - epsilon(RHTIndices);

%% Calculate incremental lift coefficient due to elevator 
[deltaCl_LHTail, detlaCir_LHTail] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.LHTail, dE1, V_s(LHTIndices), Chord(LHTIndices), Vehicle);
[deltaCl_RHTail, detlaCir_RHTail] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.RHTail, dE2, V_s(RHTIndices), Chord(RHTIndices), Vehicle);

%% Calculate indcued AOA for horizontal tail and take additional outputs for analysis
symm = 1; 
[AOAi_LHTail,ADDL_LHTail] = EMPAERO_CalcInducedAOA(Vehicle.Aero.LHTail,AOA_LocFree(LHTIndices),V_s(LHTIndices),symm, rho,detlaCir_LHTail);

symm = 1; 
[AOAi_RHTail,ADDL_RHTail] = EMPAERO_CalcInducedAOA(Vehicle.Aero.RHTail,AOA_LocFree(RHTIndices),V_s(RHTIndices),symm, rho,detlaCir_RHTail);

%% Calculate incremental lift coefficient due to rudder
%% Calculate incremental lift coefficient due to rudder
[deltaCl_VTail, detlaCir_VTail] = EMPAERO_CalcDeltaClCS(Vehicle.Aero.VTail, dR, V_s(VTIndices), Chord(VTIndices), Vehicle);

%% Calculating the indcued AOA and additional outputs for vertical tail
symm = 0; 
[AOAi_VTail, ADDL_VTail] = EMPAERO_CalcInducedAOA(Vehicle.Aero.VTail,AOA_LocFree(VTIndices),V_s(VTIndices),symm, rho,detlaCir_VTail);

% warning('effect of CS on induced AOA activated')
AOA_ind = [ AOAi_LWing;  AOAi_RWing; AOAi_LHTail; AOAi_RHTail; AOAi_VTail];
AOA_ind_TOT = [ADDL_LWing.indAOA_Tot; ADDL_RWing.indAOA_Tot; ADDL_LHTail.indAOA_Tot; ADDL_RHTail.indAOA_Tot; ADDL_VTail.indAOA_Tot];
AOA_ind_op = AOA_ind;

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


deltaCD_p = zeros(size(All_indices));
deltaCD_p_test = deltaCD_p;

% For compatible LSName Vehicle.Aero.Indices
% For compatible CSName Vehicle.Geom.(LSName).Controls.Name

% left wing
% IBD Flap 
LSName = 'LWing'; CSName = 'Inboard_Flaperon'; LSIndices = LWIndices; CSIndices = IBDLWFlaperon; CSChordRatio = IBDFlaperonFCR; def = dF1;
deltaCD_p_IBDLWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_IBDLWFlaperon;

% MBD Flap 
LSName = 'LWing'; CSName = 'Midboard_Flaperon'; LSIndices = LWIndices; CSIndices = MBDLWFlaperon; CSChordRatio = MBDFlaperonFCR; def = dF2;
deltaCD_p_MBDLWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_MBDLWFlaperon;

% OBD Flap 
LSName = 'LWing'; CSName = 'Outboard_Flaperon'; LSIndices = LWIndices; CSIndices = OBDLWFlaperon; CSChordRatio = OBDFlaperonFCR; def = dF2;
deltaCD_p_OBDLWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_OBDLWFlaperon;

% right wing
% IBD Flap 
LSName = 'RWing'; CSName = 'Inboard_Flaperon'; LSIndices = RWIndices; CSIndices = IBDRWFlaperon; CSChordRatio = IBDFlaperonFCR; def = dF1;
deltaCD_p_IBDRWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_IBDRWFlaperon;

% MBD Flap 
LSName = 'RWing'; CSName = 'Midboard_Flaperon'; LSIndices = RWIndices; CSIndices = MBDRWFlaperon; CSChordRatio = MBDFlaperonFCR; def = dF2;
deltaCD_p_MBDRWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_MBDRWFlaperon;

% OBD Flap 
LSName = 'RWing'; CSName = 'Outboard_Flaperon'; LSIndices = RWIndices; CSIndices = OBDRWFlaperon; CSChordRatio = OBDFlaperonFCR; def = dF2;
deltaCD_p_OBDRWFlaperon = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_OBDRWFlaperon;

% Left H. Tail 
% Elevator 
LSName = 'LHTail'; CSName = 'Elevator'; LSIndices = LHTIndices; CSIndices = LeftElev; CSChordRatio = ElevatorFCR; def = dE1;
deltaCD_p_LHTailElev = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_LHTailElev;

% Right H. Tail 
% Elevator 
LSName = 'RHTail'; CSName = 'Elevator'; LSIndices = RHTIndices; CSIndices = RightElev; CSChordRatio = ElevatorFCR; def = dE1;
deltaCD_p_RHTailElev = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_RHTailElev;

% V. Tail
% Rudder 
LSName = 'VTail'; CSName = 'Rudder'; LSIndices = VTIndices; CSIndices = RudIndices; CSChordRatio = RudderFCR; def = dR;
deltaCD_p_Rudder = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle);
index = LSIndices(CSIndices);
deltaCD_p(index) = deltaCD_p_Rudder;


%% Query sectional aero
% compute the effective angle of attack --> Effective AOA = Freestream AOA- downwash - Induced AOA
% downwash already subtracted before sending for induced AOA calculation
AOA_eff = AOA_LocFree - ActivateIndAOA * AOA_ind_op;

% Left Wing
AOA_eff_LW = AOA_eff(LWIndices);
Cl_LW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_LW);
Cd_LW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_LW);
Cm_LW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_LW);

% Right Wing
AOA_eff_RW = AOA_eff(RWIndices);
Cl_RW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_RW);
Cd_RW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_RW);
Cm_RW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_RW);

% LHTail
AOA_eff_LHTail = AOA_eff(LHTIndices);
Cl_LHTail = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_LHTail);
Cd_LHTail = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_LHTail);
Cm_LHTail = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_LHTail);

% RHTail
AOA_eff_RHTail = AOA_eff(RHTIndices);
Cl_RHTail = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_RHTail);
Cd_RHTail = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_RHTail);
Cm_RHTail = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_RHTail);

% Query Vertical stabilizer sectional aero
AOA_eff_VTail = AOA_eff(VTIndices);
Cl_VTail = Vehicle.Aero.GI_Cl_AOA_VT(AOA_eff_VTail);
Cd_VTail = Vehicle.Aero.GI_Cd_AOA_VT(AOA_eff_VTail);
Cm_VTail = Vehicle.Aero.GI_Cm_AOA_VT(AOA_eff_VTail);

Cl = [Cl_LW; Cl_RW; Cl_LHTail; Cl_RHTail; Cl_VTail];
Cd = [Cd_LW; Cd_RW; Cd_LHTail; Cd_RHTail; Cd_VTail];
Cm = [Cm_LW; Cm_RW; Cm_LHTail; Cm_RHTail; Cm_VTail];

% zero out all other values if a lifitng surface is specified for testing 
Cl = Cl.*zeroOutVec;
Cd = Cd.*zeroOutVec;
Cm = Cm.*zeroOutVec;

% incremental lift due to control surface
deltaCL = [deltaCl_LWing; deltaCl_RWing; deltaCl_LHTail; deltaCl_RHTail; deltaCl_VTail];
deltaCL = deltaCL.*zeroOutVec;
% incremental profile drag due to control surface
deltaCD_p = deltaCD_p.*zeroOutVec;
%% Compute strip lift, drag, and pitching moment loads

c_strip = Vehicle.Aero.StripDefn.Chord; % m

% Cl, Cd queried are respectively perpendicular and parallel to the local wind axes
% ( along the freestream bent by the indcued downwash if present)
% no drag is indcued along the local wind axis...which means cdi should not
% be added to the queried profile drag cd

% local wind axes
dL_s = qbar_s .* dA .* Cl ;
delta_dL_s = qbar_s .* dA .* deltaCL;
dL_s_TOT = dL_s + delta_dL_s;


dD_s_profile = qbar_s .* dA .* Cd;
delta_dD_s_profile = qbar_s.* Sref .* deltaCD_p;                               % dD_s_Profile_CS(N) is 3D drag due to control surface deflection distributed over the span of control surface

dD_s_TOT = dD_s_profile + delta_dD_s_profile;                                      
dM_s = qbar_s .* dA .* c_strip .* Cm;                                                                   

% dM_s(LWIndices)

%% Transformation: local wind axes to strip axis system

% convert these loads back from local wind axes to strip axis system
% AOA_LocFree replaced by AOA_eff
dFx_s = dL_s_TOT.*sind(AOA_eff) - dD_s_TOT.*cosd(AOA_eff); 
dFy_s =  0;
dFz_s = -dL_s_TOT.*cosd(AOA_eff) - dD_s_TOT.*sind(AOA_eff);

dMx_s = 0;
dMy_s = dM_s;
dMz_s = 0;



% % convert these loads from strip bais [S] to body-fixed basis [B] 

dFx_AC_b =  R_BS_11.*dFx_s + R_BS_12.*dFy_s  + R_BS_13.*dFz_s;
dFy_AC_b =  R_BS_21.*dFx_s + R_BS_22.*dFy_s  + R_BS_23.*dFz_s;
dFz_AC_b =  R_BS_31.*dFx_s + R_BS_32.*dFy_s  + R_BS_33.*dFz_s;

dMx_AC_b =  R_BS_11.*dMx_s + R_BS_12.*dMy_s  + R_BS_13.*dMz_s;
dMy_AC_b =  R_BS_21.*dMx_s + R_BS_22.*dMy_s  + R_BS_23.*dMz_s;
dMz_AC_b =  R_BS_31.*dMx_s + R_BS_32.*dMy_s  + R_BS_33.*dMz_s;

%% post-processing

% deviation of the local wind axis from freestream
totDeviation = ActivateIndAOA * AOA_ind_op + ActivateWingDownwash * epsilon;
% induced drag
inducedDrag = dL_s_TOT .* sind(totDeviation);
% very small value of negative induced drag is seen at angle of attack 
% close to the zero-lift-AOA 
% most likely due to resolution of the induced angle calculation process 
inducedDrag(inducedDrag<=0) = 0;
ppData.dD_s_i = inducedDrag;
% profile drag 
ppData.dD_s_p = dD_s_TOT .* cosd(totDeviation);



Cir.Gamma = [ADDL_LWing.Gamma; ADDL_RWing.Gamma; ADDL_LHTail.Gamma; ADDL_RHTail.Gamma; ADDL_VTail.Gamma];
Cir.dCdA = [ADDL_LWing.dCdA; ADDL_RWing.dCdA; ADDL_LHTail.dCdA; ADDL_LHTail.dCdA; ADDL_VTail.dCdA];
Cir.c0 =[ADDL_LWing.c0; ADDL_RWing.c0; ADDL_LHTail.c0; ADDL_LHTail.c0; ADDL_VTail.c0];

ppData.Cir = Cir;
ppData.dA = dA;
ppData.indAOA = AOA_ind;

ppData.Cl = Cl;
ppData.Cd = Cd;

AOA_For_Rotation = AOA_ind;
% totDeviation: from local wind axis to global freestream direction
% indAOA: from local wind axis to local freestream direction, control surface effect on indAOA turned OFF 
% indAOA_TOT: from local wind axis to local freestream direction, control surface effect on indAOA turned ON

dFx_w = -dL_s .* sind(AOA_For_Rotation)  - dD_s_profile .* cosd(AOA_For_Rotation);
dFz_w = -dL_s .* cosd(AOA_For_Rotation) + dD_s_profile .* sind(AOA_For_Rotation);

dDi = dL_s .* sind(AOA_For_Rotation);
dDp =  dD_s_profile .* cosd(AOA_For_Rotation);

dFx_w_CS = -delta_dL_s .* sind(AOA_For_Rotation)  - delta_dD_s_profile .* cosd(AOA_For_Rotation);
dFz_w_CS = -delta_dL_s .* cosd(AOA_For_Rotation) + delta_dD_s_profile .* sind(AOA_For_Rotation);
delta_dDi = delta_dL_s .* sind(AOA_For_Rotation);
delta_dDp =  delta_dD_s_profile .* cosd(AOA_For_Rotation);

ppData.dL_w = -dFz_w - dFz_w_CS;
ppData.dD_w = -dFx_w - dFx_w_CS;
ppData.dM_w = dM_s;


% stopper = 1;
% % lift and drag of the lifting surfaces in freestream wind axis 
% dL = -dFz_w;
% dD = -dFx_w;
% 
% delta_dL = -dFz_w_CS;
% delta_dD = - dFx_w_CS;


% InducedDrag_LS = [ADDL_LWing.InducedDrag_LS; ADDL_RWing.InducedDrag_LS; ADDL_LCan.InducedDrag_LS; ADDL_RCan.InducedDrag_LS; ADDL_VS.InducedDrag_LS];
% InducedDrag_CS = [ADDL_LWing.InducedDrag_CS; ADDL_RWing.InducedDrag_CS; ADDL_LCan.InducedDrag_CS; ADDL_RCan.InducedDrag_CS; ADDL_VS.InducedDrag_CS];
% 
% LWingForces = [sum(dL(LWIndices)); sum(dD(LWIndices)); sum(dDp(LWIndices)); sum(dDi(LWIndices)); sum(InducedDrag_LS(LWIndices))];
% LWingFlaperonForces = [sum(delta_dL(LWIndices)); sum(delta_dDi(LWIndices)); sum(delta_dDp(LWIndices)); sum(delta_dDp(LWIndices)); sum(InducedDrag_CS(LWIndices))];
% LoadTable.LWing = [LWingForces];
% LoadTable.LFlaperon = [LWingFlaperonForces];
% 
% RWingForces = [sum(dL(RWIndices)); sum(dD(RWIndices)); sum(dDp(RWIndices)); sum(dDi(RWIndices)); sum(InducedDrag_LS(RWIndices)) ];
% RWingFlaperonForces = [sum(delta_dL(RWIndices)); sum(delta_dD(RWIndices)); sum(delta_dDp(RWIndices)); sum(delta_dDi(RWIndices)); sum(InducedDrag_CS(RWIndices))];
% LoadTable.RWing = [RWingForces];
% LoadTable.RFlaperon = [RWingFlaperonForces];
% 
% LCanForces = [sum(dL(LCIndices)); sum(dD(LCIndices)); sum(dDp(LCIndices)); sum(dDi(LCIndices)); sum(InducedDrag_LS(LCIndices))];
% LCanardvatorForces = [sum(delta_dL(LCIndices)); sum(delta_dD(LCIndices));  sum(delta_dDp(LCIndices));  sum(delta_dDi(LCIndices)); sum(InducedDrag_CS(LCIndices))];
% LoadTable.LCan = [LCanForces];
% LoadTable.LCanardvator = [LCanardvatorForces];
% 
% 
% RCanForces = [sum(dL(RCIndices)); sum(dD(RCIndices)); sum(dDp(RCIndices)); sum(dDi(RCIndices)); sum(InducedDrag_LS(RCIndices))];
% RCanardvatorForces = [sum(delta_dL(RCIndices)); sum(delta_dD(RCIndices)); sum(delta_dDp(RCIndices)); sum(delta_dDi(RCIndices)); sum(InducedDrag_CS(RCIndices))];
% LoadTable.RCan = [RCanForces];
% LoadTable.RCanardvator = [RCanardvatorForces];
% 
% VSForces = [sum(dL(VTIndices)); sum(dD(VTIndices)); sum(dDp(VTIndices)); sum(dDi(VTIndices)); sum(InducedDrag_LS(VTIndices))];
% RudderForces= [sum(delta_dL(VTIndices)); sum(delta_dD(VTIndices)); sum(delta_dDp(VTIndices)); -sum(delta_dDi(VTIndices)); sum(InducedDrag_CS(VTIndices))];
% LoadTable.VS = [VSForces];
% LoadTable.Rudder = [RudderForces];
% LoadTable.TOT = [sum(dL+delta_dL); sum(dD+delta_dD); sum(dDp+delta_dDp); sum(dDi+delta_dDi); sum(InducedDrag_LS + InducedDrag_CS)] ;
% LoadTable = struct2table(LoadTable);
% LoadTable.Properties.RowNames = {'Lift(N)','Drag(N)', 'Profile Drag(N)', 'Induced Drag(N)', 'Induced Drag-Direct (N)'}
end
