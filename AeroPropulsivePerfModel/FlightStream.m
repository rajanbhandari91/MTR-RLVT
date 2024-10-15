function [dFx_AC_b,dFy_AC_b,dFz_AC_b,dMx_AC_b,dMy_AC_b,dMz_AC_b,AOA_eff] = FlightStream(a,rho,Vehicle,V_s,AOA_dir,Controls,th_ROM)


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
LVTIndices = Vehicle.Aero.Indices.Left_VS;
% Right Vertical stab, root to tip
RVTIndices = Vehicle.Aero.Indices.Right_VS;

% CONTROL SURFACE LOCATIONS
IBDLWFlaperon = Vehicle.Aero.Indices.IBD_Flaperon_on_LWing;
OBDLWFlaperon = Vehicle.Aero.Indices.OBD_Flaperon_on_LWing;
IBDRWFlaperon = Vehicle.Aero.Indices.IBD_Flaperon_on_RWing;
OBDRWFlaperon = Vehicle.Aero.Indices.OBD_Flaperon_on_RWing;
LRudIndices = Vehicle.Aero.Indices.Rudder_on_Left_VS;
RRudIndices = Vehicle.Aero.Indices.Rudder_on_Right_VS;
LeftElev = Vehicle.Aero.Indices.Elevator_on_LHTail;
RightElev = Vehicle.Aero.Indices.Elevator_on_RHTail;


FlaperonFCR = Vehicle.Aero.ControlChordFracs.IBD_Flaperon_on_RWing;
RudderFCR = Vehicle.Aero.ControlChordFracs.Rudder_on_Right_VS;
ElevatorFCR = Vehicle.Aero.ControlChordFracs.Elevator_on_LHTail;



% % % Phi Rotations of the Components for ROM data collection 
% PhiROM(LCanIndices,:) = 0;
% PhiROM(RCanIndices,:) = 0;
PhiROM(LWIndices,:) = 0;
PhiROM(RWIndices,:) = 0;
PhiROM(LHTIndices,:) = 0;
PhiROM(RHTIndices,:) = 0;
PhiROM(LVTIndices,:) = -90;
PhiROM(RVTIndices,:) = -90;


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




% LPC-03 Phoenix (01/18/2022)

% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Left rudder deflection, deg, positive TER
% 8. Right rudder deflection, deg, positive TER

% 9. Cruise prop throttle, [0-1]
% 10. Cruise prop pitch, deg
% 11. Cruise prop RPM

% 12. Lift prop 1 RPM
% 13. Lift prop 2 RPM
% 14. Lift prop 3 RPM
% 15. Lift prop 4 RPM
% 16. Lift prop 5 RPM
% 17. Lift prop 6 RPM
% 18. Lift prop 7 RPM
% 19. Lift prop 8 RPM

% 20. Lift prop 1 pitch
% 21. Lift prop 2 pitch
% 22. Lift prop 3 pitch
% 23. Lift prop 4 pitch
% 24. Lift prop 5 pitch
% 25. Lift prop 6 pitch
% 26. Lift prop 7 pitch
% 27. Lift prop 8 pitch

% capture flaperon deflections
dF1 = Controls(1);
dF2 = Controls(2);
dF3 = Controls(3);
dF4 = Controls(4);

% Capture eevator deflections
dE1 = Controls(5);
dE2 = Controls(6);

% capture rudder deflections
dR1 = Controls(7);
dR2 = Controls(8);

% Cruise Props
dt = Controls(9);
CPBladePitch = Controls(10);
CPRPM = Controls(11);

% capture RPMs
LiftPropRPMs = Controls(12:19);
% capture prop blade pitch
LPBladePitch = Controls(20:27);






%% AOA calculation
AOA_LocFree = AOA_dir - Vehicle.Aero.StripDefn.dth;

% AOA_LocFree

qbar_s = 0.5*rho.*(us_s.^2 + ws_s.^2);

% Calling Induced Drag Calculation Function
symm = 0; Root = 0; Tip = 1;
[AOAi_LWing,InducedDragLWing] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor,Vehicle.Aero.LWing.ROM,AOA_LocFree(LWIndices),V_s(LWIndices),a,symm,Root,Tip, rho);

symm = 0; Root = 0; Tip = 1;
[AOAi_RWing,InducedDragRWing] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor,Vehicle.Aero.RWing.ROM,AOA_LocFree(RWIndices),V_s(RWIndices),a,symm,Root,Tip, rho);

symm = 1; Root = 0; Tip = 1;
[AOAi_LHT,InducedDragLHT] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor,Vehicle.Aero.LHTail.ROM,AOA_LocFree(LHTIndices),V_s(LHTIndices),a,symm,Root,Tip, rho);

symm = 1; Root = 0; Tip = 1;
[AOAi_RHT,InducedDragRHT] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor,Vehicle.Aero.RHTail.ROM,AOA_LocFree(RHTIndices),V_s(RHTIndices),a,symm,Root,Tip, rho);


symm = 0; Root = 0; Tip = 1;
[AOAi_LVT,InducedDragLVT] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor,Vehicle.Aero.Left_VS.ROM,AOA_LocFree(LVTIndices),V_s(LVTIndices),a,symm,Root,Tip, rho);

symm = 0; Root = 0; Tip = 1;
[AOAi_RVT,InducedDragRVT] = CalcInducedAOA(Vehicle.Aero.LWing.AreaScaleFactor, Vehicle.Aero.Right_VS.ROM,AOA_LocFree(RVTIndices),V_s(RVTIndices),a,symm,Root,Tip, rho);


AOA_ind = [ AOAi_LWing;  AOAi_RWing; AOAi_LHT; AOAi_RHT; AOAi_LVT; AOAi_RVT];

% InducedDrag = [0*InducedDragLWing; 0*InducedDragRWing; InducedDragLHT; InducedDragRHT; 0*InducedDragLVT; 0*InducedDragRVT];
InducedDrag = Vehicle.Aero.KCDi*[InducedDragLWing; InducedDragRWing; InducedDragLHT; InducedDragRHT; InducedDragLVT; InducedDragRVT];


% Change in AOA because of viscocity effects
AOA_visc(LWIndices,:)    = Vehicle.Aero.LWing.AOA_visc;
AOA_visc(RWIndices,:)    = Vehicle.Aero.RWing.AOA_visc;
AOA_visc(LHTIndices,:)    = Vehicle.Aero.LHTail.AOA_visc;
AOA_visc(RHTIndices,:)    = Vehicle.Aero.RHTail.AOA_visc;
AOA_visc(LVTIndices,:)    = Vehicle.Aero.Left_VS.AOA_visc;
AOA_visc(RVTIndices,:)    = Vehicle.Aero.Right_VS.AOA_visc;




% compute the effective angle of attack --> Effective AOA = Geom AOA - Induced AOA
AOA_eff = AOA_LocFree - AOA_ind + AOA_visc;




% For some small deflections, the linear interpolation gives negative drag
% increments, so are ignored. The abs(deflection) is used to ensure
% symmetric drag increment for -ve and +ve deflection cases

delCD_p_Flaperon_L = 0*Vehicle.Aero.CDp0_GI(FlaperonFCR,abs(dF1));
delCD_p_Flaperon_L(delCD_p_Flaperon_L<0) = 0;

delCD_p_Flaperon_R = 0*Vehicle.Aero.CDp0_GI(FlaperonFCR,abs(dF2));
delCD_p_Flaperon_R(delCD_p_Flaperon_R<0) = 0;

delCD_p_Elev_L = 0*Vehicle.Aero.CDp0_GI(ElevatorFCR,abs(dE1));
delCD_p_Elev_L(delCD_p_Elev_L<0) = 0;

delCD_p_Elev_R = 0*Vehicle.Aero.CDp0_GI(ElevatorFCR,abs(dE2));
delCD_p_Elev_R(delCD_p_Elev_R<0) = 0;

delCD_p_Rudder_L = 0*Vehicle.Aero.CDp0_GI(RudderFCR,abs(dR1));
delCD_p_Rudder_L(delCD_p_Rudder_L<0) = 0;

delCD_p_Rudder_R = 0*Vehicle.Aero.CDp0_GI(RudderFCR,abs(dR2));
delCD_p_Rudder_R(delCD_p_Rudder_R<0) = 0;



%% Query sectional aero
% Query wing sectional aero
LWFlaperon = [IBDLWFlaperon OBDLWFlaperon];
AOA_eff_LW = AOA_eff(LWIndices);
Cl_LW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_LW)*Vehicle.Aero.LWing.Cl_3DCorr;
Cd_LW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_LW)*Vehicle.Aero.LWing.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_LW(LWFlaperon) = Cd_LW(LWFlaperon) + delCD_p_Flaperon_L;
Cm_LW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_LW);

RWFlaperon = [IBDRWFlaperon OBDRWFlaperon];
AOA_eff_RW = AOA_eff(RWIndices);
Cl_RW = Vehicle.Aero.GI_Cl_AOA_wing(AOA_eff_RW)*Vehicle.Aero.RWing.Cl_3DCorr;
Cd_RW = Vehicle.Aero.GI_Cd_AOA_wing(AOA_eff_RW)*Vehicle.Aero.RWing.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_RW(RWFlaperon) = Cd_RW(RWFlaperon) + delCD_p_Flaperon_R;
Cm_RW = Vehicle.Aero.GI_Cm_AOA_wing(AOA_eff_RW);

% Query horizontal stabilizer sectional aero
AOA_eff_LHT = AOA_eff(LHTIndices);
Cl_LHT = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_LHT)*Vehicle.Aero.LHTail.Cl_3DCorr;
Cd_LHT = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_LHT)*Vehicle.Aero.LHTail.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_LHT(LeftElev) = Cd_LHT(LeftElev) + delCD_p_Elev_L;
Cm_LHT = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_LHT);

AOA_eff_RHT = AOA_eff(RHTIndices);
Cl_RHT = Vehicle.Aero.GI_Cl_AOA_HT(AOA_eff_RHT)*Vehicle.Aero.RHTail.Cl_3DCorr;
Cd_RHT = Vehicle.Aero.GI_Cd_AOA_HT(AOA_eff_RHT)*Vehicle.Aero.RHTail.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_RHT(RightElev) = Cd_RHT(RightElev) + delCD_p_Elev_R;
Cm_RHT = Vehicle.Aero.GI_Cm_AOA_HT(AOA_eff_RHT);

% Query Vertical stabilizer sectional aero
AOA_eff_LVT = AOA_eff(LVTIndices);
Cl_LVT = Vehicle.Aero.GI_Cl_AOA_VT(AOA_eff_LVT)*Vehicle.Aero.Left_VS.Cl_3DCorr;
Cd_LVT = Vehicle.Aero.GI_Cd_AOA_VT(AOA_eff_LVT)*Vehicle.Aero.Left_VS.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_LVT(LRudIndices) = Cd_LVT(LRudIndices) + delCD_p_Rudder_L;
Cm_LVT = Vehicle.Aero.GI_Cm_AOA_VT(AOA_eff_LVT);

AOA_eff_RVT = AOA_eff(RVTIndices);
Cl_RVT = Vehicle.Aero.GI_Cl_AOA_VT(AOA_eff_RVT)*Vehicle.Aero.Right_VS.Cl_3DCorr;
Cd_RVT = Vehicle.Aero.GI_Cd_AOA_VT(AOA_eff_RVT)*Vehicle.Aero.Right_VS.Cd0_3DCorr*Vehicle.Aero.KCD0;
Cd_RVT(RRudIndices) = Cd_RVT(RRudIndices) + delCD_p_Rudder_R;
Cm_RVT = Vehicle.Aero.GI_Cm_AOA_VT(AOA_eff_RVT);



% Cl = [0*Cl_LW; 0*Cl_RW; Cl_LHT; Cl_RHT; 0*Cl_LVT; 0*Cl_RVT];
% Cd = [0*Cd_LW; 0*Cd_RW; Cd_LHT; Cd_RHT; 0*Cd_LVT; 0*Cd_RVT];
% Cm = [0*Cm_LW; 0*Cm_RW; Cm_LHT; Cm_RHT; 0*Cm_LVT; 0*Cm_RVT];


Cl = [Cl_LW; Cl_RW; Cl_LHT; Cl_RHT; Cl_LVT; Cl_RVT];
Cd = [Cd_LW; Cd_RW; Cd_LHT; Cd_RHT; Cd_LVT; Cd_RVT];
Cm = [Cm_LW; Cm_RW; Cm_LHT; Cm_RHT; Cm_LVT; Cm_RVT];



%% Compute strip lift, drag, and pitching moment loads

dA = Vehicle.Aero.StripDefn.Area;      % m2

c_strip = Vehicle.Aero.StripDefn.Chord; % m



% 
dL_s = qbar_s .* dA .* Cl ;
dD_s = qbar_s .* dA .* Cd +       InducedDrag;
dM_s = qbar_s .* dA .* c_strip .* Cm;               


% CD = sum(dD_s)/(qbar_s(1) * Vehicle.Geom.S);
% CD0 = sum(qbar_s .* dA .* Cd)/(qbar_s(1) * Vehicle.Geom.S);
% CDi = sum(InducedDrag)/(qbar_s(1) * Vehicle.Geom.S);
% CL = sum(dL_s)/(qbar_s(1) * Vehicle.Geom.S);


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
