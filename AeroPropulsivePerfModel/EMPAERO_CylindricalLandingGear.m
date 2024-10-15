function [Vehicle] = EMPAERO_CylindricalLandingGear(Vehicle)

% Nose landing gear  modeled as a cylinder perpendicular to the flow
NG = Vehicle.Geom.NG;
L = NG.Span;
D = NG.Stn.c(1);
L2D = L/D;
Vehicle.Aero.RefArea_NG = L*D;

CD_NG = Vehicle.Aero.GI_CD_fcn_L2D_FlowPerpendicularToCylinder(L2D);

% Nose landing gear wheel 
% Gudmundsson Reader pg. 718 Table 15-9
d_NG = Vehicle.Geom.NG_Wheel.Stn.c(1);
w_NG = Vehicle.Geom.NG_Wheel.Span;
deltaCD_NG = 0.18;

Vehicle.Aero.CD_NG = CD_NG;
Vehicle.Aero.deltaCD_NG = deltaCD_NG;
Vehicle.Aero.RefArea_NG_Wheel = d_NG * w_NG;

% Main landing gear  modeled as a cylinder perpendicular to the flow
LMG = Vehicle.Geom.LMG;
L = LMG.Span;
D = LMG.Stn.c(1);
L2D = L/D;
Vehicle.Aero.RefArea_LMG = L*D;

CD_LMG = Vehicle.Aero.GI_CD_fcn_L2D_FlowPerpendicularToCylinder(L2D);

% Nose landing gear wheel 
% Gudmundsson Reader pg. 718 Table 15-9
d_LMG = Vehicle.Geom.LMG_Wheel.Stn.c(1);
w_LMG = Vehicle.Geom.LMG_Wheel.Span;
deltaCD_LMG = 0.18;

Vehicle.Aero.CD_LMG = CD_LMG;
Vehicle.Aero.deltaCD_LMG = deltaCD_LMG;
Vehicle.Aero.RefArea_LMG_Wheel = d_LMG * w_LMG;

CD_RMG = CD_LMG ;
deltaCD_RMG = deltaCD_LMG;

Vehicle.Aero.CD_RMG = CD_RMG;
Vehicle.Aero.deltaCD_RMG = deltaCD_RMG;
Vehicle.Aero.RefArea_RMG_Wheel = d_LMG * w_LMG;

Vehicle.Aero.RefArea_RMG = L*D;
end