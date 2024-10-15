function [Vehicle] = EMPAERO_FixedLandingGear(Vehicle)
% Drag of fixed landing gear struts with tires
% Based on General Aviation Aircraft Design : Applied Methods and Procedures
% Gudmundsson(2014) pg. 718-721

% Specific to NASA-LPC
Vehicle.Geom.LandingGear_Nose = Vehicle.Geom.LandingGear_Nose;
Vehicle.Geom.LandingGear_Main_1 = Vehicle.Geom.LandingGear_Main_1;
Vehicle.Geom.LandingGear_Main_2 = Vehicle.Geom.LandingGear_Main_2;

NG_Wheel = Vehicle.Geom.NoseWheel;
MG_Wheel = Vehicle.Geom.MainWheel_1;


%% Dimensions 
% d : tire diameter
% w : tire width 
d_MG = MG_Wheel.Stn.c(1);
w_MG = MG_Wheel.Span; 
d_NG = NG_Wheel.Stn.c(1);
w_NG = NG_Wheel.Span;

Sref = Vehicle.Recalcs.WingArea_TOT_m2;
%% Main LG
% Follow the following steps to approximate main landing gear 
% Step1: Choose the strut type of the main landing gear using Fig 15-50 
% Step2: Determine the wheel fairing style using Fig 15-49
% Step3: Choose the tire type using Fig 14 -48
% Step4: Read the delta_C_DS from Table 15-11 
% Step5: Use eqn 15-91 to find the additive drag coefficient for the FLG

% For MG similar to that of CozyMKIV 
StrutType = 'J2';
FairingType ='A1';
TireType = 'B';
dC_DS = 0.458;
dC_DMain = (d_MG*w_MG)/Sref*dC_DS;
%% Nose LG
% Follow the exact same steps as followed for main LG
% Divide the delta_C_DS by 2 as the value pertains to a set of main LG

% For NG similar to that of SR22
StrutType = 'M1';
FairingType = 'A1';
TireType = 'B';
dC_DS = 0.484;
dC_DS_NG = dC_DS/2;
dC_DNose = (d_NG*w_NG)/Sref*dC_DS_NG;
% table(dC_DMain,dC_DNose);

%% Drag count 
% MLD_DC = 1e4*dC_DMain;
% NLD_DC = 1e4*dC_DNose;
% Drag_count = table(MLD_DC,NLD_DC);
% % additive drag coefficints 
Vehicle.Aero.MLG_dragcoeff = dC_DMain;
Vehicle.Aero.NLG_dragcoeff = dC_DNose;
end 