function [Vehicle] = MissionAndConstraints(Vehicle,Range_km,Architecture)

if isempty(Range_km)
    Range_km = 250;
end

Mission.SingleTripDist_km = Range_km;

ctr = 1;

Mission.Name = 'Sizing Mission';

%% DESIGN MISSION PROFILE DEFINITION

% Points 1,2
Segments(ctr).Name = 'Vert Climb';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = 6000/3.28;
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 2,3
Segments(ctr).Name = 'Hover_1';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = 6050/3.28;
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 3,4
Segments(ctr).Name = 'VFtoFF_1';
Segments(ctr).Type = 'transition';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndVel = Vehicle.Aero.VThreshold + 5*0.51444;
Segments(ctr).FPA = 0*pi/180;
Segments(ctr).dhdt = []; %400/(3.28*60);
Segments(ctr).dVdt = 0.06* 9.81;
Segments(ctr).Points = 6;
Segments(ctr).PowerSplit = [0.5,0];
ctr = ctr + 1;

% Points 4,5
Segments(ctr).Name = 'Climb_1';
Segments(ctr).Type = 'climb';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndAlt = 10000/3.28;
Segments(ctr).TAS = [];
Segments(ctr).EAS = Vehicle.Aero.VThreshold + 0.75*(Vehicle.Operations.VC*0.51444 - Vehicle.Aero.VThreshold);
Segments(ctr).ROC = 1000/(3.28*60);
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr+1;


% Points 5,6
Segments(ctr).Name = 'Cruise_1';
Segments(ctr).Type = 'cruise';
Segments(ctr).StartAlt = []; %8000/3.28;
Segments(ctr).EndAlt = 10000/3.28; %8000/3.28;
Segments(ctr).TAS = Vehicle.Operations.VC*0.51444;%135*0.5144;
Segments(ctr).PowerSplit = [1,0];
ctr = ctr+1;

% Points 6,7
Segments(ctr).Name = 'Descent_1';
Segments(ctr).Type = 'descent';
Segments(ctr).StartAlt = [];
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).EAS = Vehicle.Aero.VThreshold + 2*0.51444;
Segments(ctr).ROD = []; %500/(3.28*60);
Segments(ctr).PowerSetting = [0];
Segments(ctr).Points = 5;
ctr = ctr + 1;

% Points 7,8
Segments(ctr).Name = 'FFtoVF_1';
Segments(ctr).Type = 'transition';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndVel = 00*0.51444;
Segments(ctr).FPA = [];
Segments(ctr).dhdt = 0;
Segments(ctr).dVdt = -0.03*9.81;
Segments(ctr).Points = 6;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;


% Points 8,9
Segments(ctr).Name = 'Hover_2';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 9,10
Segments(ctr).Name = 'VertDescent_1';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6000/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% %%%% SECOND TRIP
% Points 10,11
Segments(ctr).Name = 'Vert Climb_2';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = 6000/3.28;
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 11,12
Segments(ctr).Name = 'Hover_3';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = 6050/3.28;
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 12,13
Segments(ctr).Name = 'VFtoFF_2';
Segments(ctr).Type = 'transition';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndVel = Vehicle.Aero.VThreshold + 5*0.51444;
Segments(ctr).FPA = 0*pi/180;
Segments(ctr).dhdt = []; %400/(3.28*60);
Segments(ctr).dVdt = 0.08*9.81; %0.06* 9.81;
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [0.5,0];
ctr = ctr + 1;

% Points 13,14
Segments(ctr).Name = 'Climb_2';
Segments(ctr).Type = 'climb';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndAlt = 10000/3.28;
Segments(ctr).TAS = [];
Segments(ctr).EAS = Vehicle.Aero.VThreshold + 0.75*(Vehicle.Operations.VC*0.51444 - Vehicle.Aero.VThreshold);
Segments(ctr).ROC = 1000/(3.28*60);
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr+1;


% Points 14,15
Segments(ctr).Name = 'Cruise_2';
Segments(ctr).Type = 'cruise';
Segments(ctr).StartAlt = []; %8000/3.28;
Segments(ctr).EndAlt = 10000/3.28; %8000/3.28;
Segments(ctr).TAS = Vehicle.Operations.VC*0.51444;%135*0.5144;
Segments(ctr).PowerSplit = [1,0];
ctr = ctr+1;

% Points 15,16
Segments(ctr).Name = 'Cruise_Res';
Segments(ctr).Type = 'loiter';
Segments(ctr).StartAlt = []; %8000/3.28;
Segments(ctr).EndAlt = 10000/3.28; %8000/3.28;
Segments(ctr).TAS = Vehicle.Operations.VC*0.51444;%135*0.5144;
Segments(ctr).PowerSplit = [1,0];
ctr = ctr+1;

% Points 16,17
Segments(ctr).Name = 'Descent_2';
Segments(ctr).Type = 'descent';
Segments(ctr).StartAlt = [];
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).EAS = Vehicle.Aero.VThreshold + 2*0.51444;
Segments(ctr).ROD = []; %500/(3.28*60);
Segments(ctr).PowerSetting = [0];
Segments(ctr).Points = 5;
ctr = ctr + 1;

% Points 17,18
Segments(ctr).Name = 'FFtoVF_2';
Segments(ctr).Type = 'transition';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = [];
Segments(ctr).EndVel = 00*0.51444;
Segments(ctr).FPA = [];
Segments(ctr).dhdt = 0;
Segments(ctr).dVdt = -0.03*9.81;
Segments(ctr).Points = 6;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;


% Points 18,19
Segments(ctr).Name = 'Hover_4';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6050/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Points 19,20
Segments(ctr).Name = 'VertDescent_2';
Segments(ctr).Type = 'verticalflight';
Segments(ctr).StartAlt = [];
Segments(ctr).StartVel = 0;
Segments(ctr).EndAlt = 6000/3.28;
Segments(ctr).TAS = 0;
Segments(ctr).EAS = []; %80*0.51444;
Segments(ctr).Mach = [];
Segments(ctr).Points = 5;
Segments(ctr).PowerSplit = [1.0,0];
ctr = ctr + 1;

% Collect defined mission segments into structure
Mission.Segments = Segments;

% specify distance requirements
Dists = zeros(20,20);
Dists(4,6) = Range_km * 1000;
Dists(13,15) = Range_km * 1000;
% Dists(15,16) = Vehicle.Operations.VC*0.51444 * 20 * 60;
Mission.Dists = Dists;


% specify duration requirements
Durations(1,2) = 0.50;
Durations(2,3) = 0.25;
Durations(3,4) = 0.17;
Durations(8,9) = 0.5;
Durations(9,10) = 0.50;

Durations(10,11) = 0.5;
Durations(11,12) = 0.25;
Durations(12,13) = 0.17;
Durations(15,16) = 20;
Durations(18,19) = 0.5;
Durations(19,20) = 0.5;




Mission.Durations = Durations;

Mission.PropTypes = 2;
Mission.dISA = 0;


% battery DOD at start and end of mission
StartDOD = 0.05;


EndDOD = 0.85;

% define start energy percentage
StartBattEFrac = 100 * (1-Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(StartDOD));
StartUsedREPerc = Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(StartDOD);
Mission.StartEnergyPerc = [100, StartBattEFrac];

% define residual energy requirements and definition type

% EndBattEFrac = 100*(FindAreaUnderOCVCurve(EndDOD, 1, Vehicle.Propulsion.Battery)/FindAreaUnderOCVCurve(StartDOD, EndDOD, Vehicle.Propulsion.Battery));

EndBattEFrac = 100*((1-Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(EndDOD))/(1-Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(StartDOD)));

Mission.ResEnergyPerc = [5,EndBattEFrac];
Mission.ResEnergyType = {'used','start'};

Vehicle.DefaultStartStates = [StartDOD, 0, 0, StartUsedREPerc];

% run mission initializer
% Mission = Initializer(Mission);
Vehicle.Mission = Mission;

%% END: Mission profile definition


%% POINT PERFORMANCE CONSTRAINTS DEFINITION


HiHotWtDiscount = -100; % permissible payload reduction for hi-hot conditions

ctr = 1;
PointPerf.Name(ctr,1) = {'Cruise High'};
PointPerf.KEAS(ctr) = 0;
PointPerf.KTAS(ctr,1) = Vehicle.Operations.VC;
PointPerf.PA_ft(ctr,1) = 10000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 0;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'Cruise Low'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = Vehicle.Operations.VC;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 0;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'Cruise degraded'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0.5*(Vehicle.Aero.VThreshold/0.51444 + Vehicle.Operations.VC);
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 0;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'Climb'};
PointPerf.KEAS(ctr,1) = (Vehicle.Aero.VThreshold + 0.25*(Vehicle.Operations.VC*0.51444 - Vehicle.Aero.VThreshold))/0.51444; % (0.25 * Vehicle.Aero.VThreshold + 0.75 * Vehicle.Operations.VC*0.51444)/0.51444;  %Vehicle.Aero.VThreshold/0.51444 + 1;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0000;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 900;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'Climb HOT'};
PointPerf.KEAS(ctr,1) = (Vehicle.Aero.VThreshold + 0.25*(Vehicle.Operations.VC*0.51444 - Vehicle.Aero.VThreshold))/0.51444; %Vehicle.Aero.VThreshold/0.51444 + 1;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0000;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 400;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'Transition Accel'};
PointPerf.KEAS(ctr,1) =  Vehicle.Aero.VThreshold/0.51444 + 2;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 000;
PointPerf.ACC_g(ctr,1) = 0.06;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'HOGE'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 000;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'VCLMB'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 1000;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;


PointPerf.Name(ctr,1) = {'HOGE P1 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [0,1,1,1,1,1];
ctr = ctr + 1;


PointPerf.Name(ctr,1) = {'HOGE P2 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 00;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,0,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'HOGE P5 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 0;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,0,1];
ctr = ctr + 1;

% PointPerf.Name(ctr,1) = {'HOGE SL P6 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 0;
% PointPerf.WTFRAC(ctr,1) = 1;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [1,1,1,1,1,0];
% ctr = ctr + 1;
% 
% PointPerf.Name(ctr,1) = {'HOGE SL P1,4 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 0;
% PointPerf.WTFRAC(ctr,1) = 1;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [0,1,1,0,1,1];
% ctr = ctr + 1;
% 
% PointPerf.Name(ctr,1) = {'HOGE SL P2,6 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 0;
% PointPerf.WTFRAC(ctr,1) = 1;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [1,0,1,1,1,0];
% ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'HOGE HI-HOT'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 000;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'VCLMB HI-HOT'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 500;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,1,1];
ctr = ctr + 1;



PointPerf.Name(ctr,1) = {'HOGE HI-HOT P1 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [0,1,1,1,1,1];
ctr = ctr + 1;


PointPerf.Name(ctr,1) = {'HOGE HI-HOT P2 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,0,1,1,1,1];
ctr = ctr + 1;

PointPerf.Name(ctr,1) = {'HOGE HI-HOT P5 INOP'};
PointPerf.KEAS(ctr,1) = 0;
PointPerf.KTAS(ctr,1) = 0;
PointPerf.PA_ft(ctr,1) = 6000;
PointPerf.DA_ft(ctr,1) = 0;
PointPerf.dISA_C(ctr,1) = 10;
PointPerf.WTFRAC(ctr,1) = 1;
PointPerf.FPM(ctr,1) = 100;
PointPerf.ACC_g(ctr,1) = 0;
PointPerf.Health(ctr,:) = [1,1,1,1,0,1];
% ctr = ctr + 1;

% PointPerf.Name(ctr,1) = {'HOGE HI-HOT P6 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 6000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 10;
% PointPerf.WTFRAC(ctr,1) = HiHotWtDiscount;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [1,1,1,1,1,0];
% ctr = ctr + 1;
% 
% PointPerf.Name(ctr,1) = {'HOGE HI-HOT P1,4 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 6000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 10;
% PointPerf.WTFRAC(ctr,1) = HiHotWtDiscount;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [0,1,1,0,1,1];
% ctr = ctr + 1;
% 
% PointPerf.Name(ctr,1) = {'HOGE HI-HOT P2,6 INOP'};
% PointPerf.KEAS(ctr,1) = 0;
% PointPerf.KTAS(ctr,1) = 0;
% PointPerf.PA_ft(ctr,1) = 6000;
% PointPerf.DA_ft(ctr,1) = 0;
% PointPerf.dISA_C(ctr,1) = 10;
% PointPerf.WTFRAC(ctr,1) = HiHotWtDiscount;
% PointPerf.FPM(ctr,1) = 100;
% PointPerf.ACC_g(ctr,1) = 0;
% PointPerf.Health(ctr,:) = [1,0,1,1,1,0];



PP.Case = [1:ctr]'; PP = struct2table(PP);

PointPerf = struct2table(PointPerf); PointPerf = [PP, PointPerf];
[PointPerf] = CompletePointPerfTable(PointPerf);
Constraints = PointPerf;


%% END: Point performance constraints definition



%% ARCHITECTURE 1: power sizing cases definition
if Architecture==1
    
    %           PointPerf Case     BattsLost    Motor Discount Factor
    A1Matrix = [...
        1                       0           1       % cruise high, nominal
        2                       0           1       % cruise low, nominal
        3                       1           1       % cruise degraded, 1 pack lost
        4                       0           1       % climb sea-level, nominal
        5                       0           1       % climb hi-hot, nominal
        6                       0           1       % transition accel
        % SL hover cases, NOMINAL
        7                       1           1       % hover SL, nominal
        8                       0           1       % vertical climb, standard, nominal
        % SL hover cases, OFF-NOMINAL
        9                       0           1.5     % hover SL, P1 out, off-nominal
        10                      0           1.5     % hover SL, P2 out, off-nominal
        11                      0           1.5     % hover SL, P5 out, off-nominal
        % HI-HOT hover cases, NOMINAL     
        12                      1           1       % hover hi-hot, nominal
        13                      0           1       % vertical climb, hi-hot, nominal
        % HI-HOT hover cases, OFF-NOMINAL
        14                      0           1.5     % hover hi-hot, P1 out, off-nominal
        15                      0           1.5     % hover hi-hot, P2 out, off-nominal
        16                      0           1.5     % hover hi-hot, P5 out, off-nominal
        ];
    

    PowerSizingCases.Case = A1Matrix(:,1);
    PowerSizingCases.Batts = Vehicle.Propulsion.NBatteryPacks - A1Matrix(:,2);
    PowerSizingCases.km = A1Matrix(:,3);
    
end




%% ARCHITECTURE 3: power sizing cases definition
if Architecture==3
    
    %           PointPerf Case     TG     Motor Discount Factor
    A3Matrix = [...
        1                       0           1       % cruise high, nominal
        2                       0           1       % cruise low, nominal
        3                       1           1       % cruise degraded, 1 TG lost
        4                       0           1       % climb , nominal
        5                       0           1       % climb hot, nominal
        % 6                       1           1       % climb degraded, 1 TG lost
        6                       0           1       % Transition acceleration
        % SL hover cases, NOMINAL        
        7                       0           1       % hover SL, nominal
        8                       0           1       % vertical climb, SL, nominal
        % SL hover cases, OFF-NOMINAL
        7                       1           1       % hover SL, 1 TG lost
        9                       0           1.5     % hover SL, P1 out, off-nominal
        10                      0           1.5     % hover SL, P2 out, off-nominal
        11                      0           1.5     % hover SL, P5 out, off-nominal      
        % HI-HOT hover cases, NOMINAL
        12                      0           1       % hover hi-hot, nominal 
        13                      0           1       % vertical climb, hi-hot, nominal
        % HI-HOT hover cases, OFF-NOMINAL
        12                      1           1       % hover hi-hot, 1 TG lost
        14                      0           1.5     % hover hi-hot, P1 out, off-nominal
        15                      0           1.5     % hover hi-hot, P2 out, off-nominal
        16                      0           1.5     % hover hi-hot, P5 out, off-nominal
        ];
    
    
    
    

    PowerSizingCases.Case = A3Matrix(:,1);
    PowerSizingCases.TG = Vehicle.Propulsion.NTurbogenerators - A3Matrix(:,2);
    PowerSizingCases.NBatt = Vehicle.Propulsion.NBatteryPacks .* ones(length(A3Matrix),1);
    PowerSizingCases.km = A3Matrix(:,3);
    
end


Vehicle.PointPerf.Constraints = Constraints;
Vehicle.PointPerf.PowerSizingCases = PowerSizingCases;



%% LOCAL FUNCTIONS

    function [PointPerf] = CompletePointPerfTable(PointPerf)
        
        R = 287.051;
        Conv_ft_to_m = 1/3.28084;
        
        [~,~,~,rhoSL] = atmosisa(0);
        
        
        href = [0:100:20000];
        [Tref,~,Pref,rhoref] = atmosisa(href * Conv_ft_to_m);
        
        
        PA_ft = zeros(height(PointPerf),1);
        DA_ft = zeros(height(PointPerf),1);
        
        for i = 1:height(PointPerf)
            
            
            
            % if dISA = 0, respect PA and set DA = PA
            if PointPerf.dISA_C(i) == 0
                PA_ft(i) = PointPerf.PA_ft(i);
                DA_ft(i) = PointPerf.PA_ft(i);
                [~,~,~,PointPerf.rho(i)] = atmosisa(PointPerf.PA_ft(i) * Conv_ft_to_m);
            end
            
            if PointPerf.dISA_C(i)~=0
                
                % if pressure altitude is given
                if PointPerf.PA_ft(i) > 0
                    PA_ft(i) = PointPerf.PA_ft(i);
                    [Tstd,~,Pstd,~] = atmosisa(PointPerf.PA_ft(i) * Conv_ft_to_m);
                    % account for actual temperature by adding ISA deviation
                    T = Tstd + PointPerf.dISA_C(i);
                    
                    % calculate actual density using ideal gas formula (kg/m3)
                    rho = Pstd / (R * T);
                    PointPerf.rho(i) = rho;
                    
                    % find where in the standard atmosphere this density occurs
                    DA_ft(i) = round(interp1(rhoref,href,rho,'linear'));
                end
                
                % if density altitude is given
                if PointPerf.DA_ft(i) > 0
                    DA_ft(i) = PointPerf.DA_ft(i);
                    [~,~,~,rho] = atmosisa(PointPerf.DA_ft(i) * Conv_ft_to_m);
                    PointPerf.rho(i) = rho;
                    
                    % add dISA to the standard atmosphere temps at each altitude
                    Ttest = Tref + PointPerf.dISA_C(i);
                    
                    % calculate corresponding rho
                    rhotest = Pref./(R.*Ttest);
                    
                    % find at which altitude in the standard atmosphere this
                    % density occurs
                    PA_ft(i) = round(interp1(rhotest,href,rho,'linear'));
                    
                    
                end
                
            end
            
            
            % set density ratio
            PointPerf.SIG(i,1) = PointPerf.rho(i)/rhoSL;
            
            
            % set equivalent and true airspeeds
            % if TAS is specified
            if(PointPerf.KTAS(i)>0)
                PointPerf.KEAS(i,1) = PointPerf.KTAS(i)*sqrt(PointPerf.SIG(i,1));
            end
            % if EAS is specified
            if(PointPerf.KEAS(i)>0)
                PointPerf.KTAS(i,1) = PointPerf.KEAS(i)/sqrt(PointPerf.SIG(i,1));
            end

            %if EAS is less than VThreshold, change EAS to VThreshold
            if i <=3
               if(PointPerf.KEAS(i) < Vehicle.Aero.VThreshold/0.51444)
                   PointPerf.KEAS(i,1) = Vehicle.Aero.VThreshold/0.51444 + 1;
                   PointPerf.KTAS(i,1) = PointPerf.KEAS(i)/sqrt(PointPerf.SIG(i,1));
               end
            end

            
        end
        
        PointPerf.PA_ft = PA_ft;
        PointPerf.DA_ft = DA_ft;
    end











end