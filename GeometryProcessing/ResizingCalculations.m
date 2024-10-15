%% Assumptions
BoomToMotorDimensionRatio = 1.0;
Vehicle.Recalcs.MaxPropDiam_m = 10/3.28;


DL = Vehicle.DesignPoint.DL_kgm2;                                             % Designated Disc Loading [kg/m2]
MTOM_kg = Vehicle.MassProp.MTOM_kg;                                         % Aircraft Gross Weight [kg]



%% Wing Recalcs
Vehicle.Recalcs.WingArea_TOT_m2 = MTOM_kg./Vehicle.DesignPoint.WL_kgm2;             % [m2]
Vehicle.Recalcs.WingArea_Each_m2 = Vehicle.Recalcs.WingArea_TOT_m2/2;                       % [m2]
Vehicle.Recalcs.WingSpan_m = sqrt(Vehicle.Recalcs.WingArea_TOT_m2 * Vehicle.DesignPoint.WingAspectRatio); % [m]
Vehicle.Recalcs.WingRootChord_m = 2*Vehicle.Recalcs.WingArea_TOT_m2/((1 + Vehicle.DesignPoint.WingTaperRatio)*Vehicle.Recalcs.WingSpan_m);

%% Prop and Rotor Recalcs
% calculate lift prop diameter based on disc loading
PropArea_TOT_m2 = MTOM_kg/Vehicle.DesignPoint.DL_kgm2;
maxPropArea_Each_m2 = pi*Vehicle.Recalcs.MaxPropDiam_m^2 / 4;
PropArea_Each_m2 = min(maxPropArea_Each_m2,PropArea_TOT_m2/(Vehicle.Propulsion.NLiftProps+Vehicle.Propulsion.NCruiseProps));
Vehicle.Recalcs.LiftPropDiam_m = sqrt(4 * PropArea_Each_m2/pi);
Vehicle.Propulsion.LiftPropDiam_m = Vehicle.Recalcs.LiftPropDiam_m;

Vehicle.Recalcs.MainPropDiam_m = Vehicle.Recalcs.LiftPropDiam_m;
Vehicle.Propulsion.MainPropDiam_m = Vehicle.Recalcs.MainPropDiam_m;
       
%% Lift Motor dimension Recalcs
Vehicle.Recalcs.LiftMotorPower_Each_kW = MTOM_kg * Vehicle.DesignPoint.Lift_PMR_kWkg / Vehicle.Propulsion.NLiftProps;
Vehicle.Recalcs.LiftMotorDiameter_m = Vehicle.Recalcs.LiftMotorPower_Each_kW * Vehicle.SOTA.ElectricMotorSpecDiameter_mkW; 
Vehicle.Recalcs.LiftMotorLength_m = Vehicle.Recalcs.LiftMotorPower_Each_kW * Vehicle.SOTA.ElectricMotorSpecLength_mkW;
Vehicle.Propulsion.LiftMotorPower_Each_kW = Vehicle.Recalcs.LiftMotorPower_Each_kW;

%% Main Motor calculations
Vehicle.Recalcs.MainMotorPower_Each_kW = Vehicle.DesignPoint.Cruise_PMR_kWkg * Vehicle.MassProp.MTOM_kg / Vehicle.Propulsion.NCruiseProps;
Vehicle.Propulsion.MainMotorPower_Each_kW = Vehicle.Recalcs.MainMotorPower_Each_kW;

%% Generator dimension recalcs
Vehicle.Recalcs.GeneratorDiam_m =  Vehicle.Propulsion.GeneratorPower_Each_kW * Vehicle.SOTA.GeneratorSpecDiameter_mkW;
Vehicle.Recalcs.GeneratorLength_m = Vehicle.Propulsion.GeneratorPower_Each_kW * Vehicle.SOTA.GeneratorSpecLength_mkW;

%% Turboshaft dimension recalcs
Vehicle.Recalcs.TurboshaftLength_m = Vehicle.Propulsion.TurboshaftPower_Each_kW * Vehicle.SOTA.TurboshaftSpecLength_mkW;
Vehicle.Recalcs.TurboshaftDiam_m = Vehicle.Propulsion.TurboshaftPower_Each_kW * Vehicle.SOTA.TurboshaftSpecDia_mkW;

%% Boom cross section Recalcs
b_h = Vehicle.SOTA.ElectricMotorSpecDiameter_mkW /  Vehicle.SOTA.ElectricMotorSpecLength_mkW;

Vehicle.Recalcs.BoomWidth = BoomToMotorDimensionRatio * Vehicle.Recalcs.LiftMotorDiameter_m; %Vehicle.Propulsion.LPSetup.RootCutOutFraction*Vehicle.Propulsion.LiftPropDiam_m;

Vehicle.Recalcs.BoomHeight = BoomToMotorDimensionRatio * Vehicle.Recalcs.LiftMotorLength_m;

Vehicle.Recalcs.MaxHtWdBoom = max(Vehicle.Recalcs.BoomWidth,Vehicle.Recalcs.BoomHeight);
