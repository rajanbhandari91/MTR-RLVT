function Vehicle = VehicleDefinition_Baseline(Architecture,FF)
% Collect inputs for full-factorial sweep


DL = FF.DL;
WL = FF.WL;
Vcruise = FF.Vcruise;
SpecificEnergy_KWhkg = FF.EnergyDensity_kWhkg;
MaxCRate = FF.MaxCRate;


Vehicle.Architecture = Architecture;
% DUeVTOL Baseline Vehicle Definition
Vehicle.MassProp.EnergyMass_kg = [1 1];
Vehicle.Propulsion.GearboxPower_Each_kW = 5;


%% Design Point Specification
Vehicle.DesignPoint.WL_kgm2 = WL * 4.88243;                                 % Wing Loading [kg/m2] [note: 1 lb/ft2 = 4.88243 kg/m2]
Vehicle.DesignPoint.DL_kgm2 = DL * 4.88243;                                 % Disc Loading [kg/m2] [note: 1 lb/ft2 = 4.88243 kg/m2]
Vehicle.DesignPoint.DL_Requested = [DL*4.88243];

% Vehicle.DesignPoint.DL_kgm2 = 150;

Vehicle.DesignPoint.WingAspectRatio = 10;                                 % Wing Aspect Ratio
Vehicle.DesignPoint.WingTaperRatio = 0.8;
Vehicle.DesignPoint.Cruise_PMR_kWkg = 1*0.2; %1.6 KW / 8kg                               % Cruise motor installed power / vehicle mass
Vehicle.DesignPoint.Lift_PMR_kWkg = 8*0.0750;     %0.60 KW / 8 kg                              % Lift motor installed power / vehicle mass
Vehicle.DesignPoint.Payload = 0.8;                                          % [kg]
Vehicle.DesignPoint.DesignMach = 0.07;

% Vehicle.DesignPoint.CanardVolumeRatio = 0.4;                                % canard volume ratio
% Vehicle.DesignPoint.CanardAspectRatio = 11;                                 % canard aspect ratio

Vehicle.DesignPoint.HTVolumeRatio = 1.2;                                  % horizontal tail volume ratio
Vehicle.DesignPoint.HTAspectRatio = 7.00;                                  % horizontal tail aspect ratio

Vehicle.DesignPoint.VTAspectRatio = 1.1;                                   % vertical tail aspect ratio
% Vehicle.DesignPoint.VTVolumeRatio = 0.10;                                   % vertical tail volume ratio

%% TECHNOLOGICAL STATES OF THE ART (SOTA)
Vehicle.SOTA.ICEnginePMR_kWkg = 1.0;

% turboshaft engines
Vehicle.SOTA.TurboshaftSFC_lbhph = 0.5;
Vehicle.SOTA.TurboshaftSFC_kgkWh = Vehicle.SOTA.TurboshaftSFC_lbhph * (0.4536/0.746);

% electric motors
Vehicle.SOTA.ElectricMotorPMR_kWkg = 5.66;                                   % kW/kg, based on KV450 (KW/Nir
Vehicle.SOTA.ElectricMotorSpecDiameter_mkW = 0.0855; %0.0556/0.65                       % electric motor diameter/power rating (m/kW)
Vehicle.SOTA.ElectricMotorSpecLength_mkW = 0.0462;   %0.03/0.65                       % electric motor length/power rating (m/kW)
Vehicle.SOTA.MotorEfficiency = 0.7;

% electric generators
Vehicle.SOTA.GeneratorPMR_kWkg = 6.25;                                      % kW/kg, Honeywell
Vehicle.SOTA.GeneratorEfficiency = 0.97;                                    % Honeywell
Vehicle.SOTA.GeneratorSpecDiameter_mkW = 0.36/1000;                        % m/kW, based on Honeywell design, 14" (0.36 m) for 1 MW (1000 kW) output
Vehicle.SOTA.GeneratorSpecLength_mkW = 0.61/1000;                          % m/kW, based on on Honeywell design, 24" (0.61 m) for 1 MW (1000 kW) output


% electric batteries
Vehicle.SOTA.BatterySpecificEnergy_kWhkg = SpecificEnergy_KWhkg; 
Vehicle.SOTA.BatteryEfficiency = 0.98;
Vehicle.SOTA.BatteryMaxCRate = MaxCRate;
Vehicle.SOTA.BatteryPMR_kWkg = Vehicle.SOTA.BatterySpecificEnergy_kWhkg * Vehicle.SOTA.BatteryMaxCRate;

% electric cabling
Vehicle.SOTA.Cabling_kVAmkg = 200;                                          % kVA.m/kg, Christou et al, curve C at 540V
Vehicle.SOTA.CableEfficiency = 0.985;

% fuel
Vehicle.SOTA.FuelSpecificEnergy_MJkg = 43;
Vehicle.SOTA.FuelDensity_kgm3 = 820; 

% gearbox
Vehicle.SOTA.GearboxEfficiency = 0.99;
Vehicle.SOTA.GearboxPMR_kWkg = 24;

Vehicle.SOTA.DuctedFanPowerReduction = 0.707; %1/sqrt(2);
Vehicle.SOTA.DuctD_WperArea = 3.5;                                          % Duct Weight per surface area 
                                                                            % "NASA TM: EMPRICAL EXPRESSIONS FOR ESTIMATING  
% computed
Vehicle.SOTA.BatterySpecificEnergy_MJkg = 3.6 * Vehicle.SOTA.BatterySpecificEnergy_kWhkg;
Vehicle.SOTA.BatteryEnergyDensity_kWhm3 = 2 * 1000 * Vehicle.SOTA.BatterySpecificEnergy_kWhkg; % temp value


Vehicle.SOTA.SpecificEnergy_MJkg = [Vehicle.SOTA.FuelSpecificEnergy_MJkg, Vehicle.SOTA.BatterySpecificEnergy_MJkg];
Vehicle.SOTA.EnergyDensity_kgm3 = [Vehicle.SOTA.FuelDensity_kgm3, Vehicle.SOTA.BatteryEnergyDensity_kWhm3 / Vehicle.SOTA.BatterySpecificEnergy_kWhkg];

% account for transmission line efficiency by incorporating this into the
% efficiency of certain components
Vehicle.SOTA.GeneratorEfficiency = Vehicle.SOTA.GeneratorEfficiency * Vehicle.SOTA.CableEfficiency;
Vehicle.SOTA.BatteryEfficiency = Vehicle.SOTA.BatteryEfficiency * Vehicle.SOTA.CableEfficiency;



%% some default init values
Vehicle.Propulsion.NTurbogenerators = 1;




%% All Electric Architecture (AE)
if Architecture == 1
    NumBattZones = 2;
    Vehicle.Propulsion.BattZones = 1:2;

    NumFuelZones = 1;   % even though there is no fuel, needs to be specified as 1 at least
    
    
    
    Vehicle.MassProp.EnergyMass_kg = [0 1];
    Vehicle.Propulsion.Neng = 0;                                            % number of internal combustion engines
    Vehicle.Propulsion.Turboshaft_Each_kW = .01;

    Vehicle.Propulsion.Ntank = 2;                                           % number of fuel tanks
    Vehicle.Propulsion.Qtot = 1/6.2;                                        % total volume of fuel, USG
    Vehicle.Propulsion.Qint = 0;                                            % total volume of fuel in integral tanks, USG

    Vehicle.Propulsion.GeneratorPower_Each_kW = 01;                         % total generator power rating, kW
    Vehicle.Propulsion.NBatteryPacks = 2;

    % Power flow A-matrix (size 12 x 12)
    
    % first 9 x 9 block is a diagonal matrix of ones
    A = eye(9);
    
    % change A to the correct size
    A(12,12) = 0;
    
    % row 10:
    A(10,1:9) = 1; A(10,12) = -Vehicle.SOTA.MotorEfficiency;
    
    % row 11:
    A(11,10) = Vehicle.Propulsion.NBatteryPacks; A(11,12) = -1;
    
    % row 12:
    A(12,10) = Vehicle.Propulsion.NBatteryPacks/Vehicle.SOTA.BatteryEfficiency; A(12,11) = 1;
      
    Vehicle.Propulsion.PowerFlow.A = A;
    Vehicle.Propulsion.PowerFlow.A_inv = inv(A);
    
    Vehicle.Propulsion.PowerFlow.B = zeros(12,1);
    
end



%% Hybrid Electric Architecture (HE)
if Architecture == 2
    NumFuelZones = 4;
    Vehicle.Propulsion.FuelZones = 3:6;

    NumBattZones = 1;
    Vehicle.Propulsion.BattZones = 1;
    
    Vehicle.Propulsion.NTurbogenerators = 2;
    Vehicle.MassProp.EnergyMass_kg = [1 1];
    Vehicle.Propulsion.Neng = 2;                                            % number of internal combustion engines
    Vehicle.Propulsion.Turboshaft_Each_kW = .1;

    Vehicle.Propulsion.Ntank = 2;                                           % number of fuel tanks
    Vehicle.Propulsion.Qtot = 1;                                        % total volume of fuel, USG
    Vehicle.Propulsion.Qint = 1;                                            % total volume of fuel in integral tanks, USG

    Vehicle.Propulsion.GeneratorPower_Each_kW = 01;                         % total generator power rating, kW
    Vehicle.Propulsion.NBatteryPacks = 5;    


    % set up A initially as a 9 x 9 diagonal matrix
    A = eye(9);

    % set A to the correct size 16 x 16
    A(17,17) = 0;

    % row 10:
    A(10,1:9) = 1; A(10,12:13) = -Vehicle.SOTA.MotorEfficiency;

    % row 11:
    A(11,10) = Vehicle.Propulsion.NBatteryPacks; A(11,12) = -1; 

    % row 12:
    A(12,10) = Vehicle.Propulsion.NBatteryPacks/Vehicle.SOTA.BatteryEfficiency; A(12,11) = 1; A(12,17) = -Vehicle.SOTA.BatteryEfficiency;

    % row 13
    A(13,14) = Vehicle.SOTA.GearboxEfficiency; A(13,15) = -1;
    
    % row 14
    A(14,15) = Vehicle.SOTA.GeneratorEfficiency; A(14,16) = -1;

    % row 15
    A(15,13) = -1; A(15,16) = Vehicle.Propulsion.NTurbogenerators; A(15,17) = -1;

    % row 16 and 17 are for closure relationships


    Vehicle.Propulsion.PowerFlow.A_basic = A;

    %%%%% CRUISE NOMINAL MODE
    A_cn = A;

    % closure relationship 1, row 16 --> no power on powerpath 1
    A_cn(16,12) = 1; 

    % closure relationship 2, row 17 --> no recharge power
    A_cn(17,17) = 1;

    Vehicle.Propulsion.PowerFlow.A_cn = A_cn;
    Vehicle.Propulsion.PowerFlow.A_cn_inv = inv(A_cn);

    %%%%% RECHARGE MODE
    A_rc = A;

    % closure relationship 1, row 16 --> no power on powerpath 1
    A_rc(16,12) = 1; 

    % closure relationship 2, row 17 --> specified recharge rate
    A_rc(17,17) = 1;        % need to set b(17) = recharge rate in AeroPropPerf.m
    
    Vehicle.Propulsion.PowerFlow.A_rc = A_rc;
    Vehicle.Propulsion.PowerFlow.A_rc_inv = inv(A_rc);

    %%%%% OFFSET MODE, specified turbogenerator power
    A_os = A;

    % closure relationship 1, row 16 --> turboshaft at specified power
    A_os(16,14) = 1;        % need to set b(16) = turboshaft power setting in AeroPropPerf.m

    % closure relationship 2, row 17 --> no recharge power
    A_os(17,17) = 1;

    Vehicle.Propulsion.PowerFlow.A_os = A_os;
    Vehicle.Propulsion.PowerFlow.A_os_inv = inv(A_os);    

    Vehicle.Propulsion.PowerFlow.B = zeros(17,1);

end




%% Turbo Electric Architecture (TE)
if Architecture == 3
    NumBattZones = 1; % will be zero mass, but needs to be specified as at least 1
    Vehicle.Propulsion.BattZones = 1;
    
    NumFuelZones = 4;
    Vehicle.Propulsion.FuelZones = 3:6;
    
    Vehicle.MassProp.EnergyMass_kg = [1 0];
    Vehicle.Propulsion.NTurbogenerators = 3;
    Vehicle.Propulsion.Neng = Vehicle.Propulsion.NTurbogenerators;
    Vehicle.Propulsion.Ntank = 2;                                           % number of fuel tanks
    Vehicle.Propulsion.Qtot = 1;                                           % total volume of fuel, USG
    Vehicle.Propulsion.Qint = 1;                                           % total volume of fuel in integral tanks, USG    


    % first set up a 9 x 9 matrix of ones
    A = eye(9);
    
    % change A to the correct size
    A(13,13) = 0;
    
    % row 10:
    A(10,1:9) = 1; A(10,13) = -Vehicle.SOTA.MotorEfficiency;
    
    % row 11:
    A(11,10) = Vehicle.SOTA.GearboxEfficiency; A(11,11) = -1;
    
    % row 12:
    A(12,11) = Vehicle.SOTA.GeneratorEfficiency; A(12,12) = -1;
    
    % row 13:
    A(13,12) = Vehicle.Propulsion.NTurbogenerators; A(13,13) = -1;
    
    
    Vehicle.Propulsion.PowerFlow.A = A;
    Vehicle.Propulsion.PowerFlow.A_inv = inv(A);
    Vehicle.Propulsion.PowerFlow.B = zeros(13,1);    
    
end



%% Mass Prop/Weight Build Up script operational parameters
Vehicle.MassProp.EW_Margin = 0.02;                                          % Empty Weight margin for Weight build up script
% Vehicle.MassProp.GWTolerance = 0.1;                                         % Gross Weight tolerance for weight build up script
% Vehicle.MassProp.Convergence = 0;                                           % Weight Build Up Script Convergence Flag
%% Table of Contents
% Structures
[Vehicle.Geom.Fus] = GeomEval_InitFuselage('Fuselage');
[Vehicle.Geom.LWing] = GeomEval_InitLiftingSurface('Left Wing');
[Vehicle.Geom.RWing] = GeomEval_InitLiftingSurface('Right Wing');
[Vehicle.Geom.LHTail] = GeomEval_InitLiftingSurface('Left Horizontal Tail');
[Vehicle.Geom.RHTail] = GeomEval_InitLiftingSurface('Right Horizontal Tail');
[Vehicle.Geom.Left_VS] = GeomEval_InitLiftingSurface('Left VS');
[Vehicle.Geom.Right_VS] = GeomEval_InitLiftingSurface('Right VS');
[Vehicle.Geom.L_OBD_Boom] = GeomEval_InitFuselage('Left Outboard Boom');
[Vehicle.Geom.L_IBD_Boom] = GeomEval_InitFuselage('Left Inboard Boom');
[Vehicle.Geom.R_IBD_Boom] = GeomEval_InitFuselage('Right Inboard Boom');
[Vehicle.Geom.R_OBD_Boom] = GeomEval_InitFuselage('Right Outboard Boom');

% [Vehicle.Geom.Landing_Gear] = GeomEval_InitGenericComponent('Landing Gear',3);

[Vehicle.Geom.LMG, Vehicle.Geom.LMG_WP] = GeomEval_InitLandingGear('LeftMainGear', 'LMG_WP');
[Vehicle.Geom.RMG, Vehicle.Geom.RMG_WP] = GeomEval_InitLandingGear('RightMainGear', 'RMG_WP');

[Vehicle.Geom.NG, Vehicle.Geom.NG_WP] = GeomEval_InitLandingGear('NoseGear', 'NG_WP');

% Propulsion
[Vehicle.Geom.Rotor_1] = GeomEval_InitDuctedFan('LiftProp_1');
[Vehicle.Geom.Rotor_2] = GeomEval_InitDuctedFan('LiftProp_2');
[Vehicle.Geom.Rotor_3] = GeomEval_InitDuctedFan('LiftProp_3');
[Vehicle.Geom.Rotor_4] = GeomEval_InitDuctedFan('LiftProp_4');
[Vehicle.Geom.Rotor_5] = GeomEval_InitDuctedFan('LiftProp_5');
[Vehicle.Geom.Rotor_6] = GeomEval_InitDuctedFan('LiftProp_6');
[Vehicle.Geom.Rotor_7] = GeomEval_InitDuctedFan('LiftProp_7');
[Vehicle.Geom.Rotor_8] = GeomEval_InitDuctedFan('LiftProp_8');

[Vehicle.Geom.CruiseProp] = GeomEval_InitDuctedFan('CruiseProp');

[Vehicle.Geom.LiftMotors] = GeomEval_InitGenericComponent('Lift Motors',8);
[Vehicle.Geom.CruiseMotor] = GeomEval_InitGenericComponent('Cruise Motor',1);

[Vehicle.Geom.Turboshaft] = GeomEval_InitGenericComponent('Turboshaft',Vehicle.Propulsion.NTurbogenerators);
[Vehicle.Geom.Gearbox] = GeomEval_InitGenericComponent('Gearbox',Vehicle.Propulsion.NTurbogenerators);
[Vehicle.Geom.Generator] = GeomEval_InitGenericComponent('Generator',Vehicle.Propulsion.NTurbogenerators);


[Vehicle.Geom.Power_Cabling] = GeomEval_InitGenericComponent('Power Cabling',1);
% [Vehicle.Geom.Electrical] = GeomEval_InitGenericComponent('Electrical',1);

% System
[Vehicle.Geom.Flight_Controls] = GeomEval_InitGenericComponent('Flight Controls',1);
% [Vehicle.Geom.Hydraulics] = GeomEval_InitGenericComponent('Hydraulics',1);
[Vehicle.Geom.Avionics] = GeomEval_InitGenericComponent('Avionics',1);
% [Vehicle.Geom.Furnishings] = GeomEval_InitGenericComponent('Furnishings',1);
[Vehicle.Geom.Fuel_System] = GeomEval_InitGenericComponent('Fuel System',1);
[Vehicle.Geom.All_Else_Empty] = GeomEval_InitGenericComponent('All-Else Empty',1);

% Energy
[Vehicle.Geom.Batteries] = GeomEval_InitGenericComponent('Batteries',NumBattZones);
[Vehicle.Geom.Fuel] = GeomEval_InitGenericComponent('Fuel',NumFuelZones);

% Payload
[Vehicle.Geom.Passengers] = GeomEval_InitGenericComponent('Passengers',4);
%% Begin Adding in Geometry
%% Set an initial guess mass
Vehicle.MassProp.MTOM_kg = 6;
Vehicle.MassProp.TargetCG = [0;0;0];

%% Resizing Components Calculations
% these parameters are needed by ResizingCalculations
Vehicle.Propulsion.NLiftProps = 8;                                              % number of lift props
Vehicle.Propulsion.NCruiseProps = 1;                                            % number of cruise props
Vehicle.Propulsion.GeneratorPower_Each_kW = 1;
Vehicle.Propulsion.CruisePropPower_kW = 0.4;
Vehicle.Propulsion.BatteryPackPeakPower_kW = 0;
Vehicle.Propulsion.CruisePropRPM = 6500;
load CruiseProp
CQ = Setup.PropCurves.GI_CQ_rpm(Vehicle.Propulsion.CruisePropRPM);
Vehicle.Propulsion.CruiseProp_CP = 2*pi*CQ ;

clear Setup

% ResizingCalculations;


%% Fuselage
Vehicle.Geom.Fus.Length = 0.8;  %0.8
Vehicle.Geom.Fus.MaxHeight = (6/12)/3.28;  %6''
Vehicle.Geom.Fus.MaxWidth = (6/12)/3.28;

Vehicle.Geom.Fus.RefPtLocation = [0,0,0];                                               % the coordinates of the ref point
Vehicle.Geom.Fus.eff_V = 1;
% line definition
Vehicle.Geom.Fus.LineDef = 'LPC_FuselageLines';

% cross section 1
Vehicle.Geom.Fus.CrossSections(1).Name = 'Nosecone';
Vehicle.Geom.Fus.CrossSections(1).Defn = 'CS_LPC_Fuselage';
Vehicle.Geom.Fus.CrossSections(1).FS = [0, 0.23];
Vehicle.Geom.Fus.CrossSections(1).Length = 0.1;

% cross section 2
Vehicle.Geom.Fus.CrossSections(2).Name = 'PilotCabin';
Vehicle.Geom.Fus.CrossSections(2).Defn = 'CS_LPC_Fuselage';
Vehicle.Geom.Fus.CrossSections(2).FS = [0.25, 0.50];
Vehicle.Geom.Fus.CrossSections(2).Length = 0.2;

% cross section 3
% If 2 PAX, then length = 1.22 m
% If 4 PAX, then length = 1.22 + 0.9144 m (3 ft) = 2.1344
Vehicle.Geom.Fus.CrossSections(3).Name = 'PAXCabin';
Vehicle.Geom.Fus.CrossSections(3).Defn = 'CS_LPC_Fuselage';
Vehicle.Geom.Fus.CrossSections(3).FS = [0.52, 0.78];
Vehicle.Geom.Fus.CrossSections(3).Length = 0.2;

% cross section 4
Vehicle.Geom.Fus.CrossSections(4).Name = 'Tailcone';
Vehicle.Geom.Fus.CrossSections(4).Defn = 'CS_LPC_Fuselage';
Vehicle.Geom.Fus.CrossSections(4).FS = [0.8, 1.00];
Vehicle.Geom.Fus.CrossSections(4).Length = 0.2;


% reference point is defined as center of Section 3
LNoseToSec3MidPoint = Vehicle.Geom.Fus.CrossSections(1).Length + Vehicle.Geom.Fus.CrossSections(2).Length + Vehicle.Geom.Fus.CrossSections(3).Length*0.5;
LTotal = Vehicle.Geom.Fus.CrossSections(1).Length + Vehicle.Geom.Fus.CrossSections(2).Length + Vehicle.Geom.Fus.CrossSections(3).Length + Vehicle.Geom.Fus.CrossSections(4).Length;
Vehicle.Geom.Fus.RefPt_FS = LNoseToSec3MidPoint/LTotal;                                                       % FS of fuselage coinciding with ref pt



Vehicle.Geom.Fus.Mass = 3;
Vehicle.Geom.Fus.ColorVec = 'kbrk';
Vehicle.Geom.Fus.CG_FS = [];

%% Right Wing
Vehicle.Geom.RWing.RefPtChordFrac = 0.5;
Vehicle.Geom.RWing.PlanformArea = 0.5/2;
Vehicle.Geom.RWing.AspectRatio = Vehicle.DesignPoint.WingAspectRatio/2;
Vehicle.Geom.RWing.Directionality = 1;
Vehicle.Geom.RWing.TaperDefn = Vehicle.DesignPoint.WingTaperRatio;
Vehicle.Geom.RWing.SweepDefn = [0,1;0.5,0.5;0,0];
Vehicle.Geom.RWing.t_min = [];
Vehicle.Geom.RWing.RootDihedral = 0;
Vehicle.Geom.RWing.AirfoilName = 'NACA2412airfoil';

Vehicle.Geom.RWing.ExposedEtas = [0.1,1];
Vehicle.Geom.RWing.RootIncidence = 3;

Vehicle.Geom.RWing.Controls(1).Name = 'IBD_Flaperon';
Vehicle.Geom.RWing.Controls(1).ChordFrac = [0.75, 1.0];
Vehicle.Geom.RWing.Controls(1).EtaFrac = [0.10, 0.3];

Vehicle.Geom.RWing.Controls(2).Name = 'MBD_Flaperon';
Vehicle.Geom.RWing.Controls(2).ChordFrac = [0.75, 1.0];
Vehicle.Geom.RWing.Controls(2).EtaFrac = [0.30, 0.6];

Vehicle.Geom.RWing.Controls(3).Name = 'OBD_Flaperon';
Vehicle.Geom.RWing.Controls(3).ChordFrac = [0.75, 1.0];
Vehicle.Geom.RWing.Controls(3).EtaFrac = [0.7, 0.97];
%% Left Wing
Vehicle.Geom.LWing = Vehicle.Geom.RWing;
Vehicle.Geom.LWing.Name = {'Left Wing'};
Vehicle.Geom.LWing.Directionality = -1;
%% Right Horizontal Stabilizer
Vehicle.Geom.RHTail.PlanformArea = 0.1/2;                           %Decreased proportional to the wing planform area decrement
Vehicle.Geom.RHTail.AspectRatio = Vehicle.DesignPoint.HTAspectRatio/2;
Vehicle.Geom.RHTail.Directionality = 1;
Vehicle.Geom.RHTail.TaperDefn = [];

Vehicle.Geom.RHTail.SweepDefn = [];
Vehicle.Geom.RHTail.RootDihedral = 0;
Vehicle.Geom.RHTail.AirfoilName = 'symmetricAF';

Vehicle.Geom.RHTail.EtaMounting = 1;  

Vehicle.Geom.RHTail.RotorEtas = [];
Vehicle.Geom.RHTail.RotorDiams = [];
Vehicle.Geom.RHTail.RefPtChordFrac = 0;
Vehicle.Geom.RHTail.Span = sqrt(Vehicle.Geom.RHTail.PlanformArea*Vehicle.Geom.RHTail.AspectRatio);
Vehicle.Geom.RHTail.RootIncidence = -3;

Vehicle.Geom.RHTail.Controls(1).Name = 'Elevator';
Vehicle.Geom.RHTail.Controls(1).ChordFrac = [0.6,1];
Vehicle.Geom.RHTail.Controls(1).EtaFrac = [0.1, 0.9];
%% Left Horizontal Stabilizer
Vehicle.Geom.LHTail = Vehicle.Geom.RHTail;
Vehicle.Geom.LHTail.Name = {'Left Horizontal Tail'};
Vehicle.Geom.LHTail.Directionality = -1;
%% Right Vertical Stabilizer
Vehicle.Geom.Right_VS.RefPtChordFrac = 0;
Vehicle.Geom.Right_VS.StripMaxWidth = 0.1; 

Vehicle.Geom.Right_VS.Dihedral = 90;
Vehicle.Geom.Right_VS.TaperDefn = 0.5;
Vehicle.Geom.Right_VS.PlanformArea = 0.7432 * 0.19;
Vehicle.Geom.Right_VS.AspectRatio = Vehicle.DesignPoint.VTAspectRatio;
Vehicle.Geom.Right_VS.RootIncidence = 0;
Vehicle.Geom.Right_VS.SweepDefn = [0,1; 1,1; 00, 0];

Vehicle.Geom.Right_VS.Controls(1).Name = 'Rudder';
Vehicle.Geom.Right_VS.Controls(1).ChordFrac = [0.7, 1.0];
Vehicle.Geom.Right_VS.Controls(1).EtaFrac = [0.2, 0.8];
%% Left Vertical Stabilizer
Vehicle.Geom.Left_VS = Vehicle.Geom.Right_VS;
Vehicle.Geom.Left_VS.Name = {'Left VS'};
Vehicle.Geom.Left_VS.Directionality = 1;



%% Left Main Gear
Vehicle.Geom.LMG.PlanformArea = .3*0.19;                    %Scaling factor on the basis of wings
Vehicle.Geom.LMG.AspectRatio = 8;
Vehicle.Geom.LMG.Directionality = -1;
Vehicle.Geom.LMG.TaperDefn = 1;
Vehicle.Geom.LMG.SweepDefn = [0,1;0.5,0.5;0,0];
Vehicle.Geom.LMG.t_min = 0.04;
Vehicle.Geom.LMG.RootDihedral = -45;
Vehicle.Geom.LMG.AirfoilName = 'symmetricAF';

Vehicle.Geom.LMG.RefPtChordFrac = 0.25;
Vehicle.Geom.LMG.ExposedEtas = [0.1,1];
Vehicle.Geom.LMG.RootIncidence = 0;
Vehicle.Geom.LMG.HasWheelPant = 1;
Vehicle.Geom.LMG.LineDef = 'Generic_WheelPant_Lines';

%% Right Main Gear
Vehicle.Geom.RMG = Vehicle.Geom.LMG;
Vehicle.Geom.RMG.Name = {'RightMainGear'};
Vehicle.Geom.RMG.RootDihedral = -45;
Vehicle.Geom.RMG.Directionality = 1;
Vehicle.Geom.RMG.HasWheelPant = 1;
Vehicle.Geom.RMG.LineDef = 'Generic_WheelPant_Lines';

%% Nose Gear
Vehicle.Geom.NG = Vehicle.Geom.LMG;
Vehicle.Geom.NG.Name = {'NoseGear'};
Vehicle.Geom.NG.RootDihedral = -90;
Vehicle.Geom.NG.Directionality = 1;
Vehicle.Geom.NG.HasWheelPant = 1;
Vehicle.Geom.NG.LineDef = 'Generic_WheelPant_Lines';


%% Rotor 1
Vehicle.Geom.Rotor_1.Type = 2;

Vehicle.Geom.Rotor_1.Length = .001;
Vehicle.Geom.Rotor_1.Diam = 0.305;          %12'' rotor diameter
Vehicle.Geom.Rotor_1.RefPt_FS = 0.50;

Vehicle.Geom.Rotor_1.Theta = 0;
Vehicle.Geom.Rotor_1.Phi = 0;

Vehicle.Geom.Rotor_1.Directionality = 1;
Vehicle.Geom.Rotor_1.TaperDefn = 1;

Vehicle.Geom.Rotor_1.MaxDistBetweenStations = 0.5;
%% Rotor 2
Vehicle.Geom.Rotor_2 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_2.Name = {'LiftProp_2'}; 
%% Rotor 3
Vehicle.Geom.Rotor_3 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_3.Name = {'LiftProp_3'}; 
%% Rotor 4
Vehicle.Geom.Rotor_4 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_4.Name = {'LiftProp_4'}; 
%% Rotor 5
Vehicle.Geom.Rotor_5 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_5.Name = {'LiftProp_5'}; 
%% Rotor 6
Vehicle.Geom.Rotor_6 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_6.Name = {'LiftProp_6'}; 
%% Rotor 7
Vehicle.Geom.Rotor_7 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_7.Name = {'LiftProp_7'}; 
%% Rotor 8
Vehicle.Geom.Rotor_8 = Vehicle.Geom.Rotor_1;
Vehicle.Geom.Rotor_8.Name = {'LiftProp_8'};
%% Prop 1
Vehicle.Geom.CruiseProp.Type = 2;
Vehicle.Geom.CruiseProp.SpinDir = -1;
Vehicle.Geom.CruiseProp.Length = .001;
Vehicle.Geom.CruiseProp.Diam = 0.457;  %18'' prop diameter
Vehicle.Geom.CruiseProp.RefPt_FS = 0.50;

Vehicle.Geom.CruiseProp.Theta = -90;
Vehicle.Geom.CruiseProp.Psi = 0;

Vehicle.Geom.CruiseProp.Directionality = 1;
Vehicle.Geom.CruiseProp.TaperDefn = 1;

Vehicle.Geom.CruiseProp.MaxDistBetweenStations = 0.5;
%% Right Inboard Boom
% Vehicle.Geom.R_IBD_Boom.Length = 6.6166;
Vehicle.Geom.R_IBD_Boom.MaxHeight = 0.2 * 0.19;         %Scaling
Vehicle.Geom.R_IBD_Boom.MaxWidth = 0.2 * (18/13) * 0.19;    %Scaling

Vehicle.Geom.R_IBD_Boom.RefPt_FS = 0.5;                                                       % FS of fuselage coinciding with ref pt
Vehicle.Geom.R_IBD_Boom.RefPtLocation = [0,0,0];                                               % the coordinates of the ref point
Vehicle.Geom.R_IBD_Boom.eff_V = 1;
% line definition
Vehicle.Geom.R_IBD_Boom.LineDef = 'LPC_BoomLines';

% cross section 1
Vehicle.Geom.R_IBD_Boom.CrossSections(1).Name = 'NoseCap';
Vehicle.Geom.R_IBD_Boom.CrossSections(1).Defn = 'CS_rect';
Vehicle.Geom.R_IBD_Boom.CrossSections(1).FS = [0, 0.3];
Vehicle.Geom.R_IBD_Boom.CrossSections(1).Length = 0.1;

Vehicle.Geom.R_IBD_Boom.CrossSections(2).Name = 'ConstSect';
Vehicle.Geom.R_IBD_Boom.CrossSections(2).Defn = 'CS_rect';
Vehicle.Geom.R_IBD_Boom.CrossSections(2).FS = [0.31, 0.69];
Vehicle.Geom.R_IBD_Boom.CrossSections(2).Length = 6;

Vehicle.Geom.R_IBD_Boom.CrossSections(3).Name = 'EndCap';
Vehicle.Geom.R_IBD_Boom.CrossSections(3).Defn = 'CS_rect';
Vehicle.Geom.R_IBD_Boom.CrossSections(3).FS = [0.70, 1];
Vehicle.Geom.R_IBD_Boom.CrossSections(3).Length = 0.1;


Vehicle.Geom.R_IBD_Boom.Mass = 1;

Vehicle.Geom.R_IBD_Boom.CG_FS = [];
%% Left Inboard Boom
Vehicle.Geom.L_IBD_Boom = Vehicle.Geom.R_IBD_Boom;
Vehicle.Geom.L_IBD_Boom.Name = {'Left Inboard Boom'};
%% Right Outboard Boom
Vehicle.Geom.R_OBD_Boom = Vehicle.Geom.R_IBD_Boom;
Vehicle.Geom.R_OBD_Boom.MaxHeight = 0.3 * 0.19;
Vehicle.Geom.R_OBD_Boom.MaxWidth = 0.3 * (18/13) * 0.19;

Vehicle.Geom.R_OBD_Boom.RefPt_FS = 0.5; 
Vehicle.Geom.R_OBD_Boom.Name = {'Right Outboard Boom'};
%% Left Outboard Boom
Vehicle.Geom.L_OBD_Boom = Vehicle.Geom.R_OBD_Boom;
Vehicle.Geom.L_OBD_Boom.Name = {'Left Outboard Boom'};
%% Landing Gear
% Vehicle.Geom.Landing_Gear.Length = [.2 .2 .2];
% Vehicle.Geom.Landing_Gear.nMLDG = 2;                                        % Number of Main Landing Gear
% Vehicle.Geom.Landing_Gear.nNLDG = 1;                                        % Number of Nose Landing Gear
%% End of Geometry
%% Lift Electric Motors
%% Cruise Electric Motors
%% Power Cabling
%% Turboshaft
%% Generator
%% Electrical
%% Flight Controls
%% Hydraulics
%% Avionics
%% Furnishings
%% Fuel System
%% Fuel Tank
%% Batteries
Vehicle.Geom.Batteries.Mass = (0.1./length(Vehicle.Geom.Batteries.Mass))*ones(1,length(Vehicle.Geom.Batteries.Mass));                                          % [kg]


%% Passengers
%% All-Else Empty
%% End of Geometry







% Cruise Propellers
Vehicle.Propulsion.Cruise_K_W = 2.8;                                        % Blade Material Coefficient
% Vehicle.Propulsion.CruisePropDiam_m = 2;                                    % cruise propeller diameter, m
Vehicle.Propulsion.NbladesCruiseProp = 2;                                   % number of blades per cruise prop
Vehicle.Propulsion.CruiseBladeAF = 100;
Vehicle.Propulsion.CruisePropRPM = 5500;

% Rotor Propellers
Vehicle.Propulsion.Rotor_K_W = 1.2;                                         % Blade Material Coefficient
% Vehicle.Propulsion.RotorPropDiam_m = 1.4905;                                % cruise propeller diameter, m
% Vehicle.Propulsion.NbladesRotorProp = 3;                                    % number of blades per cruise prop
Vehicle.Propulsion.RotorBladeAF = 100;
Vehicle.Propulsion.RotorPropRPM = 6000;
% Vehicle.Propulsion.CablingkVAm = 1000;                                      % total kVA.m of propulsion related cabling

Vehicle.Propulsion.TLinterp = load('TLinterp.mat','TLinterp');              % Thrust Lapse Gridded Interpolant
Vehicle.Propulsion.RTinterp = load('RTinterp.mat','RTinterp');              % Residue Thrust Gridded Interpolant

% create the equivalent of the power lapse table in terms of density and
% not altitude
[~,~,~,rho] = atmosisa([0 100 5000 10000 15000 20000 25000 30000]/3.28);
Vehicle.Propulsion.TurboshaftLapse_Vkt_rho = Vehicle.Propulsion.TLinterp.TLinterp;


Vehicle.Propulsion.TurboshaftLapse_Vkt_rho.GridVectors{2} = fliplr(rho);
Vehicle.Propulsion.TurboshaftLapse_Vkt_rho.Values = fliplr(Vehicle.Propulsion.TurboshaftLapse_Vkt_rho.Values);


% End of Propulsion


%% Begin Adding in  System
Vehicle.Systems.W_uav = 25;                                % avionics weight, uninstalled, kg
%% End of System
%% Begin Adding in TTW Operations

% 150 kt = 173 mph
% 175 kt = 201 mph
% 190 kt = 219 mph

Vehicle.Operations.nult = 3.8*1.5;                             % ultimate load factor - FIXED
Vehicle.Operations.VC = Vcruise; %287;                               % design cruise speed, kt
Vehicle.Operations.VmaxSLkt = Vehicle.Operations.VC*1.05;  % max level flight speed at sea level, kt
Vehicle.Operations.VD = 1.1*Vehicle.Operations.VmaxSLkt;                           % design dive speed, kt
Vehicle.Operations.NPAX = 4;                               % number of passengers, including crew
Vehicle.Operations.OccupantMass_kg = 90;
Vehicle.Operations.MaxPayload = Vehicle.DesignPoint.Payload;
%% End of Operations


%% TEMP: parameters added 11/17/2021 to get FMComp to run
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% COMPLETE DUMP OF VEHICLE DEFINITION FROM MADCASP SETUP
Vehicle.Propulsion.I_cruiseprop = 2; % temporary

%%

% CLThreshold =  Vehicle.Aero.SI_CL_AOA_BETA_FLAP(AOAThreshold,0,0);

rhoSL = 1.225; % sea level air density, kg/m3
g = 9.81; % acceleration due to gravity, m/s2
CLtransition = 0.8;
VThreshold = sqrt((2/rhoSL) * (Vehicle.DesignPoint.WL_kgm2 * g) * (1/CLtransition));

Vehicle.Aero.VThreshold = VThreshold;

AOAThreshold = 12;
Vehicle.Aero.PitchSchedule_V = griddedInterpolant([0,0.9*VThreshold,VThreshold],[0,0,AOAThreshold],'linear','nearest');

%% LIFT ROTOR DEFINITION
% Load basic rotor setup
TempSetup=load('LiftProp.mat');
% load('LP_DataBase.mat');

% Vehicle.Propulsion.LPDesignGI_c_R = GI_c_R;
% Vehicle.Propulsion.LPDesignGI_beta_R = GI_beta_R;
% Vehicle.Propulsion.LPGIThrustScaler = GI_ThrustScaler_DL;
% Vehicle.Propulsion.LPGITorqueScaler = GI_TorqueScaler_DL;

Vehicle.Propulsion.LPSetup = TempSetup.Setup;

Vehicle.Propulsion.LPSetup.r_R = Vehicle.Propulsion.LPSetup.r/Vehicle.Propulsion.LPSetup.RotorRadius;
Vehicle.Propulsion.LPSetup.c_R = Vehicle.Propulsion.LPSetup.ChordDist/Vehicle.Propulsion.LPSetup.RotorRadius;


[Vehicle.Propulsion.LPSetup] = UpdateRotorSetup(Vehicle.Propulsion.LPSetup);

% update as required
Vehicle.Propulsion.LiftPropDiam_m = 2 * Vehicle.Propulsion.LPSetup.RotorRadius;

Vehicle.Propulsion.LPSetup.nRotor = 8;

% spin direction
Vehicle.Propulsion.LPSetup.SpinDir = [1, -1, +1, -1, 1, -1, 1, -1];


Vehicle.Geom.Rotor_1.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(1);
Vehicle.Geom.Rotor_2.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(2);
Vehicle.Geom.Rotor_3.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(3);
Vehicle.Geom.Rotor_4.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(4);
Vehicle.Geom.Rotor_5.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(5);
Vehicle.Geom.Rotor_6.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(6);
Vehicle.Geom.Rotor_7.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(7);
Vehicle.Geom.Rotor_8.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(8);


Vehicle.Propulsion.LPSetup.RotorAxisPhi = - 5  *Vehicle.Propulsion.LPSetup.SpinDir.*[1,1,1,1,-1,-1,-1,-1];






% make consistent with PEACE geometry definition
Vehicle.Geom.Rotor_1.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(1);
Vehicle.Geom.Rotor_2.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(2);
Vehicle.Geom.Rotor_3.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(3);
Vehicle.Geom.Rotor_4.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(4);
Vehicle.Geom.Rotor_5.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(5);
Vehicle.Geom.Rotor_6.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(6);
Vehicle.Geom.Rotor_7.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(7);
Vehicle.Geom.Rotor_8.Phi = Vehicle.Propulsion.LPSetup.RotorAxisPhi(8);


Vehicle.Propulsion.LPSetup.RotorAxisTheta = [0 0 0 0 0 0 0 0];
Vehicle.Propulsion.LPSetup.HealthStatus = [1 1 1 1 1 1 1 1];


% update rotor locations (3 x 6 matrix)
Vehicle.Propulsion.LPSetup.RotorLoc = [...
Vehicle.Geom.Rotor_1.RefPtLocation; Vehicle.Geom.Rotor_2.RefPtLocation; Vehicle.Geom.Rotor_3.RefPtLocation; Vehicle.Geom.Rotor_4.RefPtLocation;...
Vehicle.Geom.Rotor_5.RefPtLocation; Vehicle.Geom.Rotor_6.RefPtLocation; Vehicle.Geom.Rotor_7.RefPtLocation; Vehicle.Geom.Rotor_8.RefPtLocation]';

% pitch angles
Vehicle.Propulsion.LPSetup.RotorAxisTheta = 0 * ones(1,8);

% roll angles

% spin directions
% Vehicle.Propulsion.LPSetup.SpinDir = [1, -1, 1, -1, 1, -1, 1, -1];


Vehicle.Propulsion.LPSetup.XBlades_RA = Vehicle.Propulsion.LPSetup.dXBlades_RA + reshape(Vehicle.Propulsion.LPSetup.RotorLoc(1,:),[1,1,Vehicle.Propulsion.LPSetup.nRotor]);
Vehicle.Propulsion.LPSetup.YBlades_RA = Vehicle.Propulsion.LPSetup.dYBlades_RA + reshape(Vehicle.Propulsion.LPSetup.RotorLoc(2,:),[1,1,Vehicle.Propulsion.LPSetup.nRotor]);
Vehicle.Propulsion.LPSetup.ZBlades_RA = Vehicle.Propulsion.LPSetup.dZBlades_RA + + reshape(Vehicle.Propulsion.LPSetup.RotorLoc(3,:),[1,1,Vehicle.Propulsion.LPSetup.nRotor]);

Vehicle.Propulsion.I_liftprop = (20/2.2)*Vehicle.Propulsion.LPSetup.RotorRadius^2/3;


%% Resizing calculations
ResizingCalculations;

%% LOAD CRUISE PROPELLER
load CruiseProp
Vehicle.Propulsion.CPSetup = Setup;
Vehicle.Propulsion.CruisePropDiam_m = 0.3810;
% adjust parameters as needed
Vehicle.Propulsion.CPSetup.SpinDir = -1;
Vehicle.Propulsion.CPSetup.RotorRadius = 0.5 * Vehicle.Propulsion.CruisePropDiam_m;
Vehicle.Propulsion.CPSetup.RotorAxisTheta = -90;


Vehicle.Propulsion.CPSetup.r_R = Vehicle.Propulsion.CPSetup.r/Vehicle.Propulsion.CPSetup.RotorRadius;
Vehicle.Propulsion.CPSetup.c_R = Vehicle.Propulsion.CPSetup.ChordDist/Vehicle.Propulsion.CPSetup.RotorRadius;

% update rotor setup
[Vehicle.Propulsion.CPSetup] = UpdateRotorSetup(Vehicle.Propulsion.CPSetup);


clear Setup









%% INITIALIZE AERODYNAMIC DATABASE
Vehicle = LPC_AERODECK(Vehicle);
% % Inclusion of Crud Factor (K1) and Induced Drag Factor (K2)
Vehicle.Aero.KCD0 = 1.0;
Vehicle.Aero.KCDi = 1.0;











%% ENERGY ZONE DEFINITIONS
ctr = 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'NoseCone'};
EnergyZones.Parent(ctr,1) = {'Fus'};
EnergyZones.Limits(ctr,:) = [0,0.22];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'TailCone'};
EnergyZones.Parent(ctr,1) = {'Fus'};
EnergyZones.Limits(ctr,:) = [0,0.22];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;


EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'LWingIB'};
EnergyZones.Parent(ctr,1) = {'LWing'};
EnergyZones.Limits(ctr,:) = [0,1];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'RWingIB'};
EnergyZones.Parent(ctr,1) = {'RWing'};
EnergyZones.Limits(ctr,:) = [0,1];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;


EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'LWingMB'};
EnergyZones.Parent(ctr,1) = {'LWing'};
EnergyZones.Limits(ctr,:) = [0,1];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;


EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'RWingMB'};
EnergyZones.Parent(ctr,1) = {'RWing'};
EnergyZones.Limits(ctr,:) = [0,1];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;

Vehicle.Propulsion.EnergyZones = struct2table(EnergyZones);

ctr = ctr + 1;


