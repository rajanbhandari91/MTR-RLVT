function Vehicle = VehicleDefinition_Baseline(Vehicle,Architecture,FF)

global Settings
Settings = Vehicle.Settings;

DL = FF.DL;
WL = FF.WL;
Vcruise = FF.Vcruise;

Vehicle.Architecture = Architecture;
Vehicle.MassProp.EnergyMass_kg = [1 1];
Vehicle.Propulsion.GearboxPower_Each_kW = 5;


%% Design Point Specification
Vehicle.DesignPoint.WL_kgm2 = [WL*4.88243];                                 % Wing Loading [kg/m2] [note: 1 kg/m2 = 4.88243 lb/ft2]
Vehicle.DesignPoint.DL_kgm2 = [DL*4.88243];                                 % Disc Loading [kg/m2] [note: 1 kg/m2 = 4.88243 lb/ft2]
Vehicle.DesignPoint.DL_Requested = [DL*4.88243];

% Vehicle.DesignPoint.DL_kgm2 = 150;

Vehicle.DesignPoint.WingAspectRatio = FF.AR;                                 % Wing Aspect Ratio
Vehicle.DesignPoint.WingTaperRatio = FF.TR;
Vehicle.DesignPoint.Cruise_PMR_kWkg = 0.20;                                 % Cruise motor installed power / vehicle mass
Vehicle.DesignPoint.Lift_PMR_kWkg = 0.40;                                   % Lift motor installed power / vehicle mass
Vehicle.DesignPoint.Payload = 545;                                          % kg / 1200 lbs
Vehicle.DesignPoint.DesignMach = 0.25;

% Vehicle.DesignPoint.CanardVolumeRatio = 0.4;                                % canard volume ratio
% Vehicle.DesignPoint.CanardAspectRatio = 11;                                 % canard aspect ratio

Vehicle.DesignPoint.HTVolumeRatio = 1.1;                                  % horizontal tail volume ratio
Vehicle.DesignPoint.HTAspectRatio = 3.92;                                  % horizontal tail aspect ratio

Vehicle.DesignPoint.VTAspectRatio = 1.5;                                   % vertical tail aspect ratio
Vehicle.DesignPoint.VTVolumeRatio = 0.32;        


%% Technological states of the art
Updates = [];
Updates.KF_PackMass = 0.8 * 0.77;                     % pack mass knockdown factor
Updates.StageOfLife = 1;                        % 0: brand new, 1: end of life
Updates.BusVoltage = 800;                       % volts
Updates.K_BattSpecEnergy = 1.0;                   % improvement factor for specific energy

[Vehicle] = TechSOTADefinition(Vehicle,Updates);

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
    Vehicle.Propulsion.CruisePropPower_kW = 800;

    Vehicle.Propulsion.Ntank = 2;                                           % number of fuel tanks
    Vehicle.Propulsion.Qtot = 1/6.2;                                        % total volume of fuel, USG
    Vehicle.Propulsion.Qint = 0;                                            % total volume of fuel in integral tanks, USG

    Vehicle.Propulsion.GeneratorPower_Each_kW = 01;                         % total generator power rating, kW
    Vehicle.Propulsion.NBatteryPacks = FF.NBattery;

    % Power flow A-matrix (size 8 x 8)

    % first 6 x 6 block is a diagonal matrix of ones
    A = eye(6);

    % change A to the correct size
    A(7,1:6) = 1;
    A(7,8) = -Vehicle.Propulsion.NBatteryPacks*Vehicle.SOTA.MotorEfficiency;

    % row 8
    A(8,8) = -1;
    A(8,7) = Vehicle.Propulsion.NBatteryPacks;


    Vehicle.Propulsion.PowerFlow.A = A;
    Vehicle.Propulsion.PowerFlow.A_inv = inv(A);

    Vehicle.Propulsion.PowerFlow.B = zeros(8,1);

    Vehicle.Propulsion.Battery.N_series = 1;
    Vehicle.Propulsion.Battery.N_parallel = 1;

end



%% Turbo Electric Architecture (TE)
if Architecture == 3
    NumFuelZones = 2;
    Vehicle.Propulsion.FuelZones = 3:6;

    NumBattZones = 1;
    Vehicle.Propulsion.BattZones = 1;

    Vehicle.Propulsion.NTurbogenerators = 1;
    Vehicle.MassProp.EnergyMass_kg = [1 1];
    Vehicle.Propulsion.Neng = 1;                                            % number of internal combustion engines
    Vehicle.Propulsion.Turboshaft_Each_kW = .1;
    Vehicle.Propulsion.CruisePropPower_kW = 800;

    Vehicle.Propulsion.Ntank = 2;                                           % number of fuel tanks
    Vehicle.Propulsion.Qtot = 1;                                            % total volume of fuel, USG
    Vehicle.Propulsion.Qint = 1;                                            % total volume of fuel in integral tanks, USG

    Vehicle.Propulsion.GeneratorPower_Each_kW = 01;                         % total generator power rating, kW
    Vehicle.Propulsion.NBatteryPacks = 1;


    % set up A initially as a 6 x 6 diagonal matrix
    A = eye(6);

    % set A to the correct size 13 x 13
    A(13,13) = 0;

    % row 7:
    A(7,1:6) = 1; A(7,8:9) = -Vehicle.SOTA.MotorEfficiency*Vehicle.SOTA.MotorEfficiency;

    % row 8:
    A(8,7) = Vehicle.Propulsion.NBatteryPacks; A(8,8) = -1;

    % row 9:
    A(9,10) = Vehicle.SOTA.GearboxEfficiency; A(9,11) = -1; 

    % row 10
    A(10,11) = Vehicle.SOTA.GeneratorEfficiency; A(10,12) = -1;

    % row 11
    A(11,9) = -1; A(11,12) = Vehicle.Propulsion.NTurbogenerators; A(11,13) = -1;

    % row 12 and 13 are for closure relationships


    Vehicle.Propulsion.PowerFlow.A_basic = A;

    %%%%% CRUISE NOMINAL MODE
    A_cn = A;

    % closure relationship 1, row 12 --> no power on powerpath 1
    A_cn(12,8) = 1;

    % closure relationship 2, row 13 --> no recharge power
    A_cn(13,13) = 1;

    Vehicle.Propulsion.PowerFlow.A_cn = A_cn;
    Vehicle.Propulsion.PowerFlow.A_cn_inv = inv(A_cn);

    %%%%% RECHARGE MODE
    A_rc = A;

    % closure relationship 1, row 12 --> no power on powerpath 1
    A_rc(12,8) = 1;

    % closure relationship 2, row 13 --> specified recharge rate
    A_rc(13,13) = 1;        % need to set b(13) = recharge rate in AeroPropPerf.m

    Vehicle.Propulsion.PowerFlow.A_rc = A_rc;
    Vehicle.Propulsion.PowerFlow.A_rc_inv = inv(A_rc);

    %%%%% OFFSET MODE, specified turbogenerator power
    A_os = A;

    % closure relationship 1, row 12 --> turboshaft at specified power
    A_os(12,10) = 1;        % need to set b(12) = turboshaft power setting in AeroPropPerf.m

    % closure relationship 2, row 13 --> no recharge power
    A_os(13,13) = 1;

    Vehicle.Propulsion.PowerFlow.A_os = A_os;
    Vehicle.Propulsion.PowerFlow.A_os_inv = inv(A_os);

    Vehicle.Propulsion.PowerFlow.B = zeros(13,1);

    Vehicle.Propulsion.Battery.N_series = 1;
    Vehicle.Propulsion.Battery.N_parallel = 1;

end



%% Mass Prop/Weight Build Up script operational parameters
Vehicle.MassProp.EW_Margin = 0.02;                                          % Empty Weight margin for Weight build up script
% Vehicle.MassProp.GWTolerance = 0.1;                                         % Gross Weight tolerance for weight build up script
% Vehicle.MassProp.Convergence = 0;                                           % Weight Build Up Script Convergence Flag
%% Table of Contents

[Vehicle.Geom.Motors] = GeomEval_InitGenericComponent('Motors', 6);
[Vehicle.Geom.Power_Cabling] = GeomEval_InitGenericComponent('Power Cabling',1);
[Vehicle.Geom.Avionics] = GeomEval_InitGenericComponent('Avionics',1);
[Vehicle.Geom.Gearbox] = GeomEval_InitGenericComponent('Gearbox',1);
[Vehicle.Geom.Generator] = GeomEval_InitGenericComponent('Generator',1);
[Vehicle.Geom.Electrical] = GeomEval_InitGenericComponent('Electrical',1);
[Vehicle.Geom.Flight_Controls] = GeomEval_InitGenericComponent('Flight Controls',1);
[Vehicle.Geom.Hydraulics] = GeomEval_InitGenericComponent('Hydraulics',1);
[Vehicle.Geom.Avionics] = GeomEval_InitGenericComponent('Avionics',1);
[Vehicle.Geom.Furnishings] = GeomEval_InitGenericComponent('Furnishings',1);
[Vehicle.Geom.All_Else_Empty] = GeomEval_InitGenericComponent('All-Else Empty',1);

% Energy
[Vehicle.Geom.Batteries] = GeomEval_InitGenericComponent('Batteries',NumBattZones);
[Vehicle.Geom.Fuel] = GeomEval_InitGenericComponent('Fuel',NumFuelZones);

% Payload
[Vehicle.Geom.Passengers] = GeomEval_InitGenericComponent('Payload',6);
% 

%% Begin Adding in Geometry

[Vehicle.Geom.LWing] = GeomEval_InitLiftingSurface('Left Wing');
[Vehicle.Geom.RWing] = GeomEval_InitLiftingSurface('Right Wing');
[Vehicle.Geom.LHTail] = GeomEval_InitLiftingSurface('Left Horizontal Tail');
[Vehicle.Geom.RHTail] = GeomEval_InitLiftingSurface('Right Horizontal Tail');
[Vehicle.Geom.VTail] = GeomEval_InitLiftingSurface('Vertical Tail');

%% Set an initial guess mass
Vehicle.MassProp.MTOM_kg = Settings.MTOMGuess_kg;   %22 lb to kg
Vehicle.MassProp.TargetCG = [0;0;0];
%% Resizing Components Calculations
% these parameters are needed by ResizingCalculations
Vehicle.Propulsion.NLiftProps = 2;                                              % number of lift props
Vehicle.Propulsion.NCruiseProps = 4;                                            % number of cruise props
Vehicle.Propulsion.GeneratorPower_Each_kW = 1450;
Vehicle.Propulsion.CruisePropPower_kW = 500;
Vehicle.Propulsion.TurboshaftPower_Each_kW = 2700;
Vehicle.Propulsion.BatteryPackPeakPower_kW = 0;
Vehicle.Propulsion.LiftMotorPower_Each_kW = 120 ;
Vehicle.Propulsion.CruiseMotorPower_Each_kW = 120;

%% Parse the geometry parameters from OpenVSP
filename = Settings.VSPFileName;
Param = ParseOpenVSP(filename);
Vehicle.OpenVSP = Param;

%% Fuselage
FusFields = fieldnames(Param.Fus);
for i =  1: length(fieldnames(Param.Fus))
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name) = GeomEval_InitFuselage([Param.Fus.(FusFields{i}).Name]);
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).Length = Param.Fus.(FusFields{i}).Length;
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).MaxHeight = Param.Fus.(FusFields{i}).MaxHeight;
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).MaxWidth = Param.Fus.(FusFields{i}).MaxWidth;
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).FuselageLines = Param.Fus.(FusFields{i}).FUS;
FSTemp = [0 Param.Fus.(FusFields{i}).FS]; 

for j = 1 : length(Param.Fus.(FusFields{i}).FS)
    Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).Name = ['CrossSection_',num2str(j)];
    if j ~= length(Param.Fus.(FusFields{i}).FS)
        CS_fieldName = ['CS' num2str(j)];
        Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).CS = Param.Fus.(FusFields{i}).CS.(CS_fieldName);
    else
        % load CS_circ CS
        % Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).CS = CS;
        Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).CS = Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j-1).CS;
    end
    Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).FS = [FSTemp(j) , FSTemp(j+1)];
    if j > 1
        Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).FS = [FSTemp(j)+0.001 , FSTemp(j+1)];
    end
    Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections(j).Length = (FSTemp(j+1) - FSTemp(j))*Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).Length;
end

Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).RefPt_FS = 0.3; % (Vehicle.Geom.Fus.CrossSections(1).Length + Vehicle.Geom.Fus.CrossSections(2).Length...
%+ Vehicle.Geom.Fus.CrossSections(3).Length + 0.5*Vehicle.Geom.Fus.CrossSections(4).Length)/ Vehicle.Geom.Fus.Length;
if strcmp(Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).Name,'Fuselage') ~= 1
    Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).RefPt_FS = 0;
end

Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).RefPtLocationTemp = Param.Fus.(FusFields{i}).Location;
% Vehicle.Geom.Fus.FinenessRatio = Vehicle.Geom.Fus.Length/max(Vehicle.Geom.Fus.MaxHeight,Vehicle.Geom.Fus.MaxWidth);

% Rotation Tensor
phi = Param.Fus.(FusFields{i}).Rotation(1);
theta = -Param.Fus.(FusFields{i}).Rotation(2);
psi = Param.Fus.(FusFields{i}).Rotation(3);

% Evaluate the rotation tensor
R1 = [1 0 0;...
    0 cosd(phi) sind(phi);...
    0 -sind(phi) cosd(phi)];

R2 = [cosd(theta) 0 -sind(theta);...
    0 1 0;...
    sind(theta) 0 cosd(theta)];

R3 = [cosd(psi) sind(psi) 0;...
    -sind(psi) cosd(psi) 0;...
    0 0 1];

R = R1*R2*R3;

Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).R_BC = [R(1,1:3), R(2,1:3), R(3,1:3)];


Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).Mass = 1e-6;
% Make the color vector based on the number of crosssections
% Assuming you have the number of fuselage sections stored in a variable
numSections = length(Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CrossSections); % Replace with the actual number of sections
baseColors = 'kbrgmyc';

% Initialize the ColorVec
ColorVec = '';

% Create the color vector dynamically
if numSections <= length(baseColors)
    ColorVec = baseColors(1:numSections);
else
    % Repeat the base colors as needed
    ColorVec = repmat(baseColors, 1, ceil(numSections/length(baseColors)));
    ColorVec = ColorVec(1:numSections);  % Trim to the correct length
end

% Assign it to the vehicle geometry
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).ColorVec = ColorVec;
Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).CG_FS = [];
end

%% Lifting Surfaces
LSFields = fieldnames(Param.LS);
for i =  1: length(fieldnames(Param.LS))
    if startsWith(Param.LS.(LSFields{i}).Type, 'Wing')
        % Check if any name has keyword Wing
        if startsWith(Param.LS.(LSFields{i}).Name, 'Wing')
            Vehicle.Geom.RWing = GeomEval_InitLiftingSurface('Right Wing') ;
            Vehicle.Geom.RWing.RefPtChordFrac = 0;
            Vehicle.Geom.RWing.PlanformArea = Param.LS.(LSFields{i}).PlanformArea;
            Vehicle.Geom.RWing.AspectRatio = Param.LS.(LSFields{i}).AR;
            Vehicle.Geom.RWing.Directionality = Param.LS.(LSFields{i}).Directionality;
            Vehicle.Geom.RWing.TaperDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).TR];
            Vehicle.Geom.RWing.SweepDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).SweepChordLoc;Param.LS.(LSFields{i}).Sweep];
            Vehicle.Geom.RWing.t_min = [];
            Vehicle.Geom.RWing.Dihedral = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Dihedral];
            Vehicle.Geom.RWing.Twist = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Twist];

            if length(unique(Param.LS.(LSFields{i}).AFName)) > 1
                Vehicle.Geom.RWing.AirfoilName = cellfun(@(x) struct('name', x), Param.LS.(LSFields{i}).AFName, 'UniformOutput', false);
                eta = Param.LS.(LSFields{i}).eta;
                midpoints = (eta(1:end-1) + eta(2:end)) / 2;
                b = eta(2:end) - midpoints;
                blend_start = midpoints - 0.99*b;
                blend_end = midpoints + 0.99*b;
                Vehicle.Geom.RWing.AirfoilEta = [eta(1), reshape([blend_start; blend_end], 1, []), eta(end)];
            else
                Vehicle.Geom.RWing.AirfoilName{1}.name = Param.LS.(LSFields{i}).AFName{1};
                Vehicle.Geom.RWing.AirfoilEta = [0 1];
            end

            Vehicle.Geom.RWing.RefPtLocationTemp = Param.LS.(LSFields{i}).Transform.Location;
            Vehicle.Geom.RWing.RefPtChordFrac = Param.LS.(LSFields{i}).RefPtChordFrac;
            Vehicle.Geom.RWing.ExposedEtas = [0,1];
            Vehicle.Geom.RWing.RootIncidence = Param.LS.(LSFields{i}).RootIncidence;

            [~,ncolm] = size(Param.LS.(LSFields{i}).CS.eta);
            for k = 1 : ncolm
                Vehicle.Geom.RWing.Controls(k).Name = Param.LS.(LSFields{i}).CS.Name(k);
                Vehicle.Geom.RWing.Controls(k).ChordFrac = [1-Param.LS.(LSFields{i}).CS.Chordfrac(k),1];
                Vehicle.Geom.RWing.Controls(k).EtaFrac = [Param.LS.(LSFields{i}).CS.eta(1,k), Param.LS.(LSFields{i}).CS.eta(2,k)];
            end

            if Param.LS.(LSFields{i}).Transform.Symm == 2
                Vehicle.Geom.LWing = Vehicle.Geom.RWing;
                Vehicle.Geom.LWing.Name = {'Left Wing'};
                Vehicle.Geom.LWing.Directionality = -Vehicle.Geom.RWing.Directionality;
            end
        end

        % Check if any name has keyword Tail
        if startsWith(Param.LS.(LSFields{i}).Name, 'Tail')
            if any(Param.LS.(LSFields{i}).Dihedral ~= 90)
                Vehicle.Geom.RHTail = GeomEval_InitLiftingSurface('Right Horizontal Tail') ;
                Vehicle.Geom.RHTail.RefPtChordFrac = 0;
                Vehicle.Geom.RHTail.PlanformArea = Param.LS.(LSFields{i}).PlanformArea;
                Vehicle.Geom.RHTail.AspectRatio = Param.LS.(LSFields{i}).AR;
                Vehicle.Geom.RHTail.Directionality = Param.LS.(LSFields{i}).Directionality;
                Vehicle.Geom.RHTail.TaperDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).TR];
                Vehicle.Geom.RHTail.SweepDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).SweepChordLoc;Param.LS.(LSFields{i}).Sweep];
                Vehicle.Geom.RHTail.t_min = [];
                Vehicle.Geom.RHTail.Dihedral = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Dihedral];
                Vehicle.Geom.RHTail.Twist = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Twist];

                if length(unique(Param.LS.(LSFields{i}).AFName)) > 1
                    Vehicle.Geom.RHTail.AirfoilName = cellfun(@(x) struct('name', x), Param.LS.(LSFields{i}).AFName, 'UniformOutput', false);
                    eta = Param.LS.(LSFields{i}).eta;
                    midpoints = (eta(1:end-1) + eta(2:end)) / 2;
                    b = eta(2:end) - midpoints;
                    blend_start = midpoints - 0.99*b;
                    blend_end = midpoints + 0.99*b;
                    Vehicle.Geom.RHTail.AirfoilEta = [eta(1), reshape([blend_start; blend_end; eta(2:end)], 1, [])];
                else
                    Vehicle.Geom.RHTail.AirfoilName{1}.name = Param.LS.(LSFields{i}).AFName{1};
                    Vehicle.Geom.RHTail.AirfoilEta = [0 1];
                end

                Vehicle.Geom.RHTail.RefPtLocationTemp = Param.LS.(LSFields{i}).Transform.Location;
                Vehicle.Geom.RHTail.RefPtChordFrac = Param.LS.(LSFields{i}).RefPtChordFrac;
                Vehicle.Geom.RHTail.ExposedEtas = [0,1];
                Vehicle.Geom.RHTail.RootIncidence = Param.LS.(LSFields{i}).RootIncidence;

                [~,ncolm] = size(Param.LS.(LSFields{i}).CS.eta);
                for k = 1 : ncolm
                    Vehicle.Geom.RHTail.Controls(k).Name = Param.LS.(LSFields{i}).CS.Name(k);
                    Vehicle.Geom.RHTail.Controls(k).ChordFrac = [1-Param.LS.(LSFields{i}).CS.Chordfrac(k),1];
                    Vehicle.Geom.RHTail.Controls(k).EtaFrac = [Param.LS.(LSFields{i}).CS.eta(1,k), Param.LS.(LSFields{i}).CS.eta(2,k)];
                end

                if Param.LS.(LSFields{i}).Transform.Symm == 2
                    Vehicle.Geom.LHTail = Vehicle.Geom.RHTail;
                    Vehicle.Geom.LHTail.Name = {'Left Horizontal Tail'};
                    Vehicle.Geom.LHTail.Directionality = -Vehicle.Geom.RHTail.Directionality;
                end
            end

            if any(Param.LS.(LSFields{i}).Dihedral == 90)
                if Param.LS.(LSFields{i}).Transform.Symm == 0
                    Vehicle.Geom.VTail = GeomEval_InitLiftingSurface('Vertical Tail') ;
                    Vehicle.Geom.VTail.RefPtChordFrac = 0;
                    Vehicle.Geom.VTail.PlanformArea = Param.LS.(LSFields{i}).PlanformArea;
                    Vehicle.Geom.VTail.AspectRatio = Param.LS.(LSFields{i}).AR;
                    Vehicle.Geom.VTail.Directionality = Param.LS.(LSFields{i}).Directionality;
                    Vehicle.Geom.VTail.TaperDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).TR];
                    Vehicle.Geom.VTail.SweepDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).SweepChordLoc;Param.LS.(LSFields{i}).Sweep];
                    Vehicle.Geom.VTail.t_min = [];
                    Vehicle.Geom.VTail.Dihedral = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Dihedral];
                    Vehicle.Geom.VTail.Twist = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Twist];

                    if length(unique(Param.LS.(LSFields{i}).AFName)) > 1
                        Vehicle.Geom.VTail.AirfoilName = cellfun(@(x) struct('name', x), Param.LS.(LSFields{i}).AFName, 'UniformOutput', false);
                        eta = Param.LS.(LSFields{i}).eta;
                        midpoints = (eta(1:end-1) + eta(2:end)) / 2;
                        b = eta(2:end) - midpoints;
                        blend_start = midpoints - 0.99*b;
                        blend_end = midpoints + 0.99*b;
                        Vehicle.Geom.VTail.AirfoilEta = [eta(1), reshape([blend_start; blend_end; eta(2:end)], 1, [])];
                    else
                        Vehicle.Geom.VTail.AirfoilName{1}.name = Param.LS.(LSFields{i}).AFName{1};
                        Vehicle.Geom.VTail.AirfoilEta = [0 1];
                    end

                    Vehicle.Geom.VTail.RefPtLocationTemp = Param.LS.(LSFields{i}).Transform.Location;
                    Vehicle.Geom.VTail.RefPtChordFrac = Param.LS.(LSFields{i}).RefPtChordFrac;
                    Vehicle.Geom.VTail.ExposedEtas = [0,1];
                    Vehicle.Geom.VTail.RootIncidence = Param.LS.(LSFields{i}).RootIncidence;

                    [~,ncolm] = size(Param.LS.(LSFields{i}).CS.eta);
                    for k = 1 : ncolm
                        Vehicle.Geom.VTail.Controls(k).Name = Param.LS.(LSFields{i}).CS.Name(k);
                        Vehicle.Geom.VTail.Controls(k).ChordFrac = [1-Param.LS.(LSFields{i}).CS.Chordfrac(k),1];
                        Vehicle.Geom.VTail.Controls(k).EtaFrac = [Param.LS.(LSFields{i}).CS.eta(1,k), Param.LS.(LSFields{i}).CS.eta(2,k)];
                    end

                    if Param.LS.(LSFields{i}).Transform.Symm == 2
                        Vehicle.Geom.RVTail = Vehicle.Geom.VTail;
                        Vehicle.Geom.RVTail.Name = {'Left Vertical Tail'};

                        Vehicle.Geom.LVTail = Vehicle.Geom.RVTail;
                        Vehicle.Geom.LVTail.Name = {'Right Vertical Tail'};
                        Vehicle.Geom.LVTail.RefPtLocationTemp(2) = - Vehicle.Geom.RVTail.RefPtLocationTemp(2);
                    end
                end
            end
        end
        % Check if the name has 'LandingGear'
        if startsWith(Param.LS.(LSFields{i}).Name, 'Landing')
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name) = GeomEval_InitLiftingSurface([Param.LS.(LSFields{i}).Name]);
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Type = 'LandingGear';
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).RefPtChordFrac = 0;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).PlanformArea = Param.LS.(LSFields{i}).PlanformArea;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).AspectRatio = Param.LS.(LSFields{i}).AR;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Directionality = Param.LS.(LSFields{i}).Directionality;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).TaperDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).TR];
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).SweepDefn = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).SweepChordLoc;Param.LS.(LSFields{i}).Sweep];
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).t_min = [];
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Dihedral = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Dihedral];
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Twist = [Param.LS.(LSFields{i}).eta;Param.LS.(LSFields{i}).Twist];

            if length(unique(Param.LS.(LSFields{i}).AFName)) > 1
                Vehicle.Geom.(Param.LS.(LSFields{i}).Name).AirfoilName = cellfun(@(x) struct('name', x), Param.LS.(LSFields{i}).AFName, 'UniformOutput', false);
                eta = Param.LS.(LSFields{i}).eta;
                midpoints = (eta(1:end-1) + eta(2:end)) / 2;
                b = eta(2:end) - midpoints;
                blend_start = midpoints - 0.99*b;
                blend_end = midpoints + 0.99*b;
                Vehicle.Geom.(Param.LS.(LSFields{i}).Name).AirfoilEta = [eta(1), reshape([blend_start; blend_end; eta(2:end)], 1, [])];
            else
                Vehicle.Geom.(Param.LS.(LSFields{i}).Name).AirfoilName{1}.name = Param.LS.(LSFields{i}).AFName{1};
                Vehicle.Geom.(Param.LS.(LSFields{i}).Name).AirfoilEta = [0 1];
            end

            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).RefPtLocationTemp = Param.LS.(LSFields{i}).Transform.Location;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).RefPtChordFrac = Param.LS.(LSFields{i}).RefPtChordFrac;
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).ExposedEtas = [0,1];
            Vehicle.Geom.(Param.LS.(LSFields{i}).Name).RootIncidence = Param.LS.(LSFields{i}).RootIncidence;

            if Param.LS.(LSFields{i}).Transform.Symm == 2
                NewName = strcat(Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Name, '_1');
                OldField1 = strcat((Param.LS.(LSFields{i}).Name), '_1');
                Vehicle.Geom.(OldField1) = Vehicle.Geom.(Param.LS.(LSFields{i}).Name);
                Vehicle.Geom.(OldField1).Name = NewName;
                                
                NewName1 = strcat(Vehicle.Geom.(Param.LS.(LSFields{i}).Name).Name, '_2');
                OldField2 = strcat((Param.LS.(LSFields{i}).Name), '_2');
                Vehicle.Geom.(OldField2) = Vehicle.Geom.(Param.LS.(LSFields{i}).Name);
                Vehicle.Geom.(OldField2).Name = NewName1;
                Vehicle.Geom.(OldField2).RefPtLocationTemp(2) = -Vehicle.Geom.(OldField1).RefPtLocationTemp(2);
                Vehicle.Geom.(OldField2).Directionality = -Vehicle.Geom.(OldField1).Directionality;
                Vehicle.Geom = rmfield(Vehicle.Geom,Param.LS.(LSFields{i}).Name);
            end
        end
    end

    % Check if the type is 'Wheel'
    if startsWith(Param.LS.(LSFields{i}).Type, 'Wheel')
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name) = GeomEval_InitLiftingSurface([Param.LS.(LSFields{i}).Wheel.Name]);
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Type = 'Wheel';
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Directionality = 1;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).TaperDefn = 1;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).SweepDefn = [0,1;0.5,0.5;0,0];
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).t_min = [];
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).RootDihedral = 0;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).AirfoilName{1}.name= 'LG_Circ_CS';
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).AirfoilEta = [0 1];
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Dihedral = 0;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Twist = 0;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).RefPtChordFrac = 0.5;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).StripMaxWidth = 0.01;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Span = Param.LS.(LSFields{i}).Wheel.Span;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).RefPtLocationTemp = Param.LS.(LSFields{i}).Wheel.Transform.Location;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).AspectRatio = Param.LS.(LSFields{i}).Wheel.Span / Param.LS.(LSFields{i}).Wheel.Chord;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).PlanformArea =  Param.LS.(LSFields{i}).Wheel.Span^2 / Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).AspectRatio;    
        
        if Param.LS.(LSFields{i}).Wheel.Transform.Symm == 2
            NewName = strcat(Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Name, '_1');
            OldField1 = strcat((Param.LS.(LSFields{i}).Wheel.Name), '_1');
            Vehicle.Geom.(OldField1) = Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name);
            Vehicle.Geom.(OldField1).Name = NewName;

            NewName1 = strcat(Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name).Name, '_2');
            OldField2 = strcat((Param.LS.(LSFields{i}).Wheel.Name), '_2');
            Vehicle.Geom.(OldField2) = Vehicle.Geom.(Param.LS.(LSFields{i}).Wheel.Name);
            Vehicle.Geom.(OldField2).Name = NewName1;
            Vehicle.Geom.(OldField2).RefPtLocationTemp(2) = -Vehicle.Geom.(OldField1).RefPtLocationTemp(2);
            Vehicle.Geom.(OldField2).Directionality = -Vehicle.Geom.(OldField1).Directionality;

            Vehicle.Geom = rmfield(Vehicle.Geom,Param.LS.(LSFields{i}).Wheel.Name);
        end
    
    end

     % Check if the type if 'Propeller'
    if startsWith(Param.LS.(LSFields{i}).Type,'Propeller')
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name) = GeomEval_InitDuctedFan(Param.LS.(LSFields{i}).Prop.Name);
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).Type = 2;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).Diam = Param.LS.(LSFields{i}).Prop.Diameter;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).Theta =  Param.LS.(LSFields{i}).Prop.theta;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).Phi = Param.LS.(LSFields{i}).Prop.phi;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).Directionality = 1;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).TaperDefn = 1;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).RefPtLocationTemp = Param.LS.(LSFields{i}).Prop.Transform.Location;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).MaxDistBetweenStations = 0.5;
        Vehicle.Geom.(Param.LS.(LSFields{i}).Prop.Name).NBlades = Param.LS.(LSFields{i}).Prop.NBlades;
    end
end




%% Load parameters for empirical aerodyanamics calculation
Vehicle = EMPAERO_GenAirfoilGeomGI(Vehicle);
Vehicle = EMPAERO_SetSurfaceRoughness(Vehicle);

%% Batteries
Vehicle.Geom.Batteries.Mass = (0.1./length(Vehicle.Geom.Batteries.Mass))*ones(1,length(Vehicle.Geom.Batteries.Mass));                                          % [kg]


%% Passengers
%% All-Else Empty
%% End of Geometry







% Cruise Propellers
Vehicle.Propulsion.Cruise_K_W = 215;                                        % Blade Material Coefficient
% Vehicle.Propulsion.CruisePropDiam_m = 2;                                    % cruise propeller diameter, m
Vehicle.Propulsion.NbladesCruiseProps = 8;                                   % number of blades per cruise prop
Vehicle.Propulsion.CruiseBladeAF = 100;
Vehicle.Propulsion.CruisePropRPM = 5500;


% Rotor Propellers
Vehicle.Propulsion.Rotor_K_W = 215;                                         % Blade Material Coefficient
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
Vehicle.Operations.VmaxSLkt = Vehicle.Operations.VC*1.13;  % max level flight speed at sea level, kt
Vehicle.Operations.VD = 1.1*Vehicle.Operations.VmaxSLkt;                           % design dive speed, kt
Vehicle.Operations.NPAX = 4;                               % number of passengers, including crew
Vehicle.Operations.OccupantMass_kg = 90;
Vehicle.Operations.MaxPayload = Vehicle.DesignPoint.Payload;

%% TEMP: parameters added 11/17/2021 to get FMComp to run
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% COMPLETE DUMP OF VEHICLE DEFINITION FROM MADCASP SETUP
Vehicle.Propulsion.I_cruiseprop = 2; % temporary

%%
rhoSL = 1.225; % sea level air density, kg/m3
g = 9.81; % acceleration due to gravity, m/s2
CLtransition = 0.95;
VThreshold = sqrt((2/rhoSL) * (Vehicle.DesignPoint.WL_kgm2 * g) * (1/CLtransition));

Vehicle.Aero.VThreshold = VThreshold;

AOAThreshold = 1.5;
Vehicle.Aero.PitchSchedule_V = griddedInterpolant([0,0.9*VThreshold,VThreshold],[0,0,AOAThreshold],'linear','nearest');

%% Get the prop setup requirements
% Read the setup file
Settings.FileName = 'PropRequirementsSetup_LP.xlsx';
Settings.nRotor = 2;
[Cases, PropDef, ~, ~] = ReadSetupFile(Settings.FileName);
Vehicle.Propulsion.LPDef = PropDef;
Vehicle.Propulsion.LPCases = Cases;

% Read the setup file for CP
Settings.FileName = 'PropRequirementsSetup_CP.xlsx';
Settings.nRotor = 4;
[Cases, PropDef, ~, ~] = ReadSetupFile(Settings.FileName);
Vehicle.Propulsion.CPDef = PropDef;
Vehicle.Propulsion.CPCases = Cases;




ResizingCalculations;







%% INITIALIZE AERODYNAMIC DATABASE
Vehicle = MTR_AERODECK(Vehicle);


%% ENERGY ZONE DEFINITIONS
ctr = 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'EnergyZone_1'};
EnergyZones.Parent(ctr,1) = {'Fuselage'};
EnergyZones.Limits(ctr,:) = [0,0.22];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'EnergyZone_2'};
EnergyZones.Parent(ctr,1) = {'Fuselage'};
EnergyZones.Limits(ctr,:) = [0,0.22];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;
ctr = ctr + 1;

EnergyZones.Index(ctr,1) = ctr;
EnergyZones.Name(ctr,1) = {'EnergyZone_3'};
EnergyZones.Parent(ctr,1) = {'Fuselage'};
EnergyZones.Limits(ctr,:) = [0,0.22];
EnergyZones.Efficiency(ctr,1) = 0.8;
EnergyZones.Centroid(ctr,:) = [0,0,0];
EnergyZones.GeomVol(ctr,1) = 0;
EnergyZones.EffVol(ctr,1) = 0;



Vehicle.Propulsion.EnergyZones = struct2table(EnergyZones);




