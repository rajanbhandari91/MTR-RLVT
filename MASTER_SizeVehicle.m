

%% Formatting
clc
clear
close all

%% Adding Folders and Files to MATLAB Path
addpath( 'GeometryProcessing')
addpath( 'AeroPropulsivePerfModel')
addpath( 'VehicleDefinition')
addpath( 'Aircraft Weight Functions')
addpath( 'SizingAndMissionAnalysis')
addpath( 'CommonFunctions')
addpath('BatteryModel')
addpath( 'FlightStream')
addpath( 'MADCASP')
addpath( 'PropDesign')
addpath( genpath('PropDesign'))
addpath( 'Results and Plots\')

%% Flags
Settings.ac3dFLAG = 0;
Settings.plotFLAG = 0;
Settings.FlightStream = 0;
if Settings.FlightStream == 1
    Settings.ac3dFLAG = 2;
end
Settings.SaveFileName = 'trial';                                            % .mat file under which to save sized results
Settings.PerformMissionAnalysis = 1;                                        % 1 = perform mission analysis, 0 = do not
Settings.PerformMassConvergence = 1;                                        % 1 = perform mass convergence, 0 = do not
Settings.MassMethod = 1;                                                    % mass convergence method (1 or 2)
Settings.MTOMAbort_kg = 8000;                                               % MTOM upper limit above which iterations will stop
Settings.MassConvergenceThreshold_kg = 3;                                % kg, convergence threshold for mass & energy iterations
Settings.MaxOEMIterations = 7;
Settings.OEMConvergenceThreshold_kg = 1;                                    % kg, convergence threshold for OEM iterations
Settings.MaxMassIterations = 15;                                            % maximum number of mass iterations
Settings.MTOMGuess_kg = 3700;
Settings.MaxPercdMTOMPerIter = 20;                                          % max MTOM change per iteration as %-age of current MTOM

Settings.AddlNames = {'MP1kW','MP2kW','MP3kW','MP4kW','LP1kW','LP2kW','TkW','LPRPM','BLP','BCP','LDe','LD','CL','CPRPM','dmdt','aTS','B1',...
                      'B2','B3','B4','B5','B6','OCV','VOLT','CRATE','ETA','DE','dod','PpackkW','SOC','dnac'};
Settings.StateNames = {'DOD','RE', 'FE','RER'};

Settings.SaveFileName = 'NASA_MTR.mat';
Settings.SaveFileNameac3d = 'NASA_MTR';
Settings.VSPFileName = 'NASA_Multi_tiltrotor';

Vehicle.Settings = Settings;


% % Manual Runs
FF.DL        = 15;                      % Disk Loading (lb/ft^2)
FF.WL        = 42.5;                       % Wing Loading (lb/ft^2)
FF.Vcruise   = 150;                        % Cruise Speed (kts)
FF.Range     = 37.5 * 1.852;
FF.AR        = 8.9;
FF.EnergyDensity_kWhkg = 0.14;
FF.MaxCRate = 60;

% Define the row names
Names = {'AE1', 'AE2', 'AE3', 'AE4', 'AE5', 'TE1', 'TE2', 'TE3', 'TE4', 'TE5'}';

% Define the data for each column
Names = {'BL'; 'AE2'; 'AE3'; 'AE4'; 'AE5'; 'TE1'; 'TE2'; 'TE3'; 'TE4'; 'TE5'};
WL = [44.75; 24; 34; 29; 34; 44.75; 24; 34; 29; 24];
DL = [15; 13; 10; 10; 10; 15; 10; 10; 13; 13];
AR = [12; 8;  12; 8;   8; 12; 8; 12; 8; 8];
Range = [60; 38; 38; 50; 50; 60; 38; 38; 50; 50];
Architecture_Vec = [1; 1; 1; 1; 1; 3; 3; 3; 3; 3];

% Create the table
ExcTbl = table(Names,WL, DL, AR, Range);

% Display the table
% disp(ExcTbl);




for i = 6
    FF = [];
    FF.DL                   = ExcTbl.DL(i);
    FF.WL                   = ExcTbl.WL(i);
    FF.Vcruise              = 150;
    FF.Range                = ExcTbl.Range(i) * 1.852 ;
    FF.AR = ExcTbl.AR(i);
    FF.TR = 0.8 ;
    FF.NBattery = 5 ;
    Architecture = Architecture_Vec(i);

    VehName = ExcTbl.Names{i};

    eval(sprintf('%s = VehicleDefinition_Baseline(Vehicle,Architecture,FF);',VehName))

    % eval(sprintf('%s.Settings.ac3dFLAG = 0;',VehName));
    % eval(sprintf('%s.Settings.plotFLAG = 0;',VehName));

    %% LOAD IN AIRCRAFT MISSION AND POINT PERFORMANCE CONSTRAINTS
    Dist_km = FF.Range;


    eval(sprintf('%s = MissionAndConstraints(%s, Dist_km, Architecture);',VehName,VehName));

    %% RUN MASTER SIZING FUNCTION
    eval(sprintf('[%s] = MasterSizer(%s);',VehName,VehName));
end








