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

global Settings
%% Loading in Baseline Vehicle Geometry
Vehicle = VehicleDefinition_Baseline(1);
%% Flags
Settings.plotFLAG = 1;
Settings.ac3dFLAG = 0;
%% Evaluating Geometry
Vehicle = VehicleGeometryUpdater(Vehicle);

warning('mtom hardcode, test script')
Vehicle.MassProp.MTOM_kg = 2150;
Vehicle = VehicleWeightBuildup(Vehicle);

Settings.plotFLAG = 1;
Settings.ac3dFLAG = 2;


Vehicle = VehicleGeometryUpdater(Vehicle);
Vehicle = PrepMassPropertiesTable(Vehicle);

disp(Vehicle.MassProp.WBD_Paper)
disp(Vehicle.MassProp.ErrorInGroupSummation)
% 
% warning('Testing script being run for ... testing')
% Tester

if Settings.ac3dFLAG > 0
    ac3dGEOM = Vehicle.Geom;
    Components = fieldnames(ac3dGEOM);
    for i = 1:length(Components)
        if ~isfield( ac3dGEOM.(Components{i}),'Vertices') && ~isfield( ac3dGEOM.(Components{i}),'Surfaces')
            ac3dGEOM = rmfield(ac3dGEOM,Components{i});
        elseif  isfield( ac3dGEOM.(Components{i}),'Vertices') && isfield( ac3dGEOM.(Components{i}),'Surfaces')

            if isempty( ac3dGEOM.(Components{i}).Vertices) && isempty( ac3dGEOM.(Components{i}).Surfaces)
                ac3dGEOM = rmfield(ac3dGEOM,Components{i});
            end
        end
    end
    [VertColl,VertColl_FSBLWL]=makeAC3D(ac3dGEOM, 'DUeVTOLtest.ac', 1,0,[],Settings.ac3dFLAG);
end

save('Vehicle','Vehicle')