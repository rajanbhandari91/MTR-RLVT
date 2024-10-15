clc
clear all
close all

% % adding path and subpath to MATLAB path
addpath( 'PropDesign')
addpath( genpath('PropDesign'))

% EXCEL FILE IN WHICH THE REQUIREMENTS FOR THE ***OPEN*** ROTOR ARE SPECIFIED
% Note: The first case is the design case. Rest are off-design cases
% FileName = 'PropRequirementsSetup_NMP.xlsx';
FileName = 'PropRequirementsSetup.xlsx';


% SETUP save name
SetupSaveName = 'LiftPropRaven.mat' ;
% SetupSaveName = 'CruisePropRaven.mat';


%%%%%% AC3D RELATED
% AC3D Flag. Set to 1 to generate AC3D file
GenAC3DFile = 1;
% Mean camber surface flag. Set to 1 to generate only mean surface
MCSFlag =0;
% SAVE NAME FOR AC3D FILE
AC3DSaveName = 'GenericProp.ac';






[PropData] = ReadSetupFile(FileName);

CSPropDesigner


% save the propeller/rotor setup
nRotor = 1;
nrad = length(QMIL.r);
npsi = 8;
[Setup] = CompleteRotorSetup(PropDef,QMIL,nRotor, nrad, npsi);

% Create GI for cruiseprop
SweepDesignedProp_cruise

save(SetupSaveName,'Setup')






















