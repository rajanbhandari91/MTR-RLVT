% GenerateTestMatrix
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
addpath( 'MADCASP')

global Vehicle;

load RAVEN_V2.mat

load TestPointsFull
% load MassPropCases

[FltCon] = SetDefaultFltCon();




% % mass properties
% PercMTOW = 100;     % gross weight as % of MTOW
% CGPercMAC = 40;     % CG location as % of MAC
% useClass = 0;
% VehicleDefinition(useClass,PercMTOW,CGPercMAC);

Settings.AddlOPNames =  {'L1kW','L2kW','L3kW','L4kW','L5kW','L6kW','L7kW','L8kW','CkW','TotPwrkW','CL','LD','TR','LPRPM'};
Settings.MappingNum =1;
[nTestPoints,~] = size(TestPoints);


TrimCases = 1:nTestPoints


%  TrimCases = 1:2
%30
% TrimCases = 18

%10:14 - ok
%15:30 - ok

for i = 1:1:length(TrimCases)

    fprintf('\n Trim Case %0.0f, ',TrimCases(i));
    
    FltCon.Vel = {TestPoints.KEAS(TrimCases(i)),'keas'};                   % value corresponding to above selection
    FltCon.FPA = [0];                     % flightpath angle, deg
    FltCon.ALT = {TestPoints.Alt(TrimCases(i)),'ft'} ;                 % MSL altitude, ft
    FltCon.AOA = [];
    FltCon.TRK = 180;
    FltCon.BETA = [];
    
%     FltCon.ALT = {8000,'ft'};
%     FltCon.Vel = {200,'ktas'};
    
    FltCon.Turn = {'turnrate',0};
    
    %     FltCon.BETA = 4;
    
    
%     WtCode = TestPoints.WtConfig(TrimCases(i));

    
%     Vehicle = SetMassPropDef(WtCode, Vehicle, MassPropCases);
    
    [SolnTemp(i,:)] = TrimVehicle(FltCon,Vehicle,Settings);
    Header = table();
    Header.CaseNum = i;
%     Header.WtCode = WtCode;
    
    Soln(i,:) = [Header,SolnTemp(i,:)];
    
%     try
%         [SolnTemp(i,:)] = TrimVehicle(FltCon,Vehicle,Settings);
%         Header = table();
%         Header.CaseNum = i;
%         Header.WtCode = WtCode;
%         
%         Soln(i,:) = [Header,SolnTemp(i,:)];
%     catch ERRMSG
%     end
    
end


disp(Soln)

TrimPoints = Soln;
save('./MADCASP/TrimPointsFull.mat','TrimPoints');
save('./MADCASP/Vehicle.mat','Vehicle');





