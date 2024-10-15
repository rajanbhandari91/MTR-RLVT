
function output= fitness1(X)

x1 =    X(:,1);
x2 =    X(:,2);
x3 =    X(:,3);
x4 =    X(:,4);


%% Adding Folders and Files to MATLAB Path
addpath( 'GeometryProcessing')
addpath( 'AeroPropulsivePerfModel')
addpath( 'VehicleDefinition')
addpath( 'Aircraft Weight Functions')
addpath( 'SizingAndMissionAnalysis')
addpath( 'CommonFunctions')
addpath( 'MADCASP')
addpath( 'PropDesign')
addpath( genpath('PropDesign'))


%%

 
%% Setting up Full-Factorial cases


% % % Creating combinations of DL,WL,Vcruise
% DL        = [10 15 20 25 30];                  % Disk Loading (lb/ft^2)
% WL        = [30 40 50 60];                     % Wing Loading (lb/ft^2)
% Vcruise   = [175 200 225];                     % Cruise Speed (kts)
% Range     = [80 100 120 140];                  % Range (km)


% % Increase Range for HE and TE cases
% if Architecture == 2 || Architecture == 3
%     Range     = [80, 150, 200, 250];         % Range (km)
% end
% 
% KWhKG     = [0.30 0.35 0.40];
% CRate     = [3 5 7];
% 
% % % Fix Energy density and CRate for Turbo electric
% if Architecture == 3
%     KWhKG     = [0.40];
%     CRate     = [ 5 ];
% end
% 
% 
% % % Creating combinations of DL,WL,Vcruise
% DL        = [25];                  % Disk Loading (lb/ft^2)
% WL        = [50];                     % Wing Loading (lb/ft^2)
% Vcruise   = [200];                     % Cruise Speed (kts)
% Range     = [80]; 
% KWhKG     = [0.4];
% CRate     = [5];
% 
% 
% S = allcomb(DL,WL,Vcruise,Range,KWhKG,CRate);
% [n,~] = size(S);


%% Running the Sweep for full factorial
[N,D] =     size(X);
parfor i= 1:N
    FF           = [];
    Architecture = x4(i);

    % Folder where the sweep results get saved
    SaveResults = ['Results and Plots/FullFactorial_Archi_',num2str(Architecture),'/'];
       
    FF.DL                   = x1(i);
    FF.WL                   = x2(i);
    FF.Vcruise              = x3(i);
    FF.Range                = 80;
    FF.EnergyDensity_kWhkg  = 0.4;
    FF.MaxCRate             = 5;

    Vehicle                   = [];

    MTOM = 0;
    Energy_Used = 0;
    



    try

        Vehicle = VehicleDefinition_Baseline(Architecture,FF);
        Vehicle.Settings.ac3dFLAG = 0;
        Vehicle.Settings.plotFLAG = 0;
        Vehicle.Settings.SaveFileName = [SaveResults,'LPC_FAA_DL_',...
        num2str(FF.DL),'_WL_',num2str(FF.WL),'_VC_',num2str(FF.Vcruise),'_Range_',num2str(FF.Range),'_KWhKG_',num2str(FF.EnergyDensity_kWhkg),'_CRate_',num2str(FF.MaxCRate)...
        '_Archi_',num2str(Architecture),'.mat'];

        %% LOAD IN AIRCRAFT MISSION AND POINT PERFORMANCE CONSTRAINTS
        Dist_km = FF.Range;

        Mission = MissionAndConstraints(Vehicle, Dist_km, Architecture);


        %% RUN MASTER SIZING FUNCTION
        [Vehicle] = MasterSizer(Mission,Vehicle);
         MTOM            =   Vehicle.MassProp.MTOM_kg;
         Energy_Used     =   (Vehicle.Mission.History.Energy(1,2)-Vehicle.Mission.History.Energy(end,2)) +  (Vehicle.Mission.History.Energy(1,1)-Vehicle.Mission.History.Energy(end,1));
         Mission_Time    = sum(Vehicle.Mission.History.dTime);
    catch ERRMSG
%         FailedCases = ['Results and Plots/Failed Cases/ERROR_CASE',num2str(i)];
% %         save(FailedCases,'FF','ERRMSG')
        MTOM            = 3500;
        Mission_Time    = 4000;
        Energy_Used     = 1000;

    end
    Energy_MTOM = Energy_Used / MTOM;
    Energy_Mass_MTOM = Energy

    output(i,:) = [Mission_Time MTOM Energy_Used];

end

%%     % Power Loading in hp/1000 lb

%     PL(i) = (sum([Vehicle.Mission.PointPerf.Pm1kW(7) Vehicle.Mission.PointPerf.Pm2kW(7) Vehicle.Mission.PointPerf.Pm3kW(7) Vehicle.Mission.PointPerf.Pm4kW(7)...
%         Vehicle.Mission.PointPerf.Pm5kW(7) Vehicle.Mission.PointPerf.Pm6kW(7) Vehicle.Mission.PointPerf.Pm7kW(7) Vehicle.Mission.PointPerf.Pm8kW(7)])...
%         /Vehicle.MassProp.MTOM_kg)*1000./1.64399;
%
%
%     PMR(i) = Vehicle.DesignPoint.Lift_PMR_kWkg*1000/1.64399;


% close all
% semilogx(DL,PL,'*')























