function [Vehicle] = MasterSizer_Updated(Mission,Vehicle)
close all
global Settings
Settings.plotFLAG = 1;
Settings.ac3dFLAG = 1; 
% Settings.SaveFileName = Vehicle.Settings.SaveFileName;
% Settings.SaveFileNameac3d = Vehicle.Settings.SaveFileNameac3d ;


% EXECUTION SETTINGS
PerformMissionAnalysis = 0;                                                 % 1 = perform mission analysis, 0 = do not
PerformMassConvergence = 0;                                                 % 1 = perform mass convergence, 0 = do not
MaintainSetMTOM = 1;                                                        % 1 = maintain the given MTOM, 0 = do not

% start counter
tic

% MASS CONVERGENCE ITERATION SETTINGS

% Mass method 1: uses energy mass fraction at each iteration to determine
% the MTOM increment for that iteration

% Mass method 2: uses bisection algorithm to bracket the solution, and then
% bisects the intervals following standard bisection algorithm

% Mass method 3: uses bisection algorithm to bracket the solution, then
% bisects the intervals until threshold interval size, then reverts to
% method 1 for fine adjustments
MassMethod = 3;

% Maximum percentage bump-up or bump-down of MTOM when using bisection algorithm prior
% to bracket being found. Note: specify both as positive values!
% example: 0.40 --> 40% of MTOM
dMTOM_maxincrement = 0.20;
dMTOM_maxdecrement = 0.20;

% MTOM above which to abort iterations
warning('Change it to suitable value')
MTOMAbort_kg = 50;


Switchover_MTOMInterval_kg = 1;
Switchover_EMFTolerance = 0.005;

MassConvergenceThreshold_kg = 0.012;                                            % kg, convergence threshold for mass iterations
MaxMassIter = 15;                                                           % maximum number of mass iterations


% VARIABLE INITIALIZATIONS
MassIter = 0;                                                               % mass iteration counter
MassConvergence = 0;                                                        % mass convergence flag, 1 = converged
Vehicle.Tracker.GeomUpdaterRuns = 0;

% VARIABLE INITIALIZATIONS RELATING TO VEHICLE

%  Vehicle.MassProp.EnergyMass_kg = [1,1];                                     % set to initial energy guess
%

if MaintainSetMTOM==1
    MaxMassIter = 1;
end


while MassConvergence==0 && MassIter <MaxMassIter
    
    % update mass iteration counter
    MassIter = MassIter + 1;
    
    % display sizing iteration no.
    fprintf('\n ** SIZING ITERATION %0.0f/%0.0f',MassIter,MaxMassIter);
    
    % run the geometry updater once
    fprintf('\n Running geometry updater 1/2.')
    Vehicle = VehicleGeometryUpdater(Vehicle);
    
%     % run the power sizer
%     fprintf('\n Running power sizer.\n')
%     Vehicle = PowerSizer(Vehicle,Mission);
%     Mission = Vehicle.Mission;
%     
%     % run the vehicle weight buildup calculator
    fprintf('\n VehicleWeightBuildup.m: ')
%     Vehicle = VehicleWeightBuildup(Vehicle);
% % 
% %     % prepare the mass properties table
%     Vehicle = PrepMassPropertiesTable(Vehicle);
% 
%     % run the geometry updater a second time
%     fprintf('\n Running geometry updater 2/2.\n')
%     Vehicle = VehicleGeometryUpdater(Vehicle);






    % perform mission analysis, if enabled
    if PerformMissionAnalysis
        
        %%%%%%%%%% ANALYZE THE DESIGN MISSION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        DesignMissionAnalyzer;     
       
        %%%%%%%%%% MASS CONVERGENCE ITERATION SETUP %%%%%%%%%%%%%%%%%%%%%%%
        if PerformMassConvergence == 1
            
            % RUN THE MASS CONVERGENCE ALGORITHM
            MassConvergenceControl;
            
            % Screen printout of iteration summary
            % MassDiff = Vehicle.MassProp.MTOM_kg - OldMTOM_kg;
            fprintf('\n\n iter: %0.0f, dMass %0.2f kg, New mass %0.1f, EMass F: %0.1f kg B: %0.1f kg, Res En F: %0.2f B: %0.2f, Method: %0.0f, Conv: %0.0f \n',...
                MassIter, dMass_kg, Vehicle.MassProp.MTOM_kg, EnergyMass_kg(1),  EnergyMass_kg(2), 100 * History.Energy(end,1)/History.Energy(1,1), 100 * History.Energy(end,2)/History.Energy(1,2),MassMethod,MassConvergence);
            
        end
        
        
        
    end
    
    % if mass convergence or mission analysis not to be performed, set
    % convergence flag to 1 to exit the while loop
%     if PerformMassConvergence == 0 || PerformMissionAnalysis == 0
%         MassConvergence =1;
%     end
    
end


% run the vehicle geometry updater once more [to update CG, MOI]
fprintf('\n Plotting:\n')
Settings.plotFLAG = 1;
fprintf('\n Running geometry updater 2/2.\n')
% Vehicle = VehicleGeometryUpdater(Vehicle);


% prepare the mass properties table [accounting for all components]
% Vehicle = PrepMassPropertiesTable(Vehicle);






if PerformMissionAnalysis
    
    diffEMass = History.EMass(1,:) -  History.EMass(end,:);
    SAR = History.Dist(end)./diffEMass;
    SAR(isinf(SAR)) = 0;
    
    History.MassFrac = History.Mass./History.Mass(1);
    History.EFrac = History.Energy./(History.Energy(1,:) + 1e-20);
    
    History.Dist = History.Dist/1000;
    History.dDist = History.dDist/1000;
    
    History.Flag = History.ADDL(:,end);
    
    Mission.DistTable_km = DistTable/1000;
    Mission.DistTable_NM = Mission.DistTable_km*0.539957;
    Mission.TimeTable_min = TimeTable/60;
    
    Mission.History = History;
    
    AddlNames = {'L1kW','L2kW','L3kW','L4kW','L5kW','L6kW','L7kW','L8kW','CkW','TotkW','CL','LD','N1','N2','N3','N4','N5','N6','N7','N8','Ncp','FLAG'};
    AddlTbl = array2table(History.ADDL);
    AddlTbl.Properties.VariableNames = AddlNames;
    
    History = [History,AddlTbl];
    
    Mission.HistoryIU = History;
    
    Mission.HistoryIU.Time = Mission.History.Time/60;                           % time in min
    Mission.HistoryIU.dTime = Mission.History.dTime/60;                         % time in min
    Mission.HistoryIU.Alt = Mission.History.Alt*3.28;                           % alt in ft
    Mission.HistoryIU.rho = Mission.History.rho/1.225;                          % rho as ratio
    Mission.HistoryIU.EAS = Mission.History.EAS/ 0.51444;                       % speed in kt
    Mission.HistoryIU.TAS = Mission.History.TAS/ 0.51444;                       % speed in kt
    Mission.HistoryIU.GS = Mission.History.GS/ 0.51444;                         % speed in kt
    Mission.HistoryIU.Ps = Mission.History.Ps*3.28*60;                          % in ft/min
    Mission.HistoryIU.EnHt = Mission.History.EnHt*3.28;                         % in feet
    Mission.HistoryIU.Dist = Mission.History.Dist*0.539957;                     % in NM
    Mission.HistoryIU.dDist = Mission.History.dDist*0.539957;                   % in NM
    Mission.HistoryIU.dVdt = Mission.History.dVdt/9.81;                         % in g
    Mission.HistoryIU.dhdt = Mission.History.dhdt*3.28*60;                      % in ft/min
    Mission.HistoryIU.AOA = Mission.History.AOA*180/pi;                         % in deg
    Mission.HistoryIU.FPA = Mission.History.FPA*180/pi;                         % in deg
    Mission.HistoryIU.dFPAdt = Mission.History.dFPAdt*180/pi;                   % in deg/s
    Mission.HistoryIU.Mass = Mission.History.Mass*2.20462;                      % in lb
    Mission.HistoryIU.EMass = Mission.History.EMass*2.20462;                    % in lb
    Mission.HistoryIU.dMass = Mission.History.dMass*2.20462;                    % in lb
    Mission.HistoryIU.dMdt = Mission.History.dMdt*2.20462;                      % in lb/sec
    
    
    
    
    Mission.SAR = SAR;
    
    
    Vehicle.Mission = Mission;
    
    if(isfield(Vehicle,'Mission'))
        % convert battery rating to kWh
        Vehicle.Propulsion.BatteryCapacitykWh = Vehicle.Mission.History.Energy(1,2)/3.6;
    end
    
    DistTable = DistTable/1000;
    
    Vehicle.Mission.HistoryIU
    
end



% This just to generate a plot
% fprintf('\n Plotting:\n')
% Settings.plotFLAG = 1;
% Temp1 = VehicleGeometryUpdater(Vehicle);

% % This is to display the weight buildup table
% fprintf('\n VehicleWeightBuildup.m: ')
% Temp2 = VehicleWeightBuildup(Vehicle);
% Temp3 = PrepMassPropertiesTable(Vehicle);
% saveas(figure(1),'Aircraft Geometry.png')
% disp(Vehicle.MassProp.WBD_Paper)

% save iteration history
if exist('Iterations','var')
    Vehicle.Iterations = struct2table(Iterations);
    disp(Vehicle.Iterations)
end


%%%%%%%%%%%%%%%% AC3D FILE GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if Settings.ac3dFLAG > 0
    fprintf('\n Generating AC3d file %s.ac... ',Settings.SaveFileNameac3d)
    [out] = AC3DGeometryGenerator(Vehicle,Settings);
    fprintf('File generation complete.\n');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 save(Settings.SaveFileName,'Vehicle')