function [Vehicle] = MasterSizer(Vehicle)
close all
global Settings
Settings = Vehicle.Settings;

% EXECUTION SETTINGS
PerformMissionAnalysis = Settings.PerformMissionAnalysis;                   % 1 = perform mission analysis, 0 = do not
PerformMassConvergence = Settings.PerformMassConvergence;                   % 1 = perform mass convergence, 0 = do not
OEMConvergenceThreshold_kg = Settings.OEMConvergenceThreshold_kg;
MaxMassIter = Settings.MaxMassIterations;                                 % maximum number of mass iterations

% idiot-proofing:
% if mass convergence is to be performed, then mission analysis has to be
% performed.
if PerformMassConvergence == 1
    PerformMissionAnalysis = 1;
end

% if mass convergence is not being performed, then set max mass iterations
% to 1 to prevent iterations
if PerformMassConvergence == 0
    MaxMassIter = 1;
end


% Set starting MTOM to MTOM guess in Settings struct
Vehicle.MassProp.MTOM_kg = Settings.MTOMGuess_kg;

% start counter
tic

% VARIABLE INITIALIZATIONS
MassIter = 0;                                                               % mass iteration counter
MassConvergence = 0;                                                        % mass convergence flag, 1 = converged

while MassConvergence==0 && MassIter <MaxMassIter

    % update mass iteration counter
    MassIter = MassIter + 1;

    % display sizing iteration no.
    fprintf('\n ** SIZING ITERATION %0.0f/%0.0f - MTOM %0.0f kg',MassIter,MaxMassIter, Vehicle.MassProp.MTOM_kg);



    Vehicle.Tracker.GeomUpdaterRuns = 0;
    Vehicle.Tracker.MassIter = MassIter;

    OEM_conv = 0;
    OEM_iter = 1;
    OEM_prev = 0;

    OEM_maxiter = Settings.MaxOEMIterations;

    while OEM_iter <= OEM_maxiter && OEM_conv == 0

        fprintf('\n OEM iter: %0.0f/%0.0f: ',OEM_iter, OEM_maxiter)
        % run the geometry updater once
        fprintf(' Geom updater 1/2 ||')
        Vehicle = VehicleGeometryUpdater(Vehicle);

        % run the power sizer
        fprintf(' Power sizer ||')
        Vehicle = PowerSizer(Vehicle);
        Mission = Vehicle.Mission;
        %
        % run the vehicle weight buildup calculator
        fprintf(' Weight Buildup, ')
        Vehicle = VehicleWeightBuildup(Vehicle);
        %
        % prepare the mass properties table
        Vehicle = PrepMassPropertiesTable(Vehicle);
        %
        % run the geometry updater a second time
        fprintf(' Geom updater 2/2 ||')
        Vehicle = VehicleGeometryUpdater(Vehicle);
        % %
        % % run the vehicle weight buildup calculator
        fprintf('  Weight Buildup ')
        Vehicle = VehicleWeightBuildup(Vehicle);

        % prepare the mass properties table
        Vehicle = PrepMassPropertiesTable(Vehicle);

        oem(OEM_iter) = Vehicle.MassProp.OEM_kg;

        dOEM = oem(OEM_iter) - OEM_prev;
        fprintf(' dOEM: %0.2f',dOEM);

        if abs(dOEM) < OEMConvergenceThreshold_kg
            OEM_conv = 1;
            % finalize the mass properties table
            Vehicle = PrepMassPropertiesTable(Vehicle);
        else
            OEM_prev = Vehicle.MassProp.OEM_kg;
            OEM_iter = OEM_iter + 1;
        end
    end




    % perform mission analysis, if enabled
    if PerformMissionAnalysis

        %%%%%%%%%% ANALYZE THE DESIGN MISSION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % set start energy mass
        Mission.StartEnergyMass_kg = Vehicle.MassProp.EnergyMass_kg;
        % set start mass
        Mission.TOGM_kg = Vehicle.MassProp.MTOM_kg;
        % energy
        Vehicle.MassProp.Energy_MJ = Vehicle.MassProp.EnergyMass_kg.*Vehicle.SOTA.SpecificEnergy_MJkg;


        [Mission, ~] = MissionAnalyzer(Mission,Vehicle);
        Vehicle.Mission = Mission;
        % MissionFiltered = rmfield(Mission, {'History', 'HistoryIU'});
        % 
        % % Copy all the fields at once by converting to cell and back to struct
        % Vehicle.Mission = cell2struct(struct2cell(MissionFiltered), fieldnames(MissionFiltered), 1);
        % Vehicle.Mission.History{MassIter} = Mission.History;
        % Vehicle.Mission.HistoryIU{MassIter} = Mission.HistoryIU;

        %%%%%%%%%% MASS CONVERGENCE ITERATION SETUP %%%%%%%%%%%%%%%%%%%%%%%
        if PerformMassConvergence == 1
            % RUN THE MASS CONVERGENCE ALGORITHM
            MassConvergenceControl_V2;
            
        end
        Vehicle.Iterations = struct2table(Iterations);
        disp(Vehicle.Iterations)

        if (PerformMassConvergence == 0 && MassIter>MaxMassIter)
            MassConvergence = 1;
        end

    end
end

% run the vehicle geometry updater once more [to update CG, MOI]
% fprintf('\n Plotting:\n')
% Settings.plotFLAG = 1;
% Settings.FlightStream = 0;
% fprintf('\n Running geometry updater 2/2.\n')
% Vehicle = VehicleGeometryUpdater(Vehicle);

% save iteration history
if exist('Iterations','var')
    Vehicle.Iterations = struct2table(Iterations);
    disp(Vehicle.Iterations)
end

%%%%%%%%%%%%%%%% AC3D FILE GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if Settings.ac3dFLAG > 0
    fprintf('\n Generating AC3d file %s.ac... ',Settings.SaveFileName)
    [out] = AC3DGeometryGenerator(Vehicle,Settings);
    fprintf('File generation complete.\n');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save(Settings.SaveFileName,'Vehicle')