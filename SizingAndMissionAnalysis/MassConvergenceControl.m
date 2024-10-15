
% MASS CONVERGENCE ITERATION SETTINGS

% Mass method 0: a testing mode in which mass is increased by a fixed 200
% kg per iteration. Used to check the variation of available and required
% energy masses versus MTOM.

% Mass method 1: Uses Newton-type iterations to converge upon zero net
% excess energy mass by using the slope of d(MTOM)/d(NEEM).

% Mass method 2: The MTOM update rule is
% MTOM_new = Payload / (1 - EmptyMassFraction - EnergyMassFraction)
MassMethod = Settings.MassMethod;

MTOMAbort_kg = Settings.MTOMAbort_kg;
MassConvergenceThreshold_kg = Settings.MassConvergenceThreshold_kg;        % kg, convergence threshold for mass iterations
PerformMassConvergence = Settings.PerformMassConvergence;                   % 1 = perform mass convergence, 0 = do not


History = Mission.History;

% MASS CONVERGENCE ITERATION CALCULATIONS

% calculate available energy mass (kg)
AvailableEnergyMass_kg = Vehicle.MassProp.MTOM_kg - Vehicle.MassProp.Payload_kg - Vehicle.MassProp.OEM_kg;

% current iteration MTOM
MTOM_kg = Vehicle.MassProp.MTOM_kg;

% variable initializations
GrowthFactor = 1;
dMass_kg = 0;


% % calculate the required reserve energy based on start energy and
% % the required reserve energy fraction
[ReqdStartEnergy_MJ, EnergyUsed_MJ, RequiredReserveEnergy_MJ] = CalcReqdStartEnergy(Mission);




% find required start energy, accounting for energy used + required
% reserves, but not (yet) considering minimum energy mass requirement
ReqdStartEnergy_MJ = EnergyUsed_MJ + RequiredReserveEnergy_MJ;

% find required sizing energy (MJ)
Mission.SizingEnergy_MJ = ReqdStartEnergy_MJ./(Mission.StartEnergyPerc/100);

% find the required start energy mass (kg), accounting for used and reserve
% energy, but not (yet) accounting for any minimum energy mass requirement
ReqdEnergyMass_basic_kg = Mission.SizingEnergy_MJ./Vehicle.SOTA.SpecificEnergy_MJkg;

% account for any minimum energy mass requirements by taking the MAX of the
% following
ReqdEnergyMass_kg = max(Vehicle.Propulsion.MinEnergyMass_kg, ReqdEnergyMass_basic_kg);

% test:
Vehicle.MassProp.EnergyMass_kg = AvailableEnergyMass_kg * ReqdEnergyMass_kg./sum(ReqdEnergyMass_kg);
Vehicle.MassProp.Energy_MJ = Vehicle.MassProp.EnergyMass_kg.*Vehicle.SOTA.SpecificEnergy_MJkg;

% sum the components to get net required energy mass (kg)
NetReqdEnergyMass_kg = sum(ReqdEnergyMass_kg);

% calculate net excess energy mass (kg)
NetExcessEnergyMass_kg = AvailableEnergyMass_kg - NetReqdEnergyMass_kg;

% calculate excess energy mass of each energy type (kg)
ExcessEnergyMass_kg = Vehicle.MassProp.EnergyMass_kg - ReqdEnergyMass_kg;

% capture current energy masses
EnergyMassesCaptured = Vehicle.MassProp.EnergyMass_kg;

% check if energy masses have converged
EnergyMassesConverged = all(abs(ExcessEnergyMass_kg )<MassConvergenceThreshold_kg);

% calculate current energy mass fraction (energy mass / MTOM)
emf = AvailableEnergyMass_kg/Vehicle.MassProp.MTOM_kg;




if PerformMassConvergence

    % Mass Method 1: Newton type iterations
    if MassMethod == 1
        % set default value of relaxation factor
        RelaxationFactor = 1.00;



        if MassIter == 1
            MTOM_old = MTOM_kg;
            NEEM_old = NetExcessEnergyMass_kg;
            dMass_kg = -200 * sign(NetExcessEnergyMass_kg);
            Vehicle.MassProp.MTOM_kg = MTOM_kg + dMass_kg;
        end


        if(MassIter >= 2 && MassConvergence == 0)
            MTOM_current = MTOM_kg;
            NEEM_current = NetExcessEnergyMass_kg;

            dMTOM_dEEM = max(1.0/emf,  (MTOM_current - MTOM_old)/(NEEM_current - NEEM_old));
            GrowthFactor = dMTOM_dEEM;

            if MassIter >= 10
                RelaxationFactor = 0.3;
            end
            
            % calculate unbounded MTOM change
            dMass_kg_nb = - NEEM_current * dMTOM_dEEM * RelaxationFactor;

            % calculate MTOM change bounds
            dMass_Limit = MTOM_kg * Settings.MaxPercdMTOMPerIter/100;

            % calculate bounded MTOM change
            dMass_kg = min(dMass_Limit, max(-dMass_Limit, dMass_kg_nb));

            % update "old" values to be used in next iteration
            MTOM_old = MTOM_kg;
            NEEM_old = NetExcessEnergyMass_kg;

            % verify whether mass convergence threshold has been met
            if(EnergyMassesConverged && abs(dMass_kg)<MassConvergenceThreshold_kg)
                MassConvergence = 1;
            else
                % set new MTOM
                Vehicle.MassProp.MTOM_kg = MTOM_current + dMass_kg;
            end

        end




    end



    % Mass Method 2:
    % update law is
    % MTOM_new = Payload / (1 - EmptyMassFraction - EnergyMassFraction)
    if MassMethod == 2

        EnergyMassFrac = NetReqdEnergyMass_kg / Vehicle.MassProp.MTOM_kg;
        EmptyMassFrac = Vehicle.MassProp.OEM_kg / Vehicle.MassProp.MTOM_kg;

        MTOM_new = Vehicle.MassProp.Payload_kg / (1 - EnergyMassFrac - EmptyMassFrac);

        dMass_kg = MTOM_new - Vehicle.MassProp.MTOM_kg;
        GrowthFactor = 1/EnergyMassFrac;

        Vehicle.MassProp.MTOM_kg = MTOM_new;

        % verify whether mass convergence threshold has been met
        if(EnergyMassesConverged && abs(dMass_kg)<MassConvergenceThreshold_kg)
            MassConvergence = 1;
        end

    end



end


% This is just a testing mode in which mass is increased by a set amount
if MassMethod == 0
    dMass_kg = 50;
    Vehicle.MassProp.MTOM_kg = MTOM_kg + dMass_kg;
    MassConvergence = 0;
    emf = 0;
end








% iteration abort due to excessively high MTOM
if Vehicle.MassProp.MTOM_kg > MTOMAbort_kg
    MassConvergence = -1;
    MTOM_kg = Vehicle.MassProp.MTOM_kg; % just so it shows up in the iter table
end

% add current iteration data to iteration history
Iterations.Iter(MassIter,1) = MassIter;
Iterations.MTOM(MassIter,1) = MTOM_kg;
Iterations.OEM(MassIter,1) = Vehicle.MassProp.OEM_kg;
Iterations.dMTOM(MassIter,1) = dMass_kg;
Iterations.REM(MassIter,:) = ReqdEnergyMass_kg;
Iterations.EEM(MassIter,1) = NetExcessEnergyMass_kg;
Iterations.GF(MassIter,1) = GrowthFactor;
Iterations.emf(MassIter,1) = emf;
Iterations.EnergyMasses(MassIter,:) = EnergyMassesCaptured;
Iterations.MassMethod(MassIter,1) = MassMethod;
Iterations.Conv(MassIter,1) = MassConvergence;
Iterations.Time(MassIter,1) = toc;


fprintf('\n MTOM: %0.0f kg, OEM: %0.0f kg, Fuel R/A: %0.0f/%0.0f kg, Batt R/A: %0.0f/%0.0f kg, Net EEM %0.0f kg, Method %0.0f, Conv: %0.0f \n', MTOM_kg, Vehicle.MassProp.OEM_kg, ReqdEnergyMass_kg(1), EnergyMassesCaptured(1),ReqdEnergyMass_kg(2), EnergyMassesCaptured(2), NetExcessEnergyMass_kg, MassMethod, MassConvergence);


OldExcessEnergyMass_kg = NetExcessEnergyMass_kg;
OldMTOM_kg = MTOM_kg;

