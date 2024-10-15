function [Vehicle] = TechSOTADefinition(Vehicle,Updates)


%% TECHNOLOGICAL STATES OF THE ART (SOTA)
Vehicle.SOTA.ICEnginePMR_kWkg = 1.0;

% turboshaft engines
Vehicle.SOTA.TurboshaftSFC_lbhph = 0.5;
Vehicle.SOTA.TurboshaftSFC_kgkWh = Vehicle.SOTA.TurboshaftSFC_lbhph * (0.4536/0.746);
Vehicle.SOTA.TurboshaftSpecLength_mkW = 0.000459;
Vehicle.SOTA.TurboshaftSpecDia_mkW = 0.0000932;

% electric motors
Vehicle.SOTA.ElectricMotorPMR_kWkg = 5.0;                                   % kW/kg, based on Siemens 250 kW
Vehicle.SOTA.ElectricMotorSpecDiameter_mkW = 0.0018;                        % electric motor diameter/power rating (m/kW)
Vehicle.SOTA.ElectricMotorSpecLength_mkW = 0.0013;                          % electric motor length/power rating (m/kW)
Vehicle.SOTA.MotorEfficiency = 0.95;

% electric generators
Vehicle.SOTA.GeneratorPMR_kWkg = 6.25;                                      % kW/kg, Honeywell
Vehicle.SOTA.GeneratorEfficiency = 0.97;                                    % Honeywell
Vehicle.SOTA.GeneratorSpecDiameter_mkW = 0.36/1000;                        % m/kW, based on Honeywell design, 14" (0.36 m) for 1 MW (1000 kW) output
Vehicle.SOTA.GeneratorSpecLength_mkW = 0.61/1000;                          % m/kW, based on on Honeywell design, 24" (0.61 m) for 1 MW (1000 kW) output



% set battery properties
[Batt] = SetBattProperties(Updates,Vehicle);

Vehicle.SOTA.Battery = Batt;



Vehicle.SOTA.BatterySpecificEnergy_kWhkg = Batt.SpecificEnergy_Whkg/1000; 

% bus
Vehicle.SOTA.BusVoltage = 800;

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
% Vehicle.SOTA.BatteryEfficiency = Vehicle.SOTA.BatteryEfficiency * Vehicle.SOTA.CableEfficiency;
