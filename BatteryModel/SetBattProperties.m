function [Batt] = SetBattProperties(Updates,Vehicle)




% Get characteristics of the brand new cell
[Batt] = DefaultBattProperties(Vehicle);

% battery specific energy technology factor
Batt.K_BattSpecEnergy = Updates.K_BattSpecEnergy;

% Pack Mass Knockdown Factor
Batt.KF_PackMass = Updates.KF_PackMass;         % pack mass knockdown factor
% Batt.KF_PackMass = 0.755;
% stage of life
Batt.StageOfLife = Updates.StageOfLife;         % 0: brand new, 1: end of life
Batt.StageOfLife = 0;

% resistance growth and capacity fade factors at given stage of life
Batt.ResistanceGrowthFactor = 1 + (Batt.EOLResistanceGrowthFactor-1) * Batt.StageOfLife;
Batt.CapacityFadeFactor = 1 + (Batt.EOLCapacityFadeFactor - 1) * (Batt.StageOfLife);
% Batt.CapacityFadeFactor = 0.79;

% set cell capacity based on capacity fade factor and nominal capacity
Batt.CellCapacity_Ah = 4.5 * Batt.CapacityFadeFactor * Batt.K_BattSpecEnergy;

% update gridded interpolant for battery resistance
Batt.GI_R0_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.ResistanceGrowthFactor * Batt.R0Ref, 'linear', 'nearest');

%%%%% EFFECTIVE PACK-LEVEL SPECIFIC ENERGY 
% note 1: capacity fade factor (<1) reduces this value
% note 2: pack mass factor (<1) reduces this value
% note 3: manufacturer's rated discharge knockdown (<1) increases it
% note 4: battery technology improvement factor (>1) increases it

% update manufacturer rated discharge specific energy based on SOTA factor
Batt.SpecificEnergy_MFG_Whkg = Batt.SpecificEnergy_MFG_Whkg * Batt.K_BattSpecEnergy;

% calculate effective specific energy
Batt.SpecificEnergy_Whkg =  Batt.SpecificEnergy_MFG_Whkg * Batt.CapacityFadeFactor * Batt.KF_PackMass / Batt.KF_MFG;
% Batt.SpecificEnergy_Whkg = 650;

% max DOD
Batt.MaxDOD = 0.95;