function [k_mfg] = Calc_kemfg( CRATE, CellParams )
% Function to calculate manufacturer's rated discharge knockdown

Czero = 0;
DOD_i = 0.0;
DOD_f = 1.0;

% since this factor is calculated for a brand new cell
CellParams.CellCapacity_Ah = CellParams.NominalCellCapacity_Ah;

% calculate reversible energy (J)
[Erev] = CalcDischEnergy(DOD_i, DOD_f, Czero, CellParams);

% calculate finite energy for manufacturer's rated discharge
[Emfg] = CalcDischEnergy(DOD_i, DOD_f, CRATE, CellParams);

k_mfg = Emfg / Erev;

