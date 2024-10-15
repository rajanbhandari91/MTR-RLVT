function [out, ddt_BattStates] = EvalBatteryModel(LoadInput,LoadType,DOD, BattParams, PackParams)

if~isfield(PackParams, 'N_parallel')
    PackParams.N_parallel = 1;
end

if~isfield(PackParams, 'N_series')
    PackParams.N_series = 1;
end

if~isfield(PackParams, 'N_cells')
    PackParams.N_cells = 1;
end

% open circuit voltage
OCV = BattParams.GI_OCV_fcn_DOD(DOD);

% steady-state resistance
R0 = BattParams.GI_R0_fcn_DOD(DOD);

% transient resistance
R_RC = 0; %BattParams.GI_R_RC_fcn_DOD(DOD);

% transient capacitance
C_RC = 0; %BattParams.GI_C_RC_fcn_DOD(DOD);

% Battery states
BattStates = [DOD,0,0,0,0];

% call battery dynamics function
[CellOutputs, PackOutputs, ddt_BattStates] = BatteryDynamics(LoadInput, LoadType, OCV, R0, R_RC, C_RC, BattParams, PackParams, BattStates);

% cell outputs
% CellOutputs = [
%     OCV;
%     V_terminal;
%     P_cell
%     I
%     CRATE
%     DOD
%     SOC
%     eta];

out.OCV = CellOutputs(1);
out.CellVoltage = CellOutputs(2);
out.CellPwr_W = CellOutputs(3);
out.CellCurrent_A = CellOutputs(4);
out.CRATE = CellOutputs(5);
out.DOD = CellOutputs(6);
out.SOC = CellOutputs(7);
out.eta = CellOutputs(8);


% PackOutputs = [...
%     PackVoltage
%     PackPower_W
%     PackCurrent_A
%     Ah
%     RevEnergy_Wh
%     FiniteEnergy_Wh
%     DischargeEfficiency];


out.PackVoltage = PackOutputs(1);
out.PackPwr_W = PackOutputs(2);
out.PackCurrent_A = PackOutputs(3);
out.Ah = PackOutputs(4);
out.RevEnergy_Wh = PackOutputs(5);
out.FiniteEnergy_Wh = PackOutputs(6);
out.DischargeEfficiency = PackOutputs(7);