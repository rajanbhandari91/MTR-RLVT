function [Vehicle, PointPerf] = BatterySizer(Vehicle, PbattkW, PointPerf)





%%% BASED ON ENERGY
% compute mass per pack
BattPackMass_kg = Vehicle.MassProp.EnergyMass_kg(2)/Vehicle.Propulsion.NBatteryPacks;

% compute reversible energy associated with battery pack mass
RevEnergy_Wh = BattPackMass_kg * Vehicle.SOTA.BatterySpecificEnergy_kWhkg * 1000;

% find the total area under the OCV curve and log it
DOD_i = 0;
DOD_f = 1;
[Vehicle.Propulsion.Battery.int_OCV_dx] = AreaUnderOCVCurve(DOD_i, DOD_f, Vehicle.SOTA.Battery);

% find pack charge capacity in Ah
PackCapacity_Ah = RevEnergy_Wh / Vehicle.Propulsion.Battery.int_OCV_dx;

% find number of cells based on energy considerations
Ncells_energy = PackCapacity_Ah / Vehicle.SOTA.Battery.CellCapacity_Ah;

Vehicle.Propulsion.Battery.Ncells_energy = Ncells_energy;



%%% BASED ON POWER
% set evaluation type to "power"
OpType = 1; 


SizingPower_kW = max(PbattkW);
SizingPower_W = SizingPower_kW * 1000;

LoadInput = SizingPower_W;        % max power, W

DOD = 0.80;


% find maximum power attainable from a single cell at the given DOD
OCV = Vehicle.SOTA.Battery.GI_OCV_fcn_DOD(DOD);
R0 = Vehicle.SOTA.Battery.GI_R0_fcn_DOD(DOD);
R_RC = 0;
C_RC = 0;



BattStates = [DOD,0,0,0,0];

% CellOutputs = [
%     OCV;
%     V_terminal;
%     P_cell
%     I
%     CRATE
%     DOD
%     SOC
%     eta];
% 
% PackOutputs = [...
%     PackVoltage
%     PackPower_W
%     PackCurrent
%     Ah
%     RevEnergy
%     FiniteEnergy
%     DischargeEfficiency];





% execute the battery model
LoadType = 1;                           % power is being specified
ArbitraryLargePower = 1e7;
Load = ArbitraryLargePower;

[out,  ddt_BattStates] = EvalBatteryModel(Load,LoadType,DOD,Vehicle.SOTA.Battery, Vehicle.Propulsion.Battery);

Pmax_cell_W = out.CellPwr_W;

% find the total number of cells required
Ncells_power = SizingPower_W / Pmax_cell_W;

Vehicle.Propulsion.Battery.Ncells_power = Ncells_power;
Vehicle.Propulsion.Battery.Pmax_cell_W = Pmax_cell_W;




Vehicle.Propulsion.Battery.MinPackMass_kg = (1/1000)*(Ncells_power * Vehicle.SOTA.Battery.CellCapacity_Ah) * Vehicle.SOTA.Battery.int_OCV_dx / Vehicle.SOTA.BatterySpecificEnergy_kWhkg;







% determine number of cells accounting for both power and energy
% requirements
N_cells = ceil(max(Ncells_power, Ncells_energy));
Vehicle.Propulsion.Battery.N_cells = N_cells;

% find number of cells in parallel
Vehicle.Propulsion.Battery.N_series = ceil(Vehicle.SOTA.BusVoltage / Vehicle.SOTA.Battery.GI_OCV_fcn_DOD(0));          % number of cells in series in pack


% find number of parallel branches required
Vehicle.Propulsion.Battery.N_parallel = ceil(N_cells/Vehicle.Propulsion.Battery.N_series);

% recompute number of cells (series x parallel)
% Vehicle.Propulsion.Battery.N_cells = Vehicle.Propulsion.Battery.N_series * Vehicle.Propulsion.Battery.N_parallel;

% compute pack capacity
Vehicle.Propulsion.Battery.PackCapacity_Ah = Vehicle.SOTA.Battery.CellCapacity_Ah * Vehicle.Propulsion.Battery.N_cells;





% find actual peak power output of battery
Vehicle.Propulsion.Battery.PeakPowerCapability_kW = Vehicle.Propulsion.Battery.N_cells * Vehicle.Propulsion.Battery.Pmax_cell_W/1000;





for i = 1:length(PbattkW)

    PackLoad_W = PbattkW(i) * 1000;


    [out,  ddt_BattStates] = EvalBatteryModel(PackLoad_W,OpType,DOD,Vehicle.SOTA.Battery, Vehicle.Propulsion.Battery);

%     [CellOutputs, PackOutputs, TBI] = BatteryDynamics(PackLoad_W, OpType, OCV, R0, R_RC, C_RC, Vehicle.Propulsion.Battery, BattStates);

    PointPerf.PPackkW(i,1) = out.PackPwr_W /1000;
    PointPerf.OCV(i,1) = out.OCV;
    PointPerf.VOLT(i,1) = out.CellVoltage;
    PointPerf.CRATE(i,1) = out.CRATE;
    PointPerf.DOD(i,1) = out.DOD;
    PointPerf.BattEff(i,1) = out.eta;

end





% calculate pack reversible energy
Vehicle.Propulsion.Battery.PackRevEnergy_MJ = Vehicle.Propulsion.Battery.N_cells * Vehicle.SOTA.Battery.CellCapacity_Ah * Vehicle.SOTA.Battery.int_OCV_dx * 3600 / 1e6;
Vehicle.Propulsion.Battery.PackRevEnergy_kWh = Vehicle.Propulsion.Battery.PackRevEnergy_MJ * 1000/3600;

% calculate total reversible energy (all packs)
Vehicle.Propulsion.Battery.TotRevEnergy_MJ = Vehicle.Propulsion.Battery.PackRevEnergy_MJ  * Vehicle.Propulsion.NBatteryPacks;





% function to calculate area under OCV curve between initial and final DOD
    function [int_OCV_dx] = AreaUnderOCVCurve(DOD_i, DOD_f, BattParams)


        % create testing points
        x_vec = linspace(DOD_i, DOD_f, 1000);

        % find OCV
        ocv_vec = BattParams.GI_OCV_fcn_DOD(x_vec);

        % find mean OCV
        ocv_mean = 0.5 * (ocv_vec(1:end-1) + ocv_vec(2:end));

        dx = diff(x_vec);

        int_OCV_dx = sum(ocv_mean.*dx);




    end









end















