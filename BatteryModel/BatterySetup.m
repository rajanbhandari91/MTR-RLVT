



% https://genstattu.com/tattu-28000mah-22-2v-25c-6s1p-lipo-battery-pack.html

% series-parallel configuration
Batt.N_series = 1;      % number of cells in series in pack
Batt.N_parallel = 1;    % number of parallel branches in pack

Batt.VoltageLimits = [1.1, 4.24];        % min and max voltage
Batt.CRateLimits = [-1, 1000];             % discharge and charge C-rate limits (sign convention: negative for charge, positive for discharge)

Batt.Capacity_Ah = 28;

Batt.DODRef =   [-0.1,      0,       0.2,        0.4,        0.6,        0.8,        1.0           1.1];                       % reference values for depth of discharge
Batt.OCVRef =   [3.7,       3.7,     3.7,        3.5,        3.3,        3.1,        2.8           2.8];                       % corresponding values of open circuit voltage
Batt.R0Ref =    [0.6,       0.6,     0.6,        0.6         0.6,        0.6,        10.0          10] * 1e-3;               % ohm, corresponding values of series resistance
Batt.R_RCRef =  0*[0.6,       0.6,     0.6,        0.6,        0.6,        0.6,        0.6           0.6] * 1e-3;                % ohm, resistance of RC pair
Batt.C_RCRef =  0*[10,        10,      10,         10,         10,         10,         10            10] * 1e1;                  % farad, capacitance of RC pair


% create gridded interpolants
Batt.GI_OCV_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.OCVRef, 'linear', 'nearest');
Batt.GI_R0_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.R0Ref, 'linear', 'nearest');
Batt.GI_R_RC_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.R_RCRef, 'linear', 'nearest');
Batt.GI_C_RC_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.C_RCRef, 'linear', 'nearest');







LoadInput = [200000]
OpType = 1;
OCV = [];
R0 = [];
R_RC = [];
C_RC = [];
DOD = [0.00];

[CellOutputs, PackOutputs, TBI] = BatteryDynamics(LoadInput, OpType, OCV, R0, R_RC, C_RC, Batt, [DOD,0,0,0,0]);
fprintf('\nCell OCV: %0.2f V\n',CellOutputs(1))
fprintf('Cell terminal voltage: %0.2f V\n',CellOutputs(2))
fprintf('Cell power output: %0.2f W\n',CellOutputs(3))
fprintf('Cell current: %0.2f A\n',CellOutputs(4))
fprintf('Cell C-rate: %0.2f \n',CellOutputs(5))
fprintf('Cell Depth of discharge: %0.2f \n',100 * CellOutputs(6))
fprintf('Cell State of charge: %0.2f \n', 100 * CellOutputs(7))
fprintf('Cell Efficiency: %0.2f \n',CellOutputs(8))

fprintf('Pack terminal voltage: %0.2f V\n',PackOutputs(1))
fprintf('Pack power output: %0.2f W\n',PackOutputs(2))
fprintf('Pack current: %0.2f A\n',PackOutputs(3))
