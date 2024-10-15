function [CellOutputs, PackOutputs, ddt_BattStates] = BatteryDynamics(LoadInput, OpType, OCV, R0, R_RC, C_RC, BattParams, PackParams, BattStates)


% initializations
I = 0;
ddt_BattStates = zeros(1,5);

% voltage limits
V_min = BattParams.VoltageLimits(1);
V_max = BattParams.VoltageLimits(2);

% current limits
Current1C = BattParams.NominalCellCapacity_Ah;                       % calculate 1C current
I_maxcharge = BattParams.CRateLimits(1) * Current1C;
I_maxdischarge = BattParams.CRateLimits(2) * Current1C;

% capacity
Q_As = BattParams.CellCapacity_Ah * 3600;

% configuration
N_series = PackParams.N_series;
N_parallel = PackParams.N_parallel;
N_cells = PackParams.N_cells;


% DOD and SOC
DOD = BattStates(1);
SOC = 1- DOD;

% resistance drops in RC pairs
delta_V = BattStates(2);

% ampere-seconds 
As = BattStates(3);

% reversible energy
RevEnergy_J = BattStates(4);

% finite energy
FiniteEnergy_J = BattStates(5);


% steady state effective resistance
R_ss = R0 + sum(R_RC);


% if the battery is fully discharged...
if DOD>=0.9999999
    if OpType == 1
        LoadInput(LoadInput > 0) = 0;
    end
    if OpType == 2
        LoadInput(LoadInput < OCV) = OCV(LoadInput < OCV);
    end
    if OpType == 3
        LoadInput(LoadInput > 0) = 0;
    end
end

% if the battery is fully charged
if DOD <= 0.00001
    if OpType == 1 
        LoadInput(LoadInput < 0) = 0;
    end
    if OpType == 2
        LoadInput(LoadInput > OCV) = OCV(LoadInput > OCV);
    end
    if OpType == 3
        LoadInput(LoadInput < 0) = 0;
    end

end


%%% CASE 1: power specified
if OpType == 1

    % recognize input load as **pack** power (W)
    % compute power output of cells based on this
    P_W = LoadInput/(N_cells);

    % calculate discriminant: OCV^2 - 4 R_ss x P
    % constrain this to be >=0 (enforces absolute power limit)
    Discriminant = max(0,   (OCV)^2 - 4 * R_ss * P_W);


    % calculate current, unconstrained (to be checked against constraints)
    I_basic = (OCV - sqrt(Discriminant))/(2*R_ss);

    % compare computed current against current limits
    % and enforce current limitat constraints
    I_trial = min(I_maxdischarge, max(I_maxcharge, I_basic));

    % compare the terminal voltage that would occur with this current draw
    % against terminal voltage limits, and enforce voltage limit
    % constraints
    I_Vmin = (OCV-V_min)/R_ss;
    I_Vmax = (OCV-V_max)/R_ss;
    I = max(I_Vmax, min(I_Vmin, I_trial));


end



%%% CASE 2: voltage specified
if OpType == 2

    % recognize input load as **pack** voltage (V)
    % compute cell voltage based on this
    V = LoadInput / N_series;

    % enforce the voltage limits
    V = min(V_max, max(V_min, V));

    % calculate current
    I_trial = (OCV - sum(delta_V) - V)/R0;

    % compare computed current against current limits
    % and enforce current limitat constraints
    I = min(I_maxdischarge, max(I_maxcharge, I_trial));   

end




%%% CASE 3: C-rate specified
if OpType == 3

    % recognize input load as **pack** C-Rate 
    CRATE_in = LoadInput;

    % calculate cell current based on this C-rate
    I_trial = CRATE_in * Current1C;

    % enforce current limits
    I_trial = min(I_maxdischarge, max(I_maxcharge, I_trial));

    % compare the terminal voltage that would occur with this current draw
    % against terminal voltage limits, and enforce voltage limit
    % constraints
    I_Vmin = (OCV-V_min)/R_ss;
    I_Vmax = (OCV-V_max)/R_ss;
    I = max(I_Vmax, min(I_Vmin, I_trial));    


end






%%% RATE OF CHANGE OF DEPTH OF DISCHARGE
ddt_DOD = I/Q_As;



delta_Vdot =abs(I)./C_RC - delta_V./(R_RC.*C_RC);

if C_RC == 0 || R_RC == 0
    delta_Vdot = 0;
end




% calculate C-rate
CRATE = I/Current1C;

% calculate terminal voltage (volt)
V_terminal = OCV - R0 * I - sum(delta_V);

% calculate power at terminal (watt)
P_cell = V_terminal .* I;

% calculate ampere-hours
Ah = As/3600;

% instantaneous efficiency
%eta = I * V_terminal / (I * V_terminal + I^2 * R0 + sum(delta_V.^2./R_net) + sum(C_net.*delta_V.*delta_Vdot));


eta = 1;

% when discharging
if ddt_DOD > 0
    eta = I.*V_terminal./(OCV .* ddt_DOD .* Q_As);
end

% when recharging
if ddt_DOD < 0
    eta = (OCV .* ddt_DOD .* Q_As)./ (I.*V_terminal);
end


% pack outputs
PackVoltage = V_terminal * N_series;
PackPower_W = P_cell * N_cells;
PackCurrent_A = I * N_parallel;



% reversible energy (pack)
%  max(0,ddt_DOD) * Q_As
ddt_RevEnergy = OCV * I * N_cells;             % W
RevEnergy_Wh = RevEnergy_J/(3600);                                                                     % Wh

% energy associated with finite rate discharge
ddt_FiniteEnergy = max(0, PackPower_W);                                                                % W
FiniteEnergy_Wh = FiniteEnergy_J/(3600);                                                               % Wh

% cumulative discharge efficiency
DischargeEfficiency = FiniteEnergy_Wh./RevEnergy_Wh;


% quantities to be integrated
ddt_BattStates(1) = ddt_DOD;
ddt_BattStates(2) = delta_Vdot;
ddt_BattStates(3) = I;
ddt_BattStates(4) = ddt_RevEnergy;
ddt_BattStates(5) = ddt_FiniteEnergy;


% cell outputs
CellOutputs = [
    OCV;
    V_terminal;
    P_cell
    I
    CRATE
    DOD
    SOC
    eta];




PackOutputs = [...
    PackVoltage
    PackPower_W
    PackCurrent_A
    Ah
    RevEnergy_Wh
    FiniteEnergy_Wh
    DischargeEfficiency];
