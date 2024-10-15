function [Batt] = DefaultBattProperties(Vehicle)

L = load('MolicelCharacteristics');

Batt.DODRef =   0:0.01:1;                                     % reference values for depth of discharge
Batt.OCVRef =   L.OCVfun(Batt.DODRef);                          % corresponding values of open circuit voltage
Batt.R0Ref =    L.Rssfun(Batt.DODRef);                          % ohm, corresponding values of series resistance
if Vehicle.Architecture == 3
    Batt.R0Ref =    0.01.*L.Rssfun(Batt.DODRef);
end

Batt.R_RCRef =  zeros(size(Batt.DODRef));                       % ohm, resistance of RC pair
Batt.C_RCRef =  zeros(size(Batt.DODRef));                       % farad, capacitance of RC pair

% create gridded interpolants for brand new cell
Batt.GI_OCV_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.OCVRef, 'linear', 'nearest');
Batt.GI_R0_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.R0Ref, 'linear', 'nearest');
Batt.GI_R_RC_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.R_RCRef, 'linear', 'nearest');
Batt.GI_C_RC_fcn_DOD = griddedInterpolant(Batt.DODRef, Batt.C_RCRef, 'linear', 'nearest');

DOD_i = 0;
DOD_f = 1;
[int_OCV_dx, x_vec, RE_used] = FindAreaUnderOCVCurve(DOD_i, DOD_f, Batt);
Batt.GI_usedRE_fcn_DOD = griddedInterpolant(x_vec, RE_used, 'linear', 'nearest');
Batt.GI_DOD_fcn_usedRE = griddedInterpolant(RE_used, x_vec, 'linear', 'nearest');


Batt.int_OCV_dx = int_OCV_dx;

% Nominal cell capacity
Batt.NominalCellCapacity_Ah = 4.5;

% voltage and current limits
Batt.VoltageLimits = [2.9, 4.2];            % min and max voltage
if Vehicle.Architecture == 3
    Batt.CRateLimits = [-1, 30];             % discharge and charge C-rate limits (sign convention: negative for charge, positive for discharge)
else
    Batt.CRateLimits = [-1, 14];
end
% calculate manufacturer's rated discharge knockdown factor
CRATE = 1.0;
[Batt.KF_MFG] = Calc_kemfg(CRATE, Batt);

% Manufacturer's cell specific energy
Batt.SpecificEnergy_MFG_Whkg = 650;                                         % manufacturer's cell specific energy, Wh/kg

% end of life (EOL) resistance growth and capacity fade factors
Batt.EOLResistanceGrowthFactor = 1.3;
Batt.EOLCapacityFadeFactor = 0.85;