function [EAS,TAS,Mach,Tinf,Pinf,rhoinf] = SetFltCon(Alt,dISA,VelType,Vel)

gamma = 1.4;    % ratio of specific heats for air
R = 287;        % specific heat of air, J/kg/K

% get standard sea-level properties
[~, ~, ~, rhoSLStd] = atmosisa(0);


% get standard atmosphere properties
[TISA, ~, PISA, ~] = atmosisa(Alt);

% TISA = TISA';
% PISA = PISA';

TISA = reshape(TISA,[length(TISA),1]);
PISA = reshape(PISA,[length(PISA),1]);

% actual temperature
Tinf = TISA + dISA;

% actual pressure
Pinf = PISA;

% actual density
rhoinf = Pinf./(R*Tinf);

% actual speed of sound
A = sqrt(gamma * R * Tinf);


% if Mach number has been provided as input
if strcmpi(VelType,'Mach')
    
    Mach = Vel;
    
    % compute true airspeed (TAS)
    TAS = Vel.*A;
    
    % compute equivalent airspeed (EAS)
    EAS = TAS.*sqrt(rhoinf./rhoSLStd);
    
end

% if TAS has been provided as input
if strcmpi(VelType,'TAS')
    
    TAS = Vel;
    
    % compute equivalent airspeed (EAS)
    EAS  = TAS.*sqrt(rhoinf./rhoSLStd);
    
    % compute Mach number
    Mach = TAS./A;
    
end

% if EAS has been provided as input
if strcmpi(VelType,'EAS')
    
    EAS = Vel;
    
    % compute true airspeed (TAS)
    TAS = EAS.*sqrt(rhoSLStd./rhoinf);
    
    % compute Mach number
    Mach = TAS./A;
    
end