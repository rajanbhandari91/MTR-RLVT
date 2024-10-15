function [FltCon] = CompleteFltCon(FltCon)


% altitude
if strcmpi(FltCon.ALT{2},'ft')
    h = FltCon.ALT{1}/3.28;        % convert ft to m
    FltCon.Alt_m = h;
end
if strcmpi(FltCon.ALT{2},'m')
    h = FltCon.ALT{1};
    FltCon.Alt_m = h;
end

% compute density and speed of sound
[~,a,~,rho] = atmosisa(h);

% velocity given either as KTAS, KEAS, or MACH. Compute the other two
if strcmpi(FltCon.Vel{2},'mach')
    FltCon.Mach = FltCon.Vel{1};
    FltCon.TAS = FltCon.Mach * a;
    FltCon.EAS = FltCon.TAS * sqrt(rho/1.225);
end

if strcmpi(FltCon.Vel{2},'keas')
    FltCon.EAS = FltCon.Vel{1}*0.514444;
    FltCon.TAS = FltCon.EAS * sqrt(1.225/rho);
    FltCon.Mach = FltCon.TAS/a;
end

if strcmpi(FltCon.Vel{2},'ktas')
    FltCon.TAS = FltCon.Vel{1}*0.514444;
    FltCon.EAS = FltCon.TAS * sqrt(rho/1.225);
    FltCon.Mach = FltCon.TAS/a;
end

FltCon.KTAS = FltCon.TAS/0.514444;
FltCon.KEAS = FltCon.EAS/0.514444;
FltCon.SpeedOfSound = a;
