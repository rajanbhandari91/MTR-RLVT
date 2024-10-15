function [Vehicle] = VehicleSystemUpdate(Vehicle)

% fprintf('\n Mass received %0.2f',Vehicle.MassProp.TOGM_kg);
% constants
g = 9.81;

% conversion factors
Conv_kt_to_ms = 0.51444;
Conv_m_to_ft = 3.28;
Conv_kg_to_lb = 1/0.4536;

% vehicle gross weight
mass = Vehicle.MassProp.TOGM_kg;




% temporary hardcodes
DL = Vehicle.SizingPoint.DL_kgm2;               % disc loading, lift rotors, kg/m2
PMR_kWkg = 0.65;        % overall power to mass ratio, kW/kg
WL = Vehicle.SizingPoint.WL_kgm2; % kg/m2
AR = Vehicle.Geom.Wing.AR;

% update max overall power that can be supplied by the propulsion system
Vehicle.Propulsion.MaxOverallPower_kW = PMR_kWkg * mass;


% update lift rotor size
Vehicle.Propulsion.LPSetup.RotorRadius = 0.5 * sqrt(mass/(2*pi*DL));
Vehicle.Propulsion.LiftPropDiam_m = 2 * Vehicle.Propulsion.LPSetup.RotorRadius;
Vehicle.Propulsion.LPSetup = UpdateRotorSetup(Vehicle.Propulsion.LPSetup);
Vehicle.Propulsion.LiftPropDL_kgm2 = mass/(Vehicle.Propulsion.NLiftProps * pi * Vehicle.Propulsion.LiftPropDiam_m^2/4);


% update the transition schedule

rho = 1.225;
AOAThreshold = 10;
CLThreshold =  Vehicle.Aero.SI_CL_AOA_BETA_FLAP(AOAThreshold,0,0);
VThreshold = sqrt((2/rho)*(mass*g/Vehicle.Geom.S)*(1/CLThreshold));

Vehicle.Aero.VThreshold = VThreshold;
Vehicle.Aero.PitchSchedule_V = griddedInterpolant([0,0.9*VThreshold,VThreshold],[0,0,AOAThreshold],'linear','nearest');





% Top speed requirement
VmaxKTAS = Vehicle.Operations.VmaxSLkt ;
V = VmaxKTAS * Conv_kt_to_ms;
[~,~,~,rho] = atmosisa(0000/3.28);
mass = Vehicle.MassProp.TOGM_kg;
vdot = 0;
fpadot = 0;
dhdt = 0;
dt = 1;
EvalType = 'power';

[Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,ADDL,fval,exitflag(1),output] = SolveFltCon(mass,dt,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle);
dtcp(1) = dt_out(1);

dtcp_scaler = max(dtcp);
Vehicle.Propulsion.MotorPowerRatingkW(1) = Vehicle.Propulsion.MotorPowerRatingkW(1) * dtcp_scaler;




% Hover requirement
V = 0;
[~,~,~,rho] = atmosisa(0000/3.28);
mass = Vehicle.MassProp.TOGM_kg;
vdot = 0;
fpadot = 0;
dhdt = 0;
dt = 1;
EvalType = 'power';

[Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,ADDL1,fval,exitflag(2),output] = SolveFltCon(mass,dt,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle);

P_LR_kW(1) = ADDL1(2);
if exitflag(2)<=0
    P_LR_kW(1) = mass * 0.30; % defaults to 0.3 kW/kg;
end


% Vertical climb rate requirement
V = 0;
[~,~,~,rho] = atmosisa(0000/3.28);
mass = Vehicle.MassProp.TOGM_kg;
vdot = 0;
fpadot = 0;
dhdt = 500/(3.28*60);
dt = 1;
EvalType = 'power';

[Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,ADDL2,fval,exitflag(3),output] = SolveFltCon(mass,dt,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle);
P_LR_kW(2) = ADDL2(2);

if exitflag(3)<=0
    P_LR_kW(2) = mass * 0.32; % defaults to 0.32 kW/kg;
end


P_LR_kW = min(0.35*mass,max(P_LR_kW));

Vehicle.Propulsion.LR_kWkg = P_LR_kW/mass;

Vehicle.Propulsion.MotorPowerRatingkW(2) = P_LR_kW;
Vehicle.Propulsion.Flags = exitflag;





Lboom_ft = (Vehicle.Propulsion.NLiftProps/2) * (Vehicle.Propulsion.LiftPropDiam_m * 3.28) + Vehicle.Geom.Wing.S/Vehicle.Geom.Wing.b + (2.1)*3.28;
Lboom_m = Lboom_ft/3.28;
Lfus_m = Vehicle.Geom.Fus.Ltot/3.28;
semispan_HT = 0.5 * Vehicle.Geom.HT.bh/3.28;

CablingkVAm = [...
    2 * (0.4 * Vehicle.Geom.b/2 + Lboom_m) * (Vehicle.Propulsion.MotorPowerRatingkW(2)/2);       % distribution to lift rotors
    2 * (0.5 * Lfus_m + 0.5 * semispan_HT) * (Vehicle.Propulsion.MotorPowerRatingkW(1)/2)];       % distribution to cruise props

% fprintf('here')
% sum(CablingkVAm)

Vehicle.Propulsion.CablingkVAm = sum(CablingkVAm);













if Vehicle.Architecture == 2
    
    %Vehicle.Propulsion.Prated_kW = Vehicle.Propulsion.MotorPowerRatingkW(1) / 2;
    
    Vehicle.Propulsion.Prated_kW = P_LR_kW / 2;
    
    Vehicle.Propulsion.Prated = Vehicle.Propulsion.Prated_kW * 1000 / 746;
    
    Vehicle.Propulsion.GeneratorPowerRatingkW = Vehicle.Propulsion.Prated_kW * Vehicle.Propulsion.Neng;
    
    % battery power sufficient to hover with one gas engine failed
    Vehicle.Propulsion.BatteryRatedPower_kW = P_LR_kW - Vehicle.Propulsion.Prated_kW;
    
end














     
% 
% % max overall power constraint
% Vehicle.Propulsion.MaxOverallPower_kW = 1.5*P_LR_kW;
% 




% updating wing area
Vehicle.Geom.S = mass/WL; % m2
Vehicle.Geom.cbar = Vehicle.Geom.S/sqrt(AR * Vehicle.Geom.S);

% the following are in English units, and are used for weight estimation
Vehicle.Geom.Wing.S = Vehicle.Geom.S * Conv_m_to_ft^2;                      % wing planform area, ft2
Vehicle.Geom.Wing.b = sqrt(AR * Vehicle.Geom.Wing.S);                       % wing span, ft
Vehicle.Geom.b = sqrt(AR * Vehicle.Geom.S);
Vehicle.Geom.Wing.c = Vehicle.Geom.cbar * Conv_m_to_ft;                     % wing chord, ft


Vehicle.Geom.Wing.WingLoading_kgm2 = mass/Vehicle.Geom.S;
Vehicle.Geom.Wing.WingLoading_psf = (mass * Conv_kg_to_lb)/Vehicle.Geom.Wing.S;


