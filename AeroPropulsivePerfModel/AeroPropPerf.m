function [Fxap,Fzap,M_out,dmdt,dEdt,ADDL,ddtStates] = AeroPropPerf(AOA,dcon,dt,rho,V,fpa,Vehicle)


V = max(0.1,V);

% initializations
dmdt = [0,0];
dEdt = [0,0];

ddtStates = zeros(1,4);

% set CG location to target CG
r_cg = Vehicle.MassProp.TargetCG;
% r_cg = Vehicle.MassProp.CG;
x_cg = r_cg(1); y_cg = r_cg(2); z_cg = r_cg(3);


% flow conditions
FlowCon.SpeedOfSound = 340.2941 * sqrt(rho/1.225);
FlowCon.DynVisc = 0.17000E-04;
FlowCon.rho = rho;


% capture control settings
ulat = 0;
ulong = dcon(1);
udir = 0;



if(length(dcon)>1)
    ulat = dcon(2); udir = dcon(3); 
end


FltCon.KTAS = V/0.51444;
FltCon.KEAS = FltCon.KTAS * sqrt(rho/1.225);
FltCon.SpeedOfSound = FlowCon.SpeedOfSound;
VThreshold = Vehicle.Aero.VThreshold;

TWV = dt(2);
TWH = dt(1);

% Lift Props
LPVT = 2/6 * (Vehicle.MassProp.MTOM_kg * 9.81 * TWV);
LPRPM = max(Vehicle.Propulsion.LPSetup.DesignCases.RPM); 
LPRPM(V > VThreshold) = 0;

% compute advance ratio
JLP = V/((LPRPM/60)*Vehicle.Propulsion.LiftPropDiam_m);
BLP = Vehicle.Propulsion.LPSetup.PropCurves.GI_beta_T_J(0.5*LPVT,JLP);
BLP(isnan(BLP)) = 0;

% Main Props
MPVT = 4/6 * (Vehicle.MassProp.MTOM_kg * 9.81 * TWV);
MPFT = (Vehicle.MassProp.MTOM_kg * 9.81 * TWH);
MPT = sqrt(MPVT^2 + MPFT^2);

% dnac
PropAng = atan2d(MPVT , MPFT);

DesignRPM = max(Vehicle.Propulsion.CPSetup.DesignCases.RPM);
MPRPM = DesignRPM; 
JMP = V*cosd(PropAng)/((MPRPM/60)*Vehicle.Propulsion.MainPropDiam_m);
BMP = Vehicle.Propulsion.CPSetup.PropCurves.GI_beta_T_J(0.25*MPT,JMP);



% States required by FMComp
States.AirData.AOADeg = AOA * 180/pi;
% States.AirData.AOADeg = 0;

% warning('beta hardcode in AeroPropPerf.m')
States.AirData.BETADeg = 0;             % symmetric flight condition
% States.AirData.BETADeg = AOA * 180/pi;
States.AirData.BETA = States.AirData.BETADeg *pi/180;                % symmetric flight condition

States.AirData.TAS = V;
States.AirData.uw = V * cos(AOA) * cosd(States.AirData.BETADeg);
States.AirData.vw = V * sind(States.AirData.BETADeg);                  % symmetric flight condition
States.AirData.ww = V * sin(AOA) * cosd(States.AirData.BETADeg);

States.AirData.pw = 0;                  % quasi-steady
States.AirData.qw = 0;                  % quasi-steady
States.AirData.rw = 0;                  % quasi-steady

States.AirData.a =  FlowCon.SpeedOfSound;   % note: this is a hardcode ATM
States.AirData.qbar = 0.5 * rho * V^2;
States.AirData.rho = rho;               % air density
States.Attitude.ph = 0;                 % symmetric flight condition
States.Attitude.th = AOA;               % only reqd for gravity load calc


UTrimVec = [...
    ulat
    ulong
    udir
    PropAng
    BMP
    BLP
    LPRPM
    MPRPM
    ];



% Call the control allocator function [same function to be used during
% simulation]
[Controls] = ControlAllocator(UTrimVec,FltCon, Vehicle);

% leave XInflow blank. This will trigger internal iterations to solve lift
% rotor inflow within FMComp
XInflow = [];

% mass: set to zero. This will result in gravitational load computed by
% FMComp coming out to [0;0;0]. This is fine, since within the sizing code,
% gravitational loads are computed elsewhere.
m = 0;

% CG location (r_cg):



EvalMode = 2;

[Fbf,Mbf,Hr,Hrdot,AddlRes,Xrotor_dot,AddlOutputs] = FMComp(States,Controls,XInflow,Vehicle,m,r_cg,EvalMode);


% The forces (Fbf) and moments (Mbf) computed by FMComp are expressed in
% body-fixed axes and with reference to the aircraft reference point
% (moment reference center)
Fxbf = Fbf(1);      % x-direction force
Fybf = Fbf(2);      % y-direction forcec
Fzbf = Fbf(3);      % z-direction force

Mxbf = Mbf(1);      % rolling moment
Mybf = Mbf(2);      % pitching moment
Mzbf = Mbf(3);      % yawing moment


% These need to be converted to wind axes (Fxap, Fzap)
Fxap =  Fxbf * cos(AOA) + Fzbf * sin(AOA);
Fzap = -Fxbf * sin(AOA) + Fzbf * cos(AOA);


% the y-axis is the same between body and wind axes, therefore,
Myap = Mybf - (z_cg * Fxbf - x_cg * Fzbf);


% calculate rolling moment and yawing moment about CG
Mxap = Mxbf - (-z_cg * Fybf + y_cg * Fzbf);
Mzap = Mzbf - (-y_cg * Fxbf + x_cg * Fybf);

M_out = [Myap; Mxap; Mzap];


% ADDITIONAL PARAMETERS CAPTURED FROM FMCOMP
% AddlOutputs = [LiftPropPwrkW, CruisePropPwrkW , TotPwrkW, CL, LDRatio, CD];


LiftPropPwrkW = AddlOutputs(1:2);
LiftPropPwrkW = max(0, LiftPropPwrkW);


CruisePropPwrkW = AddlOutputs(3:6);
CruisePropPwrkW = max(0, CruisePropPwrkW);


TotPwrkW = AddlOutputs(7);
CL = AddlOutputs(8);
LDRatio = AddlOutputs(9);

LDe = AddlOutputs(10);
CD = AddlOutputs(17);
Cmy = AddlOutputs(18);



%%%%%%%%%%%%%%%%%%%%%%%%%% dEdt and dmdt %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dmdt(1) = 0;
dEdt(1) = 0;

dEdt(2) = 0;


aTS = 0;

% scatter(Flap(k),Fzbf);
% hold on
% end
%% ARCHITECTURE 1: ALL-ELECTRIC (AE) PROPULSION SYSTEM ARCHITECTURE
if Vehicle.Architecture == 1
    
    Ainv = Vehicle.Propulsion.PowerFlow.A_inv;
    B = Vehicle.Propulsion.PowerFlow.B;
    B(1:4) = CruisePropPwrkW';
    B(5:6) = LiftPropPwrkW';
    
    Psol = Ainv * B;
    
    % execute the battery model
    DOD = Vehicle.States(1);            % dod integrated from battery dynamics
    UsedREPerc = Vehicle.States(4);            % reversible energy used
    dod = Vehicle.SOTA.Battery.GI_DOD_fcn_usedRE(UsedREPerc); % dod calculated based on this
  
    dod(dod > Vehicle.SOTA.Battery.MaxDOD) = Vehicle.SOTA.Battery.MaxDOD;

    LoadType = 1;                   % power is being specified
    PbattkW = Psol(7);
    Load = PbattkW * 1000;          % pack power, converted from kW to W

    [out,  ddt_BattStates] = EvalBatteryModel(Load,LoadType,dod,Vehicle.SOTA.Battery, Vehicle.Propulsion.Battery);
    
    % get battery power in kilowatt
    PwrPerPack_kW = out.PackPwr_W/1000;
    dEdt_battery_kW = - PbattkW * Vehicle.Propulsion.NBatteryPacks/out.eta;


    ddtStates(1) = ddt_BattStates(1);       % d/dt (DOD)
    ddtStates(2) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(4)/1e6;   % d/dt (Reversible Energy [MW])
    ddtStates(3) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(5)/1e6;   % d/dt (Finite Energy [MW])
    ddtStates(4) = ddtStates(2)/(Vehicle.MassProp.EnergyMass_kg(2).*Vehicle.SOTA.SpecificEnergy_MJkg(2));

    % output dEdt(2) in megawatt (MW)
    dEdt(2) = -ddtStates(2);
    
    OCV = out.OCV;
    VOLT = out.CellVoltage;
    CRATE = out.CRATE;
    ETA = out.eta;
    DE = Vehicle.States(3)/Vehicle.States(2);
    PbattkW = out.PackPwr_W/1000;    
    
end


%% ARCHITECTURE 3: TURBO-ELECTRIC (TE) PROPULSION SYSTEM ARCHITECTURE
if Vehicle.Architecture == 3
    % execute the battery model
    DOD = Vehicle.States(1);            % dod integrated from battery dynamics
    UsedREPerc = Vehicle.States(4);            % reversible energy used
    dod = Vehicle.SOTA.Battery.GI_DOD_fcn_usedRE(UsedREPerc); % dod calculated based on this

    Ainv = Vehicle.Propulsion.PowerFlow.A_cn_inv;
    B = Vehicle.Propulsion.PowerFlow.B;
    B(1:4) = CruisePropPwrkW';
    B(5:6) = LiftPropPwrkW';
    
    Psol = Ainv * B;

    Pts_total_kW = Psol(10);

    dod(dod > Vehicle.SOTA.Battery.MaxDOD) = Vehicle.SOTA.Battery.MaxDOD;

    LoadType = 1;                   % power is being specified
    PbattkW = Psol(7);
    Load = PbattkW * 1000;          % pack power, converted from kW to W

    [out,  ddt_BattStates] = EvalBatteryModel(Load,LoadType,dod,Vehicle.SOTA.Battery, Vehicle.Propulsion.Battery);
    
    % get battery power in kilowatt
    PwrPerPack_kW = out.PackPwr_W/1000;
    dEdt_battery_kW = - PbattkW * Vehicle.Propulsion.NBatteryPacks/out.eta;


    ddtStates(1) = ddt_BattStates(1);       % d/dt (DOD)
    ddtStates(2) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(4)/1e6;   % d/dt (Reversible Energy [MW])
    ddtStates(3) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(5)/1e6;   % d/dt (Finite Energy [MW])
    ddtStates(4) = ddtStates(2)/(Vehicle.MassProp.EnergyMass_kg(2).*Vehicle.SOTA.SpecificEnergy_MJkg(2));

    % output dEdt(2) in megawatt (MW)
    dEdt(2) = -ddtStates(2);
    
    OCV = out.OCV;
    VOLT = out.CellVoltage;
    CRATE = out.CRATE;
    ETA = out.eta;
    DE = Vehicle.States(3)/Vehicle.States(2);
    PbattkW = out.PackPwr_W/1000;
         
    % get turboshaft engine fuel mass flow rate in kg/h
    FuelMassFlowRate_kgh = Vehicle.SOTA.TurboshaftSFC_kgkWh * (Pts_total_kW);
    
    % get rate of change of mass in kg/s
    dmdt(1) = -FuelMassFlowRate_kgh/3600;
    
    % convert fuel mass flow rate to equivalent energy, output dEdt(1)
    dEdt(1) = dmdt(1) * Vehicle.SOTA.FuelSpecificEnergy_MJkg;       
   

end


% ADDITIONAL PARAMETERS OUTPUT FROM THIS FUNCTION
ADDL = zeros(1,19);

ADDL(1:4) = CruisePropPwrkW;
ADDL(5:6) = LiftPropPwrkW;
ADDL(7) = TotPwrkW;

ADDL(8) = LPRPM;
ADDL(9) = BLP;
ADDL(10) = BMP; %CruisePropBeta;
ADDL(11) = LDe; %CruisePropEff;
ADDL(12) = LDRatio;
ADDL(13) = CL;
ADDL(14) = MPRPM;
ADDL(15) = -dmdt(1)*3600;               % kg/h
ADDL(16) = aTS;


Bnet = Controls(20:25).*[Vehicle.Propulsion.CPSetup.HealthStatus Vehicle.Propulsion.LPSetup.HealthStatus]';
ADDL(17) = Bnet(1);
ADDL(18) = Bnet(2);
ADDL(19) = Bnet(3);
ADDL(20) = Bnet(4);
ADDL(21) = Bnet(5);
ADDL(22) = Bnet(6);

ADDL(23) = OCV;
ADDL(24) = VOLT;
ADDL(25) = CRATE;
ADDL(26) = ETA;
ADDL(27) = DE;
ADDL(28) = dod;
ADDL(29) = PbattkW;
ADDL(30) = out.SOC; 
ADDL(31) = UTrimVec(4);

% AddlNames = {'L1kW','L2kW','P1kW','P2kW','P3kW','P4kW','TkW','LRPM','BLP','BCP','LDe','LD','CL','CPRPM','KGH','aTS','B1','B2','B3','B4','B5','B6','OCV','VOLT','CRATE','ETA','DE','dod','PbattkW','SOC'};






stopper = 1;
end


