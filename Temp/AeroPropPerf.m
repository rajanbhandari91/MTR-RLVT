function [Fxap,Fzap,M_out,dmdt,dEdt,ADDL,ddtStates] = AeroPropPerf(AOA,dcon,dt,rho,V,fpa,Vehicle)

% AOA = 15*pi/180

% initializations
dmdt = [0,0];
dEdt = [0,0];

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


% set CG location to target CG
r_cg = Vehicle.MassProp.TargetCG;
x_cg = r_cg(1); y_cg = r_cg(2); z_cg = r_cg(3);


% nominal tip mach
RPM_high = Vehicle.Propulsion.Setup.DesignCases.RPM + 0;
RPM_low  = Vehicle.Propulsion.Setup.DesignCases.RPM + 0;
RPMBounds = [RPM_high RPM_low];

RPM = min(RPMBounds(2), max(RPMBounds(1),  RPMBounds(2) - (RPMBounds(2)-RPMBounds(1))*(V/0.5144)/(Vehicle.Operations.VC)));



% dt(2): '0', can be used as switch to turn and off the HLPs
SpdBrk = 0;

%%% propeller operation
% power supplied
dt_eff = dt(1);
dt_eff(dt_eff<0) = 0;
TG_totalPkW = Vehicle.Propulsion.NTurbogenerators * Vehicle.Propulsion.Turboshaft_Each_kW * Vehicle.Propulsion.TurboshaftLapse_rho(rho);
Batt_totalPkW = Vehicle.Propulsion.NBatteryPacks * Vehicle.Propulsion.BatteryPackPeakPower_kW;
TotalPsupplied = (TG_totalPkW + Batt_totalPkW) * dt_eff * 1000;
% Psupplied to motor
Psupplied = TotalPsupplied/(sum(Vehicle.Propulsion.Setup.HealthStatus));     % also accounts for prop failure
% calculate torque to each cruise prop based on power supplied
Qsupplied = Psupplied/(2*pi*RPM/60);
% calculate the torque coefficient
CQ = Qsupplied/(rho * (RPM/60)^2 * Vehicle.Propulsion.PropDiam_m^5);
% compute advance ratio
J = V/((RPM/60)*Vehicle.Propulsion.PropDiam_m);
% calculate prop pitch for power balance
B0 = Vehicle.Propulsion.PropCurves.GI_beta_CQ_J(real(CQ),real(J));



% States required by FMComp
States.AirData.AOADeg = AOA * 180/pi;
% warning('beta hardcode AeroPropPerf.m')
States.AirData.BETADeg = 0;                                    % symmetric flight condition
States.AirData.BETA = States.AirData.BETADeg *pi/180;          % symmetric flight condition

States.AirData.TAS = V;
States.AirData.uw  = V * cos(AOA) * cosd(States.AirData.BETADeg);
States.AirData.vw  = V * sind(States.AirData.BETADeg);                  % symmetric flight condition
States.AirData.ww  = V * sin(AOA) * cosd(States.AirData.BETADeg);

States.AirData.pw  = 0;                  % quasi-steady
States.AirData.qw  = 0;                  % quasi-steady
States.AirData.rw  = 0;                  % quasi-steady

% the second element in the vector should be fpa_dot
if length(fpa)>1 
    States.AirData.qw = fpa(2);
end

States.AirData.a =  FlowCon.SpeedOfSound;   % note: this is a hardcode ATM
States.AirData.qbar = 0.5 * rho * V^2;
States.AirData.rho = rho;               % air density
States.Attitude.ph = 0;                 % symmetric flight condition
States.Attitude.th = AOA;                 % only reqd for gravity load calc


% Trim Vector Inputs
UTrimVec = [...
    ulat
    ulong
    udir
    B0
    RPM
    SpdBrk];


% brk = 0:0.1:1;
% for i=1:length(brk)
% UTrimVec(6) = brk(i);

% Call the control allocator function [same function to be used during simulation]
[Controls] = ControlAllocator(UTrimVec, FltCon, Vehicle);


% leave XInflow blank. This will trigger internal iterations to solve lift
% rotor inflow within FMComp
XInflow = [];

% mass: set to zero. This will result in gravitational load computed by
% FMComp coming out to [0;0;0]. This is fine, since within the sizing code,
% gravitational loads are computed elsewhere.
m = 0;

% CG location (r_cg):
EvalMode = 2;


[Fbf,Mbf,Hr,Hrdot,AddlRes,Xrotor_dot,AddlOutputs,ppdata] = FMComp(States,Controls,XInflow,Vehicle,m,r_cg,EvalMode);

% force(i,:) = Fbf;
% moment(i,:) = Mbf;
% end

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



% AddlOutputs = [PropPwrkW, FanPwrkW, TotPwrkW, CL, CD, LDRatio, Cmy, vi];
%
NProps = Vehicle.Propulsion.NProps;

PropPwrkW        = AddlOutputs(1:NProps);
TotPwrkW         = AddlOutputs(NProps+1);
CL               = AddlOutputs(NProps+2);
CD               = AddlOutputs(NProps+3);
LDRatio          = AddlOutputs(NProps+4);
CMy              = AddlOutputs(NProps+5);
Vi               = AddlOutputs(NProps+6);
MaxThrustPerProp = AddlOutputs(NProps+7);




%%--------- dEdt and dmdt ---------%%
dmdt(1) = -00/3600;
dEdt(1) = 0;

dEdt(2) = 0;

aTS = 0;

%% ARCHITECTURE 3: TURBO-ELECTRIC (TE) PROPULSION SYSTEM ARCHITECTURE
if Vehicle.Architecture == 2

    % extract battery states
    DOD = Vehicle.States(1);                   % dod integrated from battery dynamics
    UsedREPerc = Vehicle.States(4);            % reversible energy used
    dod = Vehicle.SOTA.Battery.GI_DOD_fcn_usedRE(UsedREPerc); % dod calculated based on this
    dod(dod > Vehicle.SOTA.Battery.MaxDOD) = Vehicle.SOTA.Battery.MaxDOD;

    B = Vehicle.Propulsion.PowerFlow.B ;
    B(1:NProps) = PropPwrkW';

    % find available turboshaft power at this flight condition
    Pts_av_kW = Vehicle.Propulsion.Turboshaft_Each_kW * Vehicle.Propulsion.TurboshaftLapse_rho(rho);

    % detect climb during mission sizing, and based on power-split use approrpiate power from TS
    % 
    % if dod > maxDOD, switch TS to full capacity
    if (Vehicle.Operations.Climb_1) && (dod < Vehicle.SOTA.Battery.MaxDOD) % && (fpa(1)*180/pi >= 1) 
        Pts_av_kW = Vehicle.DesVar.perc_TS * Pts_av_kW;
    end

    % X = inv(A) * B
    Psol = Vehicle.Propulsion.PowerFlow.A_nom_inv * B ;

    % turboshaft power requirement
    Pts_req_kW = Psol(NProps + 9) ;

    if Pts_req_kW > Pts_av_kW
        % set turboshafts to max available power
        B(NProps + 11) = Pts_av_kW ; B(NProps + 12) = Pts_av_kW ;
        Psol = Vehicle.Propulsion.PowerFlow.A_off_inv * B ;
    end

    if Pts_req_kW < Pts_av_kW 

        if dod > Vehicle.Mission.EndDOD
            RecRate = 2.0; % 1-C
            RecGain = (dod - Vehicle.Mission.EndDOD) * RecRate; 
            RecGain = max(0,min(RecGain,1.0));
        else
            RecGain = 0; 
        end        
       
        B(NProps + 12) = (Pts_av_kW - Pts_req_kW) * RecGain; % recharge using  extra available power
        Psol  = Vehicle.Propulsion.PowerFlow.A_rc_inv * B;
    end

    Pts1_kW = Psol(NProps + 9);
    Pts2_kW = Psol(NProps + 10);
    PbattkW = Psol(NProps + 11);

    Pts_total_kW = Pts1_kW + Pts2_kW;

   
    LoadType = 1;                   % power is being specified
    Load = PbattkW * 1000;          % pack power, converted from kW to W

    [out,  ddt_BattStates] = EvalBatteryModel(Load,LoadType,dod,Vehicle.SOTA.Battery, Vehicle.Propulsion.Battery);

    % get battery power in kilowatt
    PwrPerPack_kW   =   out.PackPwr_W/1000;
    dEdt_battery_kW = - PbattkW * Vehicle.Propulsion.NBatteryPacks/out.eta;


    ddtStates(1) = ddt_BattStates(1);       % d/dt (DOD)
    ddtStates(2) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(4)/1e6;   % d/dt (Reversible Energy [MW])
    ddtStates(3) = Vehicle.Propulsion.NBatteryPacks * ddt_BattStates(5)/1e6;   % d/dt (Finite Energy [MW])
    ddtStates(4) = ddtStates(2)/(Vehicle.MassProp.EnergyMass_kg(2).*Vehicle.SOTA.SpecificEnergy_MJkg(2));

    % output dEdt(2) in megawatt (MW)
    dEdt(2) = -ddtStates(2);

    % convert to megawatt and output dEdt(2)
    % dEdt(2) = dEdt_battery_kW/1000;

    OCV = out.OCV;
    VOLT = out.CellVoltage;
    CRATE = out.CRATE;
    ETA = out.eta;
    DE = Vehicle.States(3)/Vehicle.States(2);
    PbattkW = out.PackPwr_W/1000;
    SOC = out.SOC;


    % get turboshaft engine fuel mass flow rate in kg/h
    SFC_kgkWh = Vehicle.SOTA.TurboshaftSFC_kgkWh_kW_rho(Pts1_kW, rho);
    FuelMassFlowRate_kgh = SFC_kgkWh * Pts_total_kW;


    % get rate of change of mass in kg/s
    dmdt(1) = -FuelMassFlowRate_kgh/3600;

    % convert fuel mass flow rate to equivalent energy, output dEdt(1)
    dEdt(1) = dmdt(1) * Vehicle.SOTA.FuelSpecificEnergy_MJkg;

end



% ADDITIONAL PARAMETERS OUTPUT FROM THIS FUNCTION

ADDL(1:NProps) = PropPwrkW;
ADDL(NProps+1) = TotPwrkW;

ADDL(NProps+2) = RPM;
ADDL(NProps+3) = B0;
ADDL(NProps+4) = CL;
ADDL(NProps+5) = CD;
ADDL(NProps+6) = LDRatio;
ADDL(NProps+7) = CMy;
ADDL(NProps+8) = Vi;
ADDL(NProps+9) = -dmdt(1)*3600; % kg/h

ADDL(NProps+10) = MaxThrustPerProp;

ADDL(NProps+11) = OCV;
ADDL(NProps+12) = VOLT;
ADDL(NProps+13) = CRATE;
ADDL(NProps+14) = ETA;
ADDL(NProps+15) = DE;
ADDL(NProps+16) = dod;
ADDL(NProps+17) = SOC;
ADDL(NProps+18) = PbattkW;
ADDL(NProps+19) = Pts1_kW;
ADDL(NProps+20) = Pts2_kW;

end


