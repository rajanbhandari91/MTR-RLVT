function [Vehicle] = PowerSizer(Vehicle)

% constants
g = 9.81;
R = 287;
rhoSL = 1.225;

% conversion factors
Conv_kt_to_ms = 0.51444;
Conv_m_to_ft = 3.28;
Conv_ft_to_m = 1/Conv_m_to_ft;

Vehicle.Propulsion.MinEnergyMass_kg = [0,0];


% count the number of point performance constraints to be evaluated
PointPerf = Vehicle.PointPerf.Constraints;
n = height(PointPerf);

DOD = 0.80;
UsedREPerc = Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(DOD);

Vehicle.States = [DOD;0;0;UsedREPerc];

% Loop over each defined point performance constraint

for i = 1:n

    Vehicle.Propulsion.LPSetup.HealthStatus = PointPerf.Health(i,5:6);
    Vehicle.Propulsion.CPSetup.HealthStatus = PointPerf.Health(i,1:4);

    nP(i,1) = sum(PointPerf.Health(i,:));

    % calculate density ratio
    rho = PointPerf.rho(i);
    SIG(i,1) = PointPerf.SIG(i);
    KEAS(i,1) = PointPerf.KEAS(i);
    KTAS(i,1) = PointPerf.KTAS(i);

    V = KTAS(i) * Conv_kt_to_ms;

    % set mass based on MTOM and given weight fraction
    if(PointPerf.WTFRAC(i)>0 && PointPerf.WTFRAC(i)<=1)
        mass(i,1) = Vehicle.MassProp.MTOM_kg * PointPerf.WTFRAC(i);
    else
        mass(i,1) = Vehicle.MassProp.MTOM_kg + PointPerf.WTFRAC(i);
        PointPerf.WTFRAC(i) = mass(i,1)/Vehicle.MassProp.MTOM_kg;
    end

    % set climb rate (in m/s)
    dhdt = PointPerf.FPM(i) * Conv_ft_to_m / 60;

    % set acceleration (in m/s/s)
    vdot = PointPerf.ACC_g(i) * g;

    % evaluation settings
    dt0 = 1;
    EvalType = 'power';
    fpadot = 0;

    if nP(i,1)<6
        EvalType = 'trim';
    end

    % solve the APPM:
    SolverType = 1;
    [~,~,~,~,dt_out,AOA(i,1),CTRL,~,~,ADDL,~,~,FLAG(i,1),~] = SolveFltCon(mass(i),dt0,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle,SolverType);

    dt(i,:) = dt_out;
    PropkW(i,:) = ADDL(1:6);
    kW(i,1) = ADDL(7);
    CL(i,1) = ADDL(13);
    LD(i,1) = ADDL(12);
    vi(i,1) = 0;
    LPRPM(i,:) = ADDL(8);
    CPRPM(i,:) = ADDL(14);
    dcon(i,:) = CTRL;
    BLP(i,:) = ADDL(21:22);
    BCP(i,:) = ADDL(17:20);
    dnac(i,1) = ADDL(31);

end

PPRes.Mass = mass;
PPRes.AOA = AOA*180/pi;
PPRes.dcon = dcon;
PPRes.dt = dt;
PPRes.dnac = dnac;
PPRes.LD = LD;
PPRes.vi = vi;
% PPRes.RPM = RPM;
PPRes.CL = CL;
PPRes.kW = kW;
PPRes.nP = nP;
% PPRes.Nbatt = PointPerf.Batts;
PPRes.FLAG = FLAG;

% PPRes.TR = TR;
% PPRes.BCP = BCP;
PPRes.B1 = BCP(:,1);
PPRes.B2 = BCP(:,2);
PPRes.B3 = BCP(:,3);
PPRes.B4 = BCP(:,4);
PPRes.B5 = BLP(:,1);
PPRes.B6 = BLP(:,2);

PPRes.Pm1kW = PropkW(:,1);
PPRes.Pm2kW = PropkW(:,2);
PPRes.Pm3kW = PropkW(:,3);
PPRes.Pm4kW = PropkW(:,4);
PPRes.Pm5kW = PropkW(:,5);
PPRes.Pm6kW = PropkW(:,6);

PPRes.LPRPM = LPRPM;
PPRes.MPRPM = CPRPM;


PPRes = struct2table(PPRes);

PointPerf = [PointPerf, PPRes];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializations
Vehicle.Propulsion.Turboshaft_Each_kW = 0;
Vehicle.Propulsion.GeneratorPower_Each_kW = 0;
Vehicle.Propulsion.GearboxPower_Each_kW = 0;







% ARCHITECTURE 1: ALL-ELECTRIC ARCHITECTURE (AE)
if Vehicle.Architecture == 1

    % create power sizing cases from point performance table
    PointPerf = PointPerf(Vehicle.PointPerf.PowerSizingCases.Case,:);
    km = Vehicle.PointPerf.PowerSizingCases.km;

   % LIFT MOTOR POWER RATING
    % account for discount factor for short term power loads
    LiftPropkW = [PointPerf.Pm5kW,PointPerf.Pm6kW];
    LiftMotorPower_Each_kW = max(max(LiftPropkW,[],2)./km);
    % update in vehicle structure
    Vehicle.Propulsion.LiftMotorPower_Each_kW = LiftMotorPower_Each_kW;
    Vehicle.DesignPoint.Lift_PMR_kWkg = LiftMotorPower_Each_kW * Vehicle.Propulsion.NLiftProps / Vehicle.MassProp.MTOM_kg;

    CruisePropPower_kW = [PointPerf.Pm1kW,PointPerf.Pm2kW,PointPerf.Pm3kW,PointPerf.Pm4kW];
    Vehicle.DesignPoint.CruisePropPower_kW = max(max(CruisePropPower_kW));
    Vehicle.Propulsion.CruisePropPower_kW = max(max(CruisePropPower_kW));

    %%%%% CRUISE MOTOR POWER RATING
    CruiseMotorPower_kW = max(max(CruisePropPower_kW));
    Vehicle.Propulsion.CruiseMotorPower_Each_kW = CruiseMotorPower_kW;
    Vehicle.DesignPoint.Cruise_PMR_kWkg = Vehicle.Propulsion.CruiseMotorPower_Each_kW * Vehicle.Propulsion.NCruiseProps / Vehicle.MassProp.MTOM_kg;

    PropkW = [CruisePropPower_kW LiftPropkW];

    n = height(PointPerf);
    NBatt = Vehicle.PointPerf.PowerSizingCases.Batts;
    P1kW = zeros(n,1);
    PbattkW = zeros(n,1);

    for i = 1:n
        A = Vehicle.Propulsion.PowerFlow.A;
        A(8,7) = NBatt(i);

        FltSol = Vehicle.Propulsion.PowerFlow.B;
        FltSol(1:6) = PropkW(i,:)';

        Psol = EvalPowerFlow(A, [], FltSol);

        P1kW(i,1) = Psol(8);
        PbattkW(i,1) = Psol(7);
    end

    PointPerf.P1kW = P1kW;
    PointPerf.PbattkW = PbattkW;

    [Vehicle, PointPerf] = BatterySizer(Vehicle, PbattkW, PointPerf);

    PointPerf.km = km;
    PointPerf.Nbatt = NBatt;

    % Battery peak power
    Vehicle.Propulsion.BatteryPackPeakPower_kW = max(PbattkW);
    % determine min battery mass per pack based on this peak power
    MinBatteryMass_kg = Vehicle.Propulsion.NBatteryPacks * Vehicle.Propulsion.Battery.MinPackMass_kg;
    Vehicle.Propulsion.MinEnergyMass_kg(2) = MinBatteryMass_kg;

    fprintf('\n Min BATT mass based on power req.: %0.0f kg\n',MinBatteryMass_kg)

end


% ARCHITECTURE 3: TURBO-ELECTRIC ARCHITECTURE (TE)
if Vehicle.Architecture == 3

    % create power sizing cases from point performance table
    PointPerf = PointPerf(Vehicle.PointPerf.PowerSizingCases.Case,:);
    km = Vehicle.PointPerf.PowerSizingCases.km;
    NBatt = Vehicle.PointPerf.PowerSizingCases.NBatt;

    % LIFT MOTOR POWER RATING
    % account for discount factor for short term power loads
    LiftPropkW = [PointPerf.Pm5kW,PointPerf.Pm6kW];
    LiftMotorPower_Each_kW = max(max(LiftPropkW,[],2)./km);
    % update in vehicle structure
    Vehicle.Propulsion.LiftMotorPower_Each_kW = LiftMotorPower_Each_kW;
    Vehicle.DesignPoint.Lift_PMR_kWkg = LiftMotorPower_Each_kW * Vehicle.Propulsion.NLiftProps / Vehicle.MassProp.MTOM_kg;

    CruisePropPower_kW = [PointPerf.Pm1kW,PointPerf.Pm2kW,PointPerf.Pm3kW,PointPerf.Pm4kW];
    Vehicle.DesignPoint.CruisePropPower_kW = max(max(CruisePropPower_kW));
    Vehicle.Propulsion.CruisePropPower_kW = max(max(CruisePropPower_kW));

    %%%%% CRUISE MOTOR POWER RATING
    CruiseMotorPower_kW = max(max(CruisePropPower_kW));
    Vehicle.Propulsion.CruiseMotorPower_Each_kW = CruiseMotorPower_kW;
    Vehicle.DesignPoint.Cruise_PMR_kWkg = Vehicle.Propulsion.CruiseMotorPower_Each_kW * Vehicle.Propulsion.NCruiseProps / Vehicle.MassProp.MTOM_kg;

    PropkW = [CruisePropPower_kW LiftPropkW];
    Ntg= Vehicle.PointPerf.PowerSizingCases.TG;

    n = height(PointPerf);
    PtskW = zeros(n,1);
    PgbkW = zeros(n,1);
    PgenkW = zeros(n,1);
    P2kW = zeros(n,1);
    P1 = zeros(n,1);
    for i = 1:n
            if Ntg(i) == 1
                A = Vehicle.Propulsion.PowerFlow.A_cn;
                FltSol = Vehicle.Propulsion.PowerFlow.B;
                FltSol(1:6) = PropkW(i,:)';
            else
                A = Vehicle.Propulsion.PowerFlow.A_os;
                FltSol = Vehicle.Propulsion.PowerFlow.B;
                FltSol(1:6) = PropkW(i,:)';
            end

            Psol = EvalPowerFlow(A, [], FltSol);

            PtskW(i,1) = Psol(10);
            PgbkW(i,1) = Psol(11);
            PgenkW(i,1) = Psol(12);
            P2kW(i,1) = Psol(9);
            PbattkW(i,1) = Psol(7);
            P1(i,1) = Psol(8);

    end

    % turboshaft power lapse
    PowerLapse = Vehicle.Propulsion.TLinterp.TLinterp(PointPerf.KTAS,PointPerf.DA_ft);
    PointPerf.PowerLapse = PowerLapse;
    PointPerf.TSkW = PtskW./PowerLapse;
    PointPerf.GBkW = PgbkW;
    PointPerf.GENkW = PgenkW;
    PointPerf.P2kW = P2kW;
    PointPerf.PbattkW = PbattkW;
    PointPerf.P1 = P1;

    [Vehicle, PointPerf] = BatterySizer(Vehicle, PbattkW, PointPerf);

    % find turboshaft power rating
    Vehicle.Propulsion.Turboshaft_Each_kW = max(PointPerf.TSkW);

    % set BSFC of turboshaft engines
    Vehicle.SOTA.TurboshaftSFC_lbhph = 1.8784 * (Vehicle.Propulsion.Turboshaft_Each_kW/0.746)^(-0.173);
    Vehicle.SOTA.TurboshaftSFC_kgkWh = Vehicle.SOTA.TurboshaftSFC_lbhph * (0.4536/0.746);

    % get sizing generator power
    Vehicle.Propulsion.GeneratorPower_Each_kW = max(PointPerf.GENkW);

    % get sizing gearbox power
    Vehicle.Propulsion.GearboxPower_Each_kW = max(PointPerf.GBkW);

    PointPerf.km = km;
    PointPerf.Ntg = Ntg;
    PointPerf.Nbatt = NBatt;

    % Battery peak power
    Vehicle.Propulsion.BatteryPackPeakPower_kW = max(PbattkW);
    % determine min battery mass per pack based on this peak power
    MinBatteryMass_kg = Vehicle.Propulsion.NBatteryPacks * Vehicle.Propulsion.Battery.MinPackMass_kg;
    % EmerBatteryMass_kg = Vehicle.Propulsion.NBatteryPacks * Vehicle.Propulsion.Battery.MinPackMass_kg;
    % Vehicle.Propulsion.EmerBatteryMass_kg = EmerBatteryMass_kg;
    Vehicle.Propulsion.MinEnergyMass_kg(2) = MinBatteryMass_kg;

    fprintf('\n Min BATT mass based on power req.: %0.0f kg\n',MinBatteryMass_kg)

end

disp(PointPerf);

% restore all lift rotors to active status
Vehicle.Propulsion.LPSetup.HealthStatus = Vehicle.Propulsion.LPSetup.HealthStatus*0 + 1;
Vehicle.Propulsion.CPSetup.HealthStatus = Vehicle.Propulsion.CPSetup.HealthStatus*0 + 1;
Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter} = PointPerf;

    function psol = EvalPowerFlow(A, Ainv, FltSol)

        % if A is given instead of A-inverse, find A-inverse
        if isempty(Ainv)
            Ainv = inv(A);
        end

        rhs = FltSol;

        psol = (Ainv * rhs)';


    end


end