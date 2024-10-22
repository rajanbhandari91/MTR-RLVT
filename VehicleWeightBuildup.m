function [Vehicle] = VehicleWeightBuildup(Vehicle)


% fprintf('RECEIVED, MTOM: %0.0f, E1: %0.0f kg, E2: %0.0f kg',Vehicle.MassProp.MTOM_kg,Vehicle.MassProp.EnergyMass_kg(1), Vehicle.MassProp.EnergyMass_kg(2));
IncomingMTOM_kg = Vehicle.MassProp.MTOM_kg;

%% conversion factors
Conv_m_to_ft = 3.28;
Conv_m2_to_ft2 = 10.7639;
Conv_lb_to_kg = 0.4536;
Conv_kg_to_lb = 1/Conv_lb_to_kg;
Conv_m_to_inch = 39.3701;
Conv_kW_to_hp = 1.34102;
Conv_Pa_to_psf = 0.021;
Conv_WL_to_eng = 4.88243;

%% OPERATIONAL PARAMETERS
Geom = Vehicle.Geom;

EWMARG = Vehicle.MassProp.EW_Margin;


EnergyMass_kg = Vehicle.MassProp.EnergyMass_kg;

Payload_kg = Vehicle.DesignPoint.Payload;

WTO = Vehicle.MassProp.MTOM_kg* Conv_kg_to_lb;

% Vehicle.Oper
nult = Vehicle.Operations.nult;                                             % ultimate load factor = 1.5 x limit load factor - FIXED
VC = Vehicle.Operations.VC;                                                 % design cruise speed, kt
VmaxSLkt = Vehicle.Operations.VmaxSLkt;                                     % max level fligWing_5 speed at sea level, kt
VD = Vehicle.Operations.VD;                                                 % design dive speed, kt
NPAX = Vehicle.Operations.NPAX;                                             % number of passengers, including crew

q = (0.5*1.225*(VC*0.5144)^2) * Conv_Pa_to_psf;                             % dynamic pressure at cruise (lb/sqft)

%% Convergence Parameters
%conv = Vehicle.MassProp.Convergence;                                        % convergence flag = 1 if iterations have converged
% GWTolerance = Vehicle.MassProp.GWTolerance;                                 % gross WEIGHT convergence tolerance
iter = 0;                                                                   % iteration counter


%% PARAMETER INITIALIZATIONS
% these parameters are used for some downstream calculations
FS_lb = 0;          % fuel system weight, also used for some other calcs

%% MASS CONVERGENCE ITERATIONS
iter = 0;
OEM_conv = 0;
iter_max = 10;
OldOEM = 0;
OEMConvergenceThreshold_kg = .1;

while OEM_conv==0 && iter<iter_max

    iter = iter + 1;
    
    W_dg = 1*WTO;   % design gross weight, set to WTO
    Wldg = WTO;     % landing gweight, set to WTO

    %% Fuselage Weight
    Lfn = Geom.Fuselage.Length * Conv_m_to_ft;                                   % fuselage length not including nose mounted nacelle, in feet (does not include booms    
    lh = 1;                                                                      % distance from wing c/4 to HTail c/4 in ft - FIXED --> set to 1 as there is no connection between fuselage and HT for LPC-03

    wf = max(Geom.Fuselage.Stn.Width) * Conv_m_to_ft;                            % max fuselage width, ft
    hf = max(Geom.Fuselage.Stn.Height) * Conv_m_to_ft;                           % max fuselage heigWing_5, ft
    Sfus = Geom.Fuselage.Stn.sdA_Wet(end) * Conv_m_to_ft^2;                      % fuselage wetted area in sqft

    W_Fus_lb = EvalFusWeight_lb(WTO,nult,Lfn,wf,hf,VC,Sfus,W_dg,lh,q);
    Geom.Fuselage.Mass = W_Fus_lb * Conv_lb_to_kg;


    %% Wing Weight AFDD Model
    
    %%%% Pack Inputs %%%%

    WingPack(1) = Vehicle.Geom.LWing.MAC * 0.4; % width of wing structural attachements to body {m}    <<<TEST VALUE>>>
    WingPack(2) = 0.2; % pylon radius of gyration {m}    <<<TEST VALUE>>>
    WingPack(3) = Vehicle.MassProp.MTOM_kg / 2 * 9.81; % maximum thrust of one rotor {N}
    WingPack(4) = Vehicle.Propulsion.RotorPropRPM * 60 * 2*pi; % rotor speed for wing weight design condition {rad/sec}

    WingPack(5) = Vehicle.MassProp.MTOM_kg * 9.81; % structural design gross weight {N}
    WingPack(6) = 0; % tip mass at end of wing {kg}    <<<TEST VALUE>>>

    WingPack(7) = 1; % wing torsion mode frequency (fraction rotor speed) {per rev}    <<<TEST VALUE>>>
    WingPack(8) = 1; % wing beam bending mode frequency (fraction rotor speed) {per rev}    <<<TEST VALUE>>>
    WingPack(9) = 1; % wing chord bending mode frequency (fraction rotor speed) {per rev}    <<<TEST VALUE>>>

    WingPack(10) = 1; % unit weight of leading and trailing edge fairings {kg/m^2}    <<<TEST VALUE>>>
    WingPack(11) = 1; % unit weight of control surfaces {kg/m^2}    <<<TEST VALUE>>>
    WingPack(12) = 1; % unit weight of wing extenstion {kg/m^2}    <<<TEST VALUE>>>

    WingPack(13) = sum(Vehicle.Geom.LWing.CS_Area); % area of control surfaces {m^2}
    WingPack(14) = 0; % area of wing extensions {m^2}
    WingPack(15) = Vehicle.Geom.LWing.PlanformArea; % wing planform area {m^2}
    WingPack(16) = Vehicle.Geom.LWing.AspectRatio; % wing aspect ratio
    WingPack(17) = Vehicle.Geom.LWing.Sweep_qc_r2t; % wing sweep angle {deg}
    WingPack(18) = Vehicle.Geom.LWing.TaperDefn(2,1) / Vehicle.Geom.LWing.TaperDefn(2,end); % wing taper ratio
    WingPack(19) = Vehicle.Geom.LWing.Span; % wing span (length of torque box) {m}
    WingPack(20) = Vehicle.Geom.LWing.MAC; % wing chord {m}
    WingPack(21) = 0; % fraction of wing span that folds (0 to 1)
    WingPack(22) = Vehicle.Geom.LWing.ExposedSpan; % wing length (minus fuselage width) {m}
    WingPack(23) = Vehicle.Geom.LWing.Thickness_to_chord; % wing airfoil thickness-to-chord ratio
    WingPack(24) = 0.4; % torque box chord to wing chord ratio    <<<TEST VALUE>>>
    WingPack(25) = 3; % design ultimate flight load factor at W_SD {g}    <<<TEST VALUE>>>

    WingPack(26) = 1; % torque box modulus {N/m^2}    <<<TEST VALUE>>>
    WingPack(27) = 1; % spar modulus {N/m^2}    <<<TEST VALUE>>>

    WingPack(28) = 1; % ultimate strain allowable (min of spar and torque box)    <<<TEST VALUE>>>
    WingPack(29) = 1; % torque box shear modulus {N/m^2}    <<<TEST VALUE>>>

    WingPack(30) = 1; % structural efficiency factor (torque box)    <<<TEST VALUE>>>
    WingPack(31) = 1; % structural efficiency factor (spars)    <<<TEST VALUE>>>

    WingPack(31) = 1; % density of torque box material {kg/m^3}    <<<TEST VALUE>>>
    WingPack(32) = 1; % density of spar material {kg/m^3}    <<<TEST VALUE>>>

    WingPack(33) = 1; % strength correction for spar taper (equivelant stiffness)    <<<TEST VALUE>>>
    WingPack(34) = 1; % weight correction for spar taper (equivelant strength)    <<<TEST VALUE>>>
    WingPack(35) = 1; % weight correction for spar taper (equivalent siffness)    <<<TEST VALUE>>>

    WingPack(36) = 0.01; % wing fittings and brackets (fraction maximum thrust of one rotor)    <<<TEST VALUE>>>
    WingPack(37) = 0.004; % wing fold/tilt (fraction total weight excluding fold, including weight on tips)    <<<TEST VALUE>>>
    WingPack(38) = 0.2; % wing extension fold/tilt (fraction extenstion weight)    <<<TEST VALUE>>>

    WingPack(39) = 1; % X_prim
    WingPack(40) = 1; % X_flap
    WingPack(41) = 1; % X_fair
    WingPack(42) = 1; % X_fit
    WingPack(43) = 1; % X_fold
    WingPack(44) = 1; % X_efold
    WingPack(45) = 1; % X_ext

    WingPack(46) = 0; % 1 for tilt rotor or tiltwing 0 for fixed
    WingPack(47) = 0; % 1 for jump takeoff reqiurement 0 for else
    WingPack(48) = 0; % 0 for landing gear on wing 1 for otherwise


    [Main_Wings_kg] = EvalWingWEIGHT(WingPack);

    % dividing this mass by 2, and assign to left and right wings
    Geom.LWing.Mass = Main_Wings_kg / 2;
    Geom.RWing.Mass = Main_Wings_kg / 2;

    %% Horizontal Tail Weight
    Sh = 2*Vehicle.Geom.RHTail.PlanformArea * Conv_m_to_ft^2;                        % horizontal tail area, ft2
    bh = 2*Vehicle.Geom.RHTail.Span * Conv_m_to_ft;                           % horizontal tail span, ft
    TOCh = Geom.RHTail.Thickness_to_chord;                                        % thickness to chord ratio

    lam25h = mean(Geom.RHTail.Stn.Sw_25);                                           % quarter-chord sweep

    TRh = Geom.RHTail.TaperDefn(end);                                               % horizontal tail taper ratio

    lhtail = Vehicle.Recalcs.HTailMomentArm_m * Conv_m_to_ft;                        % tail moment arm, ft

    HTRootThickness = TOCh * Vehicle.Geom.RHTail.Stn.c(1) * Conv_m_to_ft;    % max thickness of root in ft - computed
    Ah = bh^2/Sh;                                                           % horizontal tail aspect ratio, computed

    % evaluate HTail weight in lb
    W_HTAIL_lb = EvalHTailWeight_lb(WTO,nult,Sh,lhtail,bh,HTRootThickness,W_dg,q,TOCh,Ah,lam25h,TRh);
    Geom.RHTail.Mass = W_HTAIL_lb/2*Conv_lb_to_kg;
    Geom.LHTail.Mass = W_HTAIL_lb/2*Conv_lb_to_kg;

    %% Vertical Tail Weight
    nVT = 2;                                                                % number of vertical tails
    Sv = Vehicle.Recalcs.VTailArea_Each_m2 * Conv_m_to_ft^2;             % vertical tail planform area, ft2 - LINK
    bv = Vehicle.Recalcs.VertStabSpan_m * Conv_m_to_ft;                     % vertical tail span, ft - LINK
    lam25v = mean(Geom.VTail.Stn.Sw_25);                                  % quarter-chord sweep
    
    TOCv = Geom.VTail.Thickness_to_chord;                                 % vertical tail thickness-to-chord ratio - FIXED
    TRv = mean(Geom.VTail.TaperDefn(2,:));                                % vertical tail taper ratio - FIXED
    
    Ht_Hv = 0; %Geom.RHTail.EtaMounting;                                    % 0 for conventional tail, 1 for T tail
    
    VTRootThickness = TOCv * Vehicle.Geom.VTail.Stn.c(1) * Conv_m_to_ft;  % max thickness at root in ft - computed
    Av = bv^2/Sv;                                                           % vertical tail aspect ratio - computed
    
    [W_VT_lb] = EvalVTWEIGHT(nVT, Sv, bv, lam25v, TOCv, TRv, Ht_Hv, VTRootThickness, Av,q,W_dg,nult,WTO);
    
    % assign this WEIGHT to VT
    Geom.VTail.Mass = W_VT_lb * Conv_lb_to_kg;

    %% Boom Weight                                                          % based on tailboom weight estimation, Roskam rpg 1525
    
    % Skin Weight
    rho_AL = 2700;                                                          % 6061 Aluminium Alloy Density - 0.0975 lb/in^3
    SkinThickness_m = 0.75/1000;

    L_OBD_BOOM_Wetted_Area_m = 2*Vehicle.Geom.Pylons_1.Stn.sdA_Wet(end);
    R_OBD_BOOM_Wetted_Area_m = 2*Vehicle.Geom.Pylons_2.Stn.sdA_Wet(end);
          
    
    SkinWeight_kg_L_OBD_Boom = L_OBD_BOOM_Wetted_Area_m * SkinThickness_m * rho_AL;
    SkinWeight_kg_R_OBD_Boom = R_OBD_BOOM_Wetted_Area_m * SkinThickness_m * rho_AL;

    %Load Information
    W = Vehicle.MassProp.MTOM_kg*9.81;
    
    % Computing mass per unit length based on the rotor loads of the left
    % IBD boom and then assigning the weights of each boom based on their
    % lengths. 

    T(1) = W/2; T(2) = W/2; 

    % length of rotors (2 and 6) from the CG of left IBD boom
    L(1) = 0.5*(Vehicle.Geom.Prop_2.RefPtLocation(1) - Vehicle.Geom.Prop_5.RefPtLocation(1));
    L(2) = 0.5*(Vehicle.Geom.Prop_3.RefPtLocation(1) - Vehicle.Geom.Prop_6.RefPtLocation(1));


    LoadBumpUpFactor = 1.0;
    MassBumpUpFactor = 1.15;
    
    Mb = LoadBumpUpFactor * (T.*L);

    % Picking up maximum bending moment to compute mass per unit length
    
    MbMax = max(abs(Mb));
    
   
    h = 0.8*Vehicle.Geom.Pylons_1.MaxHeight;          % m
    b = 0.2*Vehicle.Geom.Pylons_2.MaxWidth;           % m  
    t = [1:0.05:5]/1000;            % m, test thicknesses                            
    FOSTarget = 1.5;                % factor of safety target
    flag = 2;
    % inputs: (i) M: root BM (Nm), (ii) h: height (m), 
    % (iii) b: width (m), (iv): test thicknesses (m)
    % (v) flag: =1 for rectangular, = 2 for I-beam
    [M_L,FOS, tsel, defl] = BoomSizeFunc(MbMax, max(T), max(L), h,b,t,FOSTarget,flag);
    % outputs: (i) M_L: mass/length (kg/m) for test thickneses
    % (ii) FOS: factor of safety for test thicknesses

    % Boom Dimension Struct     
    L_OBD_Boom_len = 2*Vehicle.Geom.Pylons_1.Length;
       
    % Summing the skin loads and structure loads
    Geom.Pylons_1.Mass = 0.5*(L_OBD_Boom_len*M_L*MassBumpUpFactor + SkinWeight_kg_L_OBD_Boom);
    Geom.Pylons_2.Mass = 0.5*(L_OBD_Boom_len*M_L*MassBumpUpFactor + SkinWeight_kg_L_OBD_Boom);
    
    Geom.Pylons_1.thickness = tsel;
    Geom.Pylons_2.thickness = tsel;
  
    Geom.Pylons_1.ML_kgm = Geom.Pylons_1.Mass/(0.5*L_OBD_Boom_len);
    Geom.Pylons_2.ML_kgm = Geom.Pylons_2.Mass/(0.5*L_OBD_Boom_len);
    
    %% Landing Gear Weight AFDD Model%%

    %%%% Pack Inputs %%%%

    LG_Pack(1)  = Vehicle.MassProp.MTOM_kg * Conv_kg_to_lb; % maximum takeoff weight {lb}
    LG_Pack(2)  = Vehicle.DesignPoint.WL_kgm2 * Conv_kg_to_lb / Conv_m2_to_ft2; % wing loading (1.0 for helicopter) {lb/ft^2}
    LG_Pack(3)  = 3; % number of landing gear assemblies

    LG_Pack(6)  = 0.0325; % landing gear weight (fraction maximum takeoff weight)
    LG_Pack(7)  = 0.08;   % retraction weight (fraction basic landing gear weight)
    LG_Pack(8)  = 0.14;   % crashworthiness weight (fraction basic landing gear weight)

    LG_Pack(9)  = 1; % landing gear factor (0 if landing gear not present, else 1)
    LG_Pack(10) = 0; % retraction factor (0 if gear is fixed, else 1)
    LG_Pack(11) = 0; % crashworthiness factor (0 if gear is not rated for crash, else 1)

    [~, W_LDG_each_lb] = EvalLDGWeight_lb(LG_Pack);
    
    Geom.LandingGear_Main_1.Mass = W_LDG_each_lb * Conv_lb_to_kg; % NDARC method does not differentiate between nose and main gear.
    Geom.LandingGear_Main_2.Mass = W_LDG_each_lb * Conv_lb_to_kg; % This could lead to innacuracies in the weight distribution
    Geom.LandingGear_Nose.Mass   = W_LDG_each_lb * Conv_lb_to_kg; % as calculated here.

    %% Nacelle Weight AFDD Model %%

    %%%% Outboard Nacelle Weight %%%%
    % Nacelles 1 and 4

    %%%% Pack Inputs %%%%
    
    NacelleOutPack(1)  = Vehicle.MassProp.MTOM_kg * Conv_kg_to_lb; % maximum takeoff weight {lb}
    NacelleOutPack(2)  = Vehicle.Geom.Turboshaft.Mass * Conv_kg_to_lb; % weight all main enignes {lb}
    NacelleOutPack(3)  = 1; % number of main engines
    NacelleOutPack(4)  = Geom.Nacelle_1.Stn.sdA_Wet(end) * Conv_m2_to_ft2; % wetted area of nacelles and pylons (less spinner) {ft^2}

    NacelleOutPack(5)  = 0.3;  % air induction weight (fraction nacelle plus air induction) (range: 0.1-0.6)
    NacelleOutPack(6)  = .005; % pylon support structure weight (fraction W_MTO)

    NacelleOutPack(7)  = 1; % engine support factor (1 if supports present, else 0)
    NacelleOutPack(8)  = 1; % engine cowl factor (1 of cowl present, else 0)
    NacelleOutPack(9)  = 1; % pylon support structure factor (1 if pylon present, else 0)
    NacelleOutPack(10) = 1; % air induction factor (1 if air induction system present, else 0)

    [W_nacelle_out, W_airind_out] = EvalNacelleWeight(NacelleOutPack);

    Geom.Naccelle_1.Mass = (W_nacelle_out + W_airind_out) * Conv_lb_to_kg;
    Geom.Naccelle_4.Mass = (W_nacelle_out + W_airind_out) * Conv_lb_to_kg;

    %%%% Inboard Nacelle Weight %%%%
    % Nacelles 2, 3, 5, and 6

    %%%% Pack Inputs %%%%
    
    NacelleInPack(1)  = Vehicle.MassProp.MTOM_kg * Conv_kg_to_lb; % maximum takeoff weight {lb}
    NacelleInPack(2)  = Vehicle.Geom.Turboshaft.Mass * Conv_kg_to_lb; % weight all main enignes {lb}
    NacelleInPack(3)  = 1; % number of main engines
    NacelleInPack(4)  = Geom.Nacelle_2.Stn.sdA_Wet(end) * Conv_m2_to_ft2; % wetted area of nacelles and pylons (less spinner) {ft^2}

    NacelleInPack(5)  = 0.3;  % air induction weight (fraction nacelle plus air induction) (range: 0.1-0.6)
    NacelleInPack(6)  = .005; % pylon support structure weight (fraction W_MTO)     <<<TEST VALUE NEEDS CHECK>>>

    NacelleInPack(7)  = 1; % engine support factor (1 if supports present, else 0)
    NacelleInPack(8)  = 1; % engine cowl factor (1 of cowl present, else 0)
    NacelleInPack(9)  = 1; % pylon support structure factor (1 if pylon present, else 0)
    NacelleInPack(10) = 1; % air induction factor (1 if air induction system present, else 0)

    [W_nacelle_in, W_airind_in] = EvalNacelleWeight(NacelleInPack);

    Geom.Naccelle_2.Mass = (W_nacelle_in + W_airind_in) * Conv_lb_to_kg;
    Geom.Naccelle_3.Mass = (W_nacelle_in + W_airind_in) * Conv_lb_to_kg;
    Geom.Naccelle_5.Mass = (W_nacelle_in + W_airind_in) * Conv_lb_to_kg;
    Geom.Naccelle_6.Mass = (W_nacelle_in + W_airind_in) * Conv_lb_to_kg;
    
    %% Rotor Weight AFDD Model [2] %%
    KW = Vehicle.Propulsion.Rotor_K_W;                                      % Blade Material Coefficient
    D_ft = Vehicle.Geom.Prop_5.Diam*Conv_m_to_ft;                          % Blade Diameter
    B = Vehicle.Propulsion.LPSetup.nBlades;                                 % Number of Blades
    AF = Vehicle.Propulsion.RotorBladeAF;                                   % Blade Activity Factor
    N = max(Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter}.LPRPM);                                    % Blade RPM
    SHP = (Vehicle.Propulsion.LiftMotorPower_Each_kW)*Conv_kW_to_hp;        % Shaft Horsepower [hp]
    M = 0;                                                                  % Design Mach Number
    
    %%%% Pack Inputs %%%%

    LiftPropPack(1) = Vehicle.Propulsion.NLiftProps; % number of auxillary thrusters
    LiftPropPack(2) = 1; % thrust per propellor {lb}
    LiftPropPack(3) = (Vehicle.Propulsion.LiftMotorPower_Each_kW)*Conv_kW_to_hp; % power per propellor {hp}
    LiftPropPack(4) = pi * (Vehicle.Geom.Prop_5.Diam * Conv_m_to_ft / 2)^2 ; % auxiliary thruster disk area {ft^2}
    LiftPropPack(5) = Vehicle.Propulsion.CPSetup.nBlades; % number of blades per propellor
    LiftPropPack(6) = max(Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter}.LPRPM); % propellor rotation sped at P_at {rpm}
    LiftPropPack(7) = Vehicle.Geom.Prop_5.Diam *Conv_m_to_ft; % propellor diameter {ft}
    LiftPropPack(8) = 1; % material factor (composite = 1, wood = 1.2, aluminum spar = 1.31, aluminum construction = 1.44)

    LiftPropPack(9) = 1; % auxillary propulsion/propellor factor (1 if present, else 0)

    Rotor_W_lb = PropWeight(LiftPropPack);
   
    Geom.Prop_5.Mass = Rotor_W_lb*Conv_lb_to_kg;
    Geom.Prop_6.Mass = Rotor_W_lb*Conv_lb_to_kg;
   
    
    %% Prop Weight AFDD Model [4] %%
    
    %%%% Pack Inputs %%%%

    CruisePropPack(1) = Vehicle.Propulsion.NCruiseProps; % number of auxillary thrusters
    CruisePropPack(2) = 1; % thrust per propellor {lb}
    CruisePropPack(3) = (Vehicle.Propulsion.CruisePropPower_kW)*Conv_kW_to_hp; % power per propellor {hp}
    CruisePropPack(4) = pi * (Vehicle.Geom.Prop_1.Diam * Conv_m_to_ft / 2)^2 ; % auxiliary thruster disk area {ft^2}
    CruisePropPack(5) = Vehicle.Propulsion.CPSetup.nBlades; % number of blades per propellor
    CruisePropPack(6) = max(Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter}.MPRPM); % propellor rotation sped at P_at {rpm}
    CruisePropPack(7) = Vehicle.Geom.Prop_1.Diam *Conv_m_to_ft; % propellor diameter {ft}
    CruisePropPack(8) = 1; % material factor (composite = 1, wood = 1.2, aluminum spar = 1.31, aluminum construction = 1.44)

    CruisePropPack(9) = 1; % auxillary propulsion/propellor factor (1 if present, else 0)

    Prop_W_lb = PropWeight(CruisePropPack);

    Geom.Prop_1.Mass = Prop_W_lb*Conv_lb_to_kg;
    Geom.Prop_2.Mass = Prop_W_lb*Conv_lb_to_kg;
    Geom.Prop_3.Mass = Prop_W_lb*Conv_lb_to_kg;
    Geom.Prop_4.Mass = Prop_W_lb*Conv_lb_to_kg;
    
    %% Lift Motor Weight
    LiftMotorMass_Each_kg = Vehicle.Propulsion.LiftMotorPower_Each_kW/ Vehicle.SOTA.ElectricMotorPMR_kWkg;
    Geom.Motors.Mass(5) = LiftMotorMass_Each_kg;
    Geom.Motors.Mass(6) = LiftMotorMass_Each_kg;
    
    %% Cruise Motor Weight
    CruiseMotorMass_Each_kg = Vehicle.Propulsion.CruiseMotorPower_Each_kW/ Vehicle.SOTA.ElectricMotorPMR_kWkg;
    Geom.Motors.Mass(1) = CruiseMotorMass_Each_kg;
    Geom.Motors.Mass(2) = CruiseMotorMass_Each_kg;
    Geom.Motors.Mass(3) = CruiseMotorMass_Each_kg;
    Geom.Motors.Mass(4) = CruiseMotorMass_Each_kg;

    %% ARCHITECTURE 2 and 3: turboshaft power path component weights
    if(Vehicle.Architecture == 2 || Vehicle.Architecture == 3)

        %%%%%%%%% TURBOSHAFT ENGINE WEIGHT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Velo_kts = 0;                                                       % Most Turboshafts are rated at static conditions
        Alt_ft = 0;                                                         % Most Turboshafts are rated sea-level conditions

        % horsepower rating of the turboshaft engine
        P_sl_hp = Vehicle.Propulsion.Turboshaft_Each_kW * Conv_kW_to_hp;

        % why the double referral (RTInterp.RTInterp)?
        RTinterp = Vehicle.Propulsion.RTinterp.RTinterp;
        TLinterp = Vehicle.Propulsion.TLinterp.TLinterp;

        %[~, E_weight_lb,~,~] = TS_IES(P_sl_hp,Velo_kts,Alt_ft,RT,TL);
        [~,TSEngMass_kg,TSEngLength_m,TSEngDiam_m,SFC_kgkwh] = TS_IES(P_sl_hp,Velo_kts,Alt_ft,RTinterp,TLinterp);

        % turboshaft engine dimensions
        Geom.Turboshaft.Length_m = TSEngLength_m;
        Geom.Turboshaft.Diam_m = TSEngDiam_m;
        Geom.Turboshaft.xDim_m = TSEngLength_m * ones(1,length(Geom.Turboshaft.Mass));
        Geom.Turboshaft.yDim_m = (TSEngDiam_m/2) * ones(1,length(Geom.Turboshaft.Mass));
        Geom.Turboshaft.zDim_m = (TSEngDiam_m/2) * ones(1,length(Geom.Turboshaft.Mass));

        % turboshaft engine SFC
        Geom.Turboshaft.SFC_kgkwh = SFC_kgkwh;

        % turboshaft mass
        %Geom.Turboshaft.Mass = TSEngMass_kg*ones(1,length(Geom.Turboshaft.Mass));

        %% Engine Weight AFDD Model %%

        %%%% Pack inputs %%%%

        EngPack(1) = TSEngMass_kg; % weight of one engine {lb}
        EngPack(2) = 1; % number of main engines or jets
        EngPack(3) = P_sl_hp; % installed takeoff power/thrust (SLS static, specified rating) per engine {hp} {lb}
        EngPack(4) = 0; % engine exhaust weight vs. power/thrust constant 0 <<<TEST VALUE>>>
        EngPack(5) = 0; % engine exhaust weight vs. power/thrust constant 1 <<<TEST VALUE>>>

        EngPack(6) = 1; % lubrication system factor (1.4799 if lubrication included in accessories, else 1)

        EngPack(7) = 1; % engine or jet factor (1 if engine present, else 0)
        EngPack(8) = 1; % engine exhaust factor (1 if exhaust present, else 0)
        EngPack(9) = 1; % engine accessories factor (1 if accessories present, else 0)

        Geom.Turboshaft.Mass = EvalEngineWeight(EngPack);

        % for sanity checking: log turboshaft kW/kg
        Vehicle.Propulsion.Turboshaft_kWkg = Vehicle.Propulsion.Turboshaft_Each_kW / TSEngMass_kg;

        %%%%%%%%%%%%% GEARBOX WEIGHT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Wgb_kg = Vehicle.Propulsion.GearboxPower_Each_kW / Vehicle.SOTA.GearboxPMR_kWkg;
        %Geom.Gearbox.Mass = Wgb_kg.*ones(1,length(Geom.Gearbox.Mass));

        %% Drive System Weight AFDD Model %%

        %%%% Pack Inputs %%%%

        DSPack(1) = Vehicle.Propulsion.TurboshaftPower_Each_kW * Conv_kW_to_hp; % drive system power limit (MCP) {hp}
        DSPack(2) = 2; % number of main rotors
        DSPack(3) = 1; % number of gear boxes
        DSPack(4) = 0; % number of intermediate drive shafts
        DSPack(5) = Vehicle.Propulsion.RotorPropRPM; % main-rotor rotation speed {rpm}
        DSPack(6) = 3500; % engine output speed {rpm} <<<TEST VALUE>>>
        DSPack(7) = 0; % length of drive shaft between motors
        DSPack(8) = Vehicle.Propulsion.RotorPropRPM * 60 * 2*pi * (Vehicle.Geom.Prop_1.Diam/2); % main rotor tip speed {ft/s}
        DSPack(9) = Vehicle.Geom.Prop_1.Mass / Vehicle.Geom.Prop_1.NBlades;

        DSPack(10) = 0.6; % second (main or tail) rotor torque limit (fraction of total drive system)
        DSPack(11) = 0.6; % second (main or tail) rotor power limit (fraction of total drive system)
        DSPack(12) = 0.13; % rotor shaft weight (fraction gear box and rotorshaft) (= 0.13, range 0.06-0.20)

        DSPack(13) = 1; % gear box factor (1 if present, else 0)
        DSPack(14) = 0; % rotor shaft factor (1 if present, else 0)
        DSPack(15) = 0; % drive shaft factor (1 if present, else 0)
        DSPack(16) = 0; % rotor brake factor (1 if present, else 0)
        
        Geom.Gearbox.Mass = EvalDriveSystemWeight(DSPack);

        %%%%%%%%%%%%% GENERATOR WEIGHT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Geom.Generator.Mass = (Vehicle.Propulsion.GeneratorPower_Each_kW/Vehicle.SOTA.GeneratorPMR_kWkg).*ones(1,length(Geom.Generator.Mass));
    
        Geom.Generator.Diam_m = Vehicle.Propulsion.GeneratorPower_Each_kW * Vehicle.SOTA.GeneratorSpecDiameter_mkW;
        Geom.Generator.Length_m = Vehicle.Propulsion.GeneratorPower_Each_kW * Vehicle.SOTA.GeneratorSpecLength_mkW;
        Geom.Generator.xDim_m = Geom.Generator.Length_m * ones(1,length(Geom.Generator.Mass));
        Geom.Generator.yDim_m = (Geom.Generator.Diam_m/2) * ones(1,length(Geom.Generator.Mass));
        Geom.Generator.zDim_m = (Geom.Generator.Diam_m/2) * ones(1,length(Geom.Generator.Mass));


        %##################################################################################################################################################################
        %%%%%%%%%%%%% FUEL SYSTEM WEIGHT  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Neng = Vehicle.Propulsion.Neng;                                         % number of engines
        Ntank = Vehicle.Propulsion.Ntank;                                       % number of fuel tanks
        
        Qtot_m3 = sum(Vehicle.Geom.Fuel.Mass)/Vehicle.SOTA.FuelDensity_kgm3;
        Qtot_USG = Qtot_m3 * 264.17;
        Vehicle.Propulsion.Qtot = Qtot_USG;
        Vehicle.Propulsion.Qint = Qtot_USG;
        if Qtot_USG > 0
            Qtot = Qtot_USG;                                         % total volume of fuel, USG
            Qint = Qtot_USG;                                         % total volume of fuel in integral tanks, USG
            
            %% Fuel System Weight AFDD Model %%

            %%%% Pack Inputs %%%%

            FSPack(1) = Vehicle.Propulsion.Ntank; % number of internal fuel tanks
            FSPack(2) = Qint; % internal fuel tank capacity {gal}
            FSPack(3) = Vehicle.Propulsion.Ntank; % total number of fuel tanks (internal and auxillary) for plumbing
            FSPack(4) = Vehicle.Propulsion.Neng; % number of main engines
            FSPack(5) = 0; % plumbing weight, constant 0 (sum of weights for auxillary fuel systems)
            FSPack(6) = 2; % plumbing weight, constant 1 (typical value = 2)
            FSPack(7) = 0; % fuel flow rate {lb/hr}     <<<TEST VALUE>>>

            FSPack(8) = 1; % fuel tank weight (fraction fuel capacity)
            FSPack(9) = 1; % ballistic tolerance factor (1.0-2.5)
            FSPack(10) = 1; % ballisitc survivability level (1.3131 for ballistically survivable, else 1)

            FSPack(11) = 1; % fuel tank factor (1 if present, else 0)
            FSPack(12) = 1; % fuel plumbing factor (1 if present, else 0)

            FS_lb = EvalFuelSysWeight_lb(FSPack);

            Geom.FuelTank_1.Mass = 0.5*FS_lb*Conv_lb_to_kg;
            Geom.FuelTank_2.Mass = 0.5*FS_lb*Conv_lb_to_kg;
        end


    end 
    
    %% Power Cabling Weight  
    [VehTemp] = EvalCablingWeight(Vehicle);
    Geom.Power_Cabling.Mass = VehTemp.Geom.Power_Cabling.Mass;
    Geom.Power_Cabling.CG = VehTemp.Geom.Power_Cabling.CG;

    %% Flight Controls Weight  AFDD Model %%
    %L_fus = Geom.Fuselage.Length*Conv_m_to_ft;                                   % Length of Fuselage [ft]
    %b_ft = Vehicle.Recalcs.WingSpan_m * Conv_m_to_ft;                       % Total Main Wing Span [ft]
    
    %W_FC_lb = EvalFCWeight_lb(L_fus,b_ft,nult,W_dg);

    %%%% Pack Inputs %%%%

    FCPack(1) = 100; % cockpit controls weight {lb}
    FCPack(2) = 50; % automatic flight control system weight {lb}

    FCPack(3) = Vehicle.Geom.VTail.PlanformArea * Conv_m2_to_ft2; % horizontal tail planform area {ft^2}
    FCPack(4) = Vehicle.MassProp.MTOM_kg * Conv_kg_to_lb; % maximum takeoff weight {lb}
    FCPack(5) = Vehicle.Propulsion.NLiftProps; % number of main rotors
    FCPack(6) = 2; % number of blades per rotor
    FCPack(7) = 1; % rotor mean blade chord      <<<TEST VALUE>>>
    FCPack(8) = Vehicle.Propulsion.RotorPropRPM * 60 * 2*pi * (Vehicle.Geom.Prop_1.Diam/2); % rotor hover tip velocity

    FCPack(9) = 0.4; % rotary wing hydraulics weight (fraction hydraulics plus boost mechanism weight) (typical value = 0.4)
    FCPack(10) = 1; % fcs redundacny factor (range 1.0-3.0)
    FCPack(11) = 1; % non-boosted controls survival factor (1.8984 for ballistically survivable, else 1)
    FCPack(12) = 1; % boost mechanism survival factor (1.3029 for ballistically survivable, else 1)
    FCPack(13) = 1; % boosted controls survival factor (1.1171 for ballistically survivable, else 1)
    FCPack(14) = 0.6; % non-boosted weight (fraction total flight controls weight) (typical value = 0.6, range 0.3-1.8, for RW)
    FCPack(15) = 1; % tilt rotor non-boosted weight (fraction boost mechanism weight)
    FCPack(16) = 1; % tilt rotor boost mechanism weight (fraction maximum takeoff weight)

    FCPack(17) = 1; % non-boosted controls factor (1 if present, else 0)
    FCPack(18) = 1; % boost mechanism factor (1 if present, else 0)
    FCPack(19) = 1; % boosted controls factor (1 if present, else 0)
    FCPack(20) = 1; % non-boosted conversion controls factor (1 if present, else 0)
    FCPack(21) = 1; % boost mechanism conversion controls factor (1 if present, else 0)

    FCPack(22) = 2; % (Fixed Wing = 0, Helicopter = 1, Tiltwing = 2, Tiltrotor = 3)

    [W_FC_lb, ~, W_FC_mb] = EvalFlightControlsWeight(FCPack);
    
    Geom.Flight_Controls.Mass = W_FC_lb*Conv_lb_to_kg; 

    %% Hydraulics Weight AFDD Model %%

    % W_hyd_lb = EvalHydWeight_lb(W_dg,Vehicle.DesignPoint.DesignMach);       % 1. Raymer, General Aviation Weights, Pg. 576

    %%%% Pack Inputs %%%%

    HydPack(1) = 50; % hydraulic equipement weight (fixed) {lb}
    HydPack(2) = W_FC_mb; % fight controls boost mechanism weight {lb}
    HydPack(3) = W_FC_lb; % flight controls weight {lb}
    HydPack(4) = 0.4; % hydraulics weight (fraction boost mechanism weight) (typical = 0.4, tiltwing)
    HydPack(5) = 1; % hydraulics weight factor (1 if present, else 0)
    HydPack(6) = 1; % (Fixed Wing / Tilt Rotor = 0, Tiltwing = 1)


    W_hyd_lb = EvalHydWeight_lb(HydPack);
    
    Geom.Hydraulics.Mass = W_hyd_lb*Conv_lb_to_kg;
        
    %% Avionics Weight
    W_avio = 2.117*(Vehicle.Systems.W_uav)^(0.933);                         % W_uav = Weight of Uninstalled Avonics
    
    Geom.Avionics.Mass = W_avio*Conv_lb_to_kg;
    %% Furnishings Weight
    W_fur_lb  = EvalFurWeight_lb(W_dg,NPAX,WTO);
    
    Geom.Furnishings.Mass = W_fur_lb*Conv_lb_to_kg;    
    %% Electrical Weight
    W_el_lb = EvalElecWeight_lb(W_avio,FS_lb,WTO);
    
    Geom.Electrical.Mass = W_el_lb*Conv_lb_to_kg; 
    
    %% empty weight summation
    % get component names
    comps = fieldnames(Geom);
    % eliminate batteries, fuel, passengers, and all-else-empty
    reducedcomps = comps(~strcmpi(comps,'Batteries')&~strcmpi(comps,'Fuel')&~strcmpi(comps,'Passengers')&~strcmpi(comps,'All_Else_Empty'));
     % loop over reduced components
    for i = 1:length(reducedcomps)
        Empty_Weight(i,:) = sum(Geom.(reducedcomps{i}).Mass);                           % Collecting and summing Component Masses [kg]
    end
    EW = sum(Empty_Weight);
    Margin = EWMARG*EW ;                                                     % "All-Else Empty" Mass [kg]
    EmptyMass_kg = EW + Margin;                                                       %
    Geom.All_Else_Empty.Mass = Margin;
    
    
    %% payload
    Payload_per_passenger =(Payload_kg/length(Geom.Passengers.Mass));
    Geom.Passengers.Mass = Payload_per_passenger*ones(1,length(Geom.Passengers.Mass));
    
    
    %% energy mass
    
    % MTOM = EmptyMass + Payload + Fuel + Batteries
    % this has to be satisfied at all times
    
    % determine available energy mass at current iteration TOGM and with
    % given payload and computed EmptyMass
    EnergyMassAvailable_kg = IncomingMTOM_kg - EmptyMass_kg - Payload_kg;

    Vehicle.MassProp.OEM_kg = EmptyMass_kg;
    Vehicle.MassProp.Payload_kg = Payload_kg;
    
    % based on energy usage in the past iteration's mission evaluation,
    % determine how to split the available energy mass between energy types
    % 1 and 2. 
    ratio = [EnergyMass_kg(1), EnergyMass_kg(2)]/sum(EnergyMass_kg);
    EnergyMass_kg = EnergyMassAvailable_kg * ratio;
    
    if EnergyMass_kg(1) < Vehicle.Propulsion.MinEnergyMass_kg(1)
        EnergyMass_kg(1) = min(EnergyMassAvailable_kg, Vehicle.Propulsion.MinEnergyMass_kg(1));
        EnergyMass_kg(2) = EnergyMassAvailable_kg - EnergyMass_kg(1);
    end

    if EnergyMass_kg(2) < Vehicle.Propulsion.MinEnergyMass_kg(2)
        EnergyMass_kg(2) = min(EnergyMassAvailable_kg, Vehicle.Propulsion.MinEnergyMass_kg(2));
        EnergyMass_kg(1) = EnergyMassAvailable_kg - EnergyMass_kg(2);
    end

    % assign these computed values to the mass properties structure
    Vehicle.MassProp.EnergyMass_kg = EnergyMass_kg;
    
    if EnergyMass_kg(1)>0
        % fuel
        Geom.Fuel.Mass = (EnergyMass_kg(1)./length(Vehicle.Geom.Fuel.Mass))*ones(1,length(Vehicle.Geom.Fuel.Mass));                               % [kg]
    else
        Geom.Fuel.Mass = (1e-10./length(Vehicle.Geom.Fuel.Mass))*ones(1,length(Vehicle.Geom.Fuel.Mass));                                          % [kg]
    end

    % if Vehicle.Architecture == 1
    
    if EnergyMass_kg(2)>0
        % battery
        Geom.Batteries.Mass = (EnergyMass_kg(2)./length(Vehicle.Geom.Batteries.Mass))*ones(1,length(Vehicle.Geom.Batteries.Mass));                                          % [kg]
    else
        Geom.Batteries.Mass = (1e-10./length(Vehicle.Geom.Batteries.Mass))*ones(1,length(Vehicle.Geom.Batteries.Mass));                                          % [kg]
    end 
    % else
        % Geom.Batteries.Mass = (Vehicle.Propulsion.MinEnergyMass_kg(2)./length(Vehicle.Geom.Batteries.Mass))*ones(1,length(Vehicle.Geom.Batteries.Mass));
    % end

    
    if(abs(Vehicle.MassProp.OEM_kg - OldOEM) < OEMConvergenceThreshold_kg)
        OEM_conv = 1;
    else
        OldOEM = Vehicle.MassProp.OEM_kg;
    end
    
    

end

    
    
    %% gross weight
%     WTO_new = EmptyMass_kg + Payload_kg + sum(EnergyMass_kg);
%     Vehicle.MassProp.MTOM_kg = WTO_new;
    Vehicle.Geom = Geom;
    
% end
Stopper = 1;


% Vehicle.MassProp.DeltaMass_kg = Vehicle.MassProp.MTOM_kg - IncomingMTOM_kg;

% fprintf(' /// COMPUTED, MTOM: %0.0f, Empty: %0.0f, E1: %0.0f kg, E2: %0.0f kg, iterations: %0.0f\n',Vehicle.MassProp.MTOM_kg,EmptyMass_kg,Vehicle.MassProp.EnergyMass_kg(1), Vehicle.MassProp.EnergyMass_kg(2), iter);

fprintf('(%0.0f iter) ||',iter);

end