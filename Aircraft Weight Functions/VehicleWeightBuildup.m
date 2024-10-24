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


    %% Wing Weight
    S_ft2 = Vehicle.Recalcs.WingArea_TOT_m2 * Conv_m_to_ft^2;               % wing planform area, ft2
    b_ft = Vehicle.Recalcs.WingSpan_m * Conv_m_to_ft;                       % wing span, ft
    AR = 2*Vehicle.Geom.RWing.AspectRatio;                                                      % aspect ratio
    TR = Geom.RWing.TaperDefn(end);                                         % wing taper ratio
    lam25 = mean(Geom.RWing.Stn.Sw_25);                                     % quarter-chord sweep
    lam50 = mean(Geom.RWing.Stn.Sw_50);                                     % Mid-Chord Sweep
    TOCmax = Geom.RWing.Thickness_to_chord;
    
    RootThickness_ft = TOCmax * Vehicle.Geom.RWing.Stn.c(1) * Conv_m_to_ft; % max thickness of wing root in ft
    
    [Main_Wings_lb] = EvalWingWEIGHT(S_ft2,b_ft,TR,AR,TOCmax,...
        lam25,lam50,RootThickness_ft,W_dg,q,0,VmaxSLkt,nult,WTO);

    % dividing this mass by 2, and assign to left and right wings
    Geom.LWing.Mass = Main_Wings_lb * Conv_lb_to_kg / 2;
    Geom.RWing.Mass = Main_Wings_lb * Conv_lb_to_kg / 2;

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
    
    %% Landing Gear Weight
    Lm = Vehicle.Geom.LandingGear_Main_1.BellyGroundClearance * Conv_m_to_inch;                       % main gear shock strut length, conv to inch
    Ln = Vehicle.Geom.LandingGear_Nose.BellyGroundClearance * Conv_m_to_inch;                       % nose gear shock strut length, conv to inch
    nMLDG = 2;
    nNLDG = 1;
    [W_LG_lb, W_MLDG_each_lb, W_NLDG_each_lb] = EvalLDGWeight_lb(nult,Wldg,Lm,Ln,nMLDG,nNLDG);

    Geom.LandingGear_Main_1.Mass = W_MLDG_each_lb * Conv_lb_to_kg;
    Geom.LandingGear_Main_2.Mass = W_MLDG_each_lb * Conv_lb_to_kg;
    Geom.LandingGear_Nose.Mass   = W_NLDG_each_lb * Conv_lb_to_kg;

    %% Nacelle Weight
    % Nacelle 1 and 4 Weight
    N_Lt = Geom.Nacelle_1.Length * Conv_m_to_ft;
    N_w = (Geom.Nacelle_1.MaxWidth) * Conv_m_to_ft;
    S_n = Geom.Nacelle_1.Stn.sdA_Wet(end) * Conv_m2_to_ft2;
    Nac_W_lb = EvalFusWeight_lb(WTO,nult,N_Lt,N_w,N_w,VC,S_n,W_dg,lh,q);
    Geom.Nacelle_1.Mass = 1.2*Nac_W_lb * Conv_lb_to_kg;                     % 1.2 factor for tilting mechanism
    Geom.Nacelle_4.Mass = 1.2*Nac_W_lb * Conv_lb_to_kg;
    
    % Nacelle 2, 3, 5, and 6 Weight
    N_Lt = Geom.Nacelle_2.Length * Conv_m_to_ft;
    N_w = (Geom.Nacelle_2.MaxWidth) * Conv_m_to_ft;
    S_n = Geom.Nacelle_2.Stn.sdA_Wet(end) * Conv_m2_to_ft2;
    Nac_W_lb = EvalFusWeight_lb(WTO,nult,N_Lt,N_w,N_w,VC,S_n,W_dg,lh,q);
    Geom.Nacelle_2.Mass = 1.2*Nac_W_lb * Conv_lb_to_kg;                     % 1.2 factor for tilting mechanism
    Geom.Nacelle_3.Mass = 1.2*Nac_W_lb * Conv_lb_to_kg;
    Geom.Nacelle_5.Mass = Nac_W_lb * Conv_lb_to_kg;                     
    Geom.Nacelle_6.Mass = Nac_W_lb * Conv_lb_to_kg;
    
    %% Rotor Weight [2]
    KW = Vehicle.Propulsion.Rotor_K_W;                                      % Blade Material Coefficient
    D_ft = Vehicle.Geom.Prop_5.Diam*Conv_m_to_ft;                          % Blade Diameter
    B = Vehicle.Propulsion.LPSetup.nBlades;                                 % Number of Blades
    AF = Vehicle.Propulsion.RotorBladeAF;                                   % Blade Activity Factor
    N = max(Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter}.LPRPM);                                    % Blade RPM
    SHP = (Vehicle.Propulsion.LiftMotorPower_Each_kW)*Conv_kW_to_hp;        % Shaft Horsepower [hp]
    M = 0;                                                                  % Design Mach Number
    
    Rotor_W_lb = PropWeight(KW,D_ft,B,AF,N,SHP,M);
   
    Geom.Prop_5.Mass = Rotor_W_lb*Conv_lb_to_kg;
    Geom.Prop_6.Mass = Rotor_W_lb*Conv_lb_to_kg;
   
    
    %% Prop Weight [4]
    KW = Vehicle.Propulsion.Cruise_K_W;                                     % Blade Material Coefficient
    D_ft = Vehicle.Geom.Prop_1.Diam *Conv_m_to_ft;                          % Blade Diameter
    B = Vehicle.Propulsion.CPSetup.nBlades;                                % Number of Blades
    AF = Vehicle.Propulsion.CruiseBladeAF;                                   % Blade Activity Factor
    N = max(Vehicle.PointPerf.Results{Vehicle.Tracker.MassIter}.MPRPM);                                    % Blade RPM
    SHP = (Vehicle.Propulsion.CruisePropPower_kW)*Conv_kW_to_hp;            % Shaft Horsepower [hp]
    M = 0;                                                                  % Design Mach Number
    
    Prop_W_lb = PropWeight(KW,D_ft,B,AF,N,SHP,M);

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
        Geom.Turboshaft.Mass = TSEngMass_kg*ones(1,length(Geom.Turboshaft.Mass));

        % for sanity checking: log turboshaft kW/kg
        Vehicle.Propulsion.Turboshaft_kWkg = Vehicle.Propulsion.Turboshaft_Each_kW / TSEngMass_kg;

        %%%%%%%%%%%%% GEARBOX WEIGHT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Wgb_kg = Vehicle.Propulsion.GearboxPower_Each_kW / Vehicle.SOTA.GearboxPMR_kWkg;
        Geom.Gearbox.Mass = Wgb_kg.*ones(1,length(Geom.Gearbox.Mass));

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

            FS_lb = EvalFuelSysWeight_lb(Qtot,Qint,Ntank,Neng);

            Geom.FuelTank_1.Mass = 0.5*FS_lb*Conv_lb_to_kg;
            Geom.FuelTank_2.Mass = 0.5*FS_lb*Conv_lb_to_kg;
        end


    end 
    
    %% Power Cabling Weight  
    [VehTemp] = EvalCablingWeight(Vehicle);
    Geom.Power_Cabling.Mass = VehTemp.Geom.Power_Cabling.Mass;
    Geom.Power_Cabling.CG = VehTemp.Geom.Power_Cabling.CG;

    %% Flight Controls Weight
    L_fus = Geom.Fuselage.Length*Conv_m_to_ft;                                   % Length of Fuselage [ft]
    b_ft = Vehicle.Recalcs.WingSpan_m * Conv_m_to_ft;                       % Total Main Wing Span [ft]
    
    W_FC_lb = EvalFCWeight_lb(L_fus,b_ft,nult,W_dg);
    
    Geom.Flight_Controls.Mass = W_FC_lb*Conv_lb_to_kg;    
    %% Hydraulics Weight
    W_hyd_lb = EvalHydWeight_lb(W_dg,Vehicle.DesignPoint.DesignMach);       % 1. Raymer, General Aviation Weights, Pg. 576
    
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