


Cases = PropData.Cases;
PropDef = PropData.PropDef;
PropFoil = PropData.PropFoil;
DuctDef = PropData.DuctDef;
CenterbodyDef = PropData.CenterbodyDef;

% Conversion factors
g = 9.81;
conv_ft_to_m = 1/3.28;
conv_kt_to_ms = 0.51444;
conv_fpm_to_ms = 0.00508;



% extracted
PropDef.r_h = PropDef.HubRadiusRatio * PropDef.r_t;                           % hub radius(m)

[~,a,~,rho] = atmosisa(Cases.Alt_ft * conv_ft_to_m);
Cases.Sig = rho/1.225;

[n,~]= size(Cases);

% find cases where RPM is given
indRPMgiven = find(~isnan(Cases.RPM));
% for these cases, determine Tip Mach
if~isempty(indRPMgiven)
    Cases.Mtip(indRPMgiven) = sqrt(  Cases.V(indRPMgiven).^2  +   (PropDef.r_t * Cases.RPM(indRPMgiven)*(2*pi/60)).^2   )./ a(indRPMgiven);
end

% find cases where tip Mach is given
indMtipgiven = find(~isnan(Cases.Mtip));
% for these cases, determine the RPM
if~isempty(indMtipgiven)
    Cases.RPM(indMtipgiven) = (sqrt(  (Cases.Mtip(indMtipgiven).*a(indMtipgiven)).^2 - Cases.V(indMtipgiven).^2     )   /PropDef.r_t)*60/(2*pi);
end


% for cases where TWR is given, compute thrust based on it
indTWRgiven = find(~isnan(Cases.TWR));
if~isempty(indTWRgiven)
    Cases.Treq_N(indTWRgiven,1) = Cases.TWR(indTWRgiven) .* Cases.Mass_kg(indTWRgiven) * g./Cases.nProps(indTWRgiven);
end

% for forward flight cases (V>0 and L/D given), compute thrust based on L/D
indfwdflt = find(Cases.KTAS>0 & ~isnan(Cases.LDRatio));
if~isempty(indfwdflt)
    Cases.Treq_N(indfwdflt,1) =  Cases.Mass_kg(indfwdflt) * g.* (1./Cases.LDRatio(indfwdflt) + Cases.ROC_fpm(indfwdflt) * conv_fpm_to_ms./(Cases.KTAS(indfwdflt) * conv_kt_to_ms))./Cases.nProps(indfwdflt) ;
    Cases.TWR(indfwdflt,1) = Cases.Treq_N(indfwdflt,1).*Cases.nProps(indfwdflt)./(g.*Cases.Mass_kg(indfwdflt));
end






[n,~] = size(Cases);


i = 1;

PropDef.Alt = Cases.Alt_ft(1);
PropDef.Sig = Cases.Sig(1);
PropDef.Vel = Cases.V(1);
PropDef.rpm = Cases.RPM(1);
PropDef.T = Cases.Treq_N(1);
PropDef.P = 0;
PropDef.dBeta = 0;



% Run QMIL at the on-design condition
% Run QPROP at the same condition
casename = sprintf('SizingPoint_%s',Cases.CaseName{1});
RunQMIL = 1;
RunQPROP = 1;
Analysis = 'PointPerf';


[QMIL,QPROP] = QMIL_PROP(PropFoil,PropDef,RunQMIL,RunQPROP,Analysis,casename);
QMILResults(i).QMIL = QMIL;
QPROPResults(i).QPROP = QPROP;


Cases.Tact_N(1,1) = QPROP.T_N;
Cases.P_kW(1,1) = QPROP.Pshaft_W/1000;
Cases.eta(1,1) = QPROP.effprop;
Cases.BP75R(1,1) = interp1(QMIL.r_R,QMIL.beta,0.75,'linear');
Cases.kW_kg(1,1) = Cases.P_kW(1,1)/(Cases.Treq_N(1,1)/9.81);
Cases.hp_klb(1,1) = Cases.kW_kg(1,1) /(0.746*2.20462/1000);
Cases.Cl_avg(1,1) = QPROP.cl_avg;



% PlotPropBlade;



for i = 2:n

    ODPropDef.Alt = Cases.Alt_ft(i);
    ODPropDef.Sig = Cases.Sig(i);
    ODPropDef.Vel = Cases.V(i);
    ODPropDef.rpm = Cases.RPM(i);
    ODPropDef.T = Cases.Treq_N(i);


    % ODPropDef.Pshaft_av = 134000;       % Available shaft-power (W)
    ODPropDef.P = 0;
    ODPropDef.dBeta1 = PropDef.BP75R_max - Cases.BP75R(1,1);
    ODPropDef.dBeta2 = PropDef.BP75R_min - Cases.BP75R(1,1);
    ODPropDef.dBetaDel = 1;
    ODPropDef.T =  0.1;

    % Test the off-design condition(s)
    % Do not run QMIL, run QPROP
    casename = sprintf('BetaSweep_%s',Cases.CaseName{i});
    RunQMIL = 0;
    RunQPROP = 1;
    Analysis = 'Beta';


    %clear QPROP2
    [~,QPROP2] = QMIL_PROP(PropFoil,ODPropDef,RunQMIL,RunQPROP,Analysis,casename);
    inds = find(unique(QPROP2.T_N));
    QPROP2 = QPROP2(inds,:);

    [Tmax,indTmax] = max(QPROP2.T_N);

    if indTmax > 1
        BetaTest(1) = interp1(QPROP2.T_N(1:indTmax),QPROP2.Dbeta(1:indTmax),Cases.Treq_N(i),'linear');
    else
        BetaTest(1) = NaN;
    end

%     BetaTest(1) = interp1(QPROP2.T_N(1:indTmax),QPROP2.Dbeta(1:indTmax),Cases.Treq_N(i),'linear');


    if ~isnan(BetaTest(1))
        PowerTest(1) = interp1(QPROP2.Dbeta(1:indTmax),QPROP2.Pshaft_W(1:indTmax),BetaTest(1),'linear');
    else
        PowerTest(1) = 9999999999;
    end

    BetaTest(2) = interp1(QPROP2.T_N(indTmax+1:end),QPROP2.Dbeta(indTmax+1:end),Cases.Treq_N(i),'linear');
    if ~isnan(BetaTest(2))
        PowerTest(2) = interp1(QPROP2.Dbeta(indTmax+1:end),QPROP2.Pshaft_W(indTmax+1:end),BetaTest(2),'linear');
    else
        PowerTest(2) = 9999999999;
    end

    if(length(BetaTest)>1)
        [~,indeffsoln] = min(PowerTest);
        BetaTest = BetaTest(indeffsoln);
    end

    Cases.BP75R(i,1) = BetaTest + Cases.BP75R(1,1);

    ODPropDef.dBeta = BetaTest;
    casename = sprintf('OffDesCase_%s',Cases.CaseName{i});
    RunQMIL = 0;
    RunQPROP = 1;
    Analysis = 'PointPerf';
    [~,QPROP3] = QMIL_PROP(PropFoil,ODPropDef,RunQMIL,RunQPROP,Analysis,casename);

    Cases.Tact_N(i,1) = QPROP3.T_N;
    Cases.P_kW(i,1) = QPROP3.Pshaft_W/1000;
    Cases.eta(i,1) = QPROP3.effprop;
    Cases.kW_kg(i,1) = Cases.P_kW(i,1)/(Cases.Treq_N(i,1)/9.81);
    Cases.hp_klb(i,1) = Cases.kW_kg(i,1) /(0.746*2.20462/1000);
    Cases.Cl_avg(i,1) = QPROP3.cl_avg;
end

% compute disc loading
Cases.DL_psf = (Cases.Tact_N * 0.2248089431)/(pi * (PropDef.r_t * 3.28084)^2);
PropData.Cases  = Cases;

% screen display the results
Cases



% % save the propeller/rotor setup
% nRotor = 1;
% nrad = length(QMIL.r);
% npsi = 8;
% [Setup] = CompleteRotorSetup(PropDef,QMIL,nRotor, nrad, npsi);
% save(SetupSaveName,'Setup')
% 
% 
% %%% GENERATING AC3D GEOMETRY of PROP/ROTOR, DUCT, and CENTERBODY
% if GenAC3DFile
% 
%     SpecSetup = Setup;
%     SpecSetup.Chord = Setup.ChordDist;
%     SpecSetup.Pitch = Setup.PitchDistDeg;
% 
%     [Geom.Rotor] = GenerateRotors(Setup.RotorLoc',0,0,1,SpecSetup,Setup.Airfoil,1,[1,0,0],'blank',MCSFlag);
%     % create duct component, if present
%     if ~isempty(DuctDef)
%         [Geom.Duct] = GenerateDuct(DuctDef);
%     end
% 
%     % create centerbody component, if defined
%     if ~isempty(CenterbodyDef)
%         [Geom.Centerbody] = GenerateCenterbody(CenterbodyDef);
%     end
% 
%     % Create AC3D file
%     [VertColl,VertColl_FSBLWL]=makeAC3D(Geom, AC3DSaveName, 1,1,[]);
% end







