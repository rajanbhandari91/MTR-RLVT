function [Cases, Setup] = PropDesignSizing(Settings, PropDef, Cases, Setupin)

% check license
[~] = CheckClearToRun();

if isempty(Settings.FileName) == 0

    % read the setup file
    [Cases, PropDef, DuctDef, CenterbodyDef] = ReadSetupFile(Settings.FileName);

end

% append savename to propeller stuct
% PropDef.Name = Settings.SaveName;

% append duct and centerbody definitions to propeller structure
% PropDef.Duct = DuctDef;
% PropDef.Centerbody = CenterbodyDef;

[n,~] = size(Cases);

i = 1;


%%%%%%%%% DESIGN POINT %%%%%%%%%%%%%%%
FltConDef.Alt = Cases.Alt_ft(1);
FltConDef.Sig = Cases.Sig(1);
FltConDef.Vel = Cases.V(1);
FltConDef.rpm = Cases.RPM(1);
FltConDef.T = Cases.Treq_N(1);
FltConDef.P = Cases.Pabs_kW(1) * 1000;  % convert kW to W
FltConDef.dBeta = 0;


% Run QMIL at the on-design condition
% Run QPROP at the same condition
casename = sprintf('SizingPoint_%s',Cases.CaseName{1});
RunQMIL = 1;
RunQPROP = 1;
Analysis = 'PointPerf';


[QMIL,QPROP] = QMIL_PROP(FltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);



% log prop performance
Cases.Tact_N(1,1) = QPROP.T_N;
Cases.P_kW(1,1) = QPROP.Pshaft_W/1000;

Cases.BP75R(1,1) = interp1(QMIL.r_R,QMIL.beta,0.75,'linear');
Cases.Cl_avg(1,1) = QPROP.cl_avg;

% log case type based on what has been specified
if Cases.Pabs_kW(1)~=0
    Cases.Specified(1) = {'Power'};
else
    Cases.Specified(1) = {'Thrust'};
end


% Plot the propeller blade
% PlotPropBlade;



%%%%%%%%%%% VARIABLE PITCH PROPELLER %%%%%%%%%%%%
if strcmpi(PropDef.Type,'variable-pitch')
    for i = 2:n

        ODFltConDef.Alt = Cases.Alt_ft(i);
        ODFltConDef.Sig = Cases.Sig(i);
        ODFltConDef.Vel = Cases.V(i);
        ODFltConDef.rpm = Cases.RPM(i);
        ODFltConDef.T = Cases.Treq_N(i);


        % ODPropDef.Pshaft_av = 134000;       % Available shaft-power (W)
        ODFltConDef.P = 0;
        ODFltConDef.dBeta1 = PropDef.BP75R_max - Cases.BP75R(1,1);
        ODFltConDef.dBeta2 = PropDef.BP75R_min - Cases.BP75R(1,1);
        ODFltConDef.dBetaDel = 1;
        ODFltConDef.T =  0.1;

        % Test the off-design condition(s)
        % Do not run QMIL, run QPROP
        casename = sprintf('BetaSweep_%s',Cases.CaseName{i});
        RunQMIL = 0;
        RunQPROP = 1;
        Analysis = 'Beta';


        % run QPROP
        [~,QPROP2] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);
        QPROP2 = QPROP2(~isnan(QPROP2.T_N),:);   % rejecting the failed cases
        inds = find(unique(QPROP2.T_N));
        QPROP2 = QPROP2(inds,:);
       
        
        [Tmax,indTmax] = max(QPROP2.T_N);


        % if operating point is specified in terms of thrust
        if(Cases.Pabs_kW(i)==0)

            Cases.Specified(i) = {'Thrust'};   


            if indTmax > 1
                BetaTest(1) = interp1(QPROP2.T_N(1:indTmax),QPROP2.Dbeta(1:indTmax),Cases.Treq_N(i),'linear');
                stopper = 1;
            else
                BetaTest(1) = NaN;
            end
         

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
                BetaSel = BetaTest(indeffsoln);
            end

        end


        % if operating point is specified in terms of power
        if(Cases.Pabs_kW(i)~=0)

            Cases.Specified(i) = {'Power'};

            % eliminate any rows that have negative thrust readings
            QPROP2mod = QPROP2(QPROP2.T_N>=0,:);

            BetaSel = interp1(QPROP2mod.Pshaft_W, QPROP2mod.Dbeta, Cases.Pabs_kW(i)*1000,'linear','extrap');

        end

        Cases.BP75R(i,1) = BetaSel + Cases.BP75R(1,1);

        ODFltConDef.dBeta = BetaSel;
        casename = sprintf('OffDesCase_%s',Cases.CaseName{i});
        RunQMIL = 0;
        RunQPROP = 1;
        Analysis = 'PointPerf';
        [~,QPROP3] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);

        % log propeller performance
        Cases.Tact_N(i,1) = QPROP3.T_N;
        Cases.P_kW(i,1) = QPROP3.Pshaft_W/1000;
        Cases.Cl_avg(i,1) = QPROP3.cl_avg;
    end
end



%%%%%%%%%%% FIXED PITCH PROPELLER %%%%%%%%%%%%
if strcmpi(PropDef.Type,'fixed-pitch')
    for i = 2:n

        ODFltConDef.Alt = Cases.Alt_ft(i);
        ODFltConDef.Sig = Cases.Sig(i);
        ODFltConDef.Vel = Cases.V(i);


        % initialize
        CaseClosed = 0;



        % if RPM is given:
        if(CaseClosed==0 && ~isnan(Cases.RPM(i)))

            Cases.Specified(i) = {'RPM'};

            ODFltConDef.P = 0;
            ODFltConDef.dBeta = 0;
            ODFltConDef.T =  0.1;
            ODFltConDef.rpm = Cases.RPM(i);

            casename = sprintf('RPMGiven_%s',Cases.CaseName{i});
            RunQMIL = 0;
            RunQPROP = 1;
            Analysis = 'PointPerf';

            [~,QPROP_final] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);

            % zero out quantities that were ignored
            Cases.LDRatio(i) = 0;
            Cases.ROC_fpm(i) = 0;
            Cases.Pabs_kW(i) = 0;
            Cases.Treq_N(i) = 0;

            % case is closed
            CaseClosed = 1;

        end


        % if the case is still open, then either power has been specified,
        % or thrust has been specified. In either case, an RPM sweep is
        % required.
        if CaseClosed == 0

            % compute speed of sound
            [~,a,~,~] = atmosisa(Cases.Alt_ft(i)/3.28);

            % compute freestream Mach
            Minf = Cases.V(i)/a;

            % set a testing range in terms of tip Mach
            MinTipMach = max(Minf,0.2);
            MaxTipMach = 1.2;

            % compute min and max tangential tip velocity
            MinTipSpeed = MinTipMach * a; MinTangSpeed = sqrt(MinTipSpeed^2 - Cases.V(i)^2);
            MaxTipSpeed = MaxTipMach * a; MaxTangSpeed = sqrt(MaxTipSpeed^2 - Cases.V(i)^2);

            % compute min and max of rpm range
            MinRPM = max(200,(MinTangSpeed/PropDef.r_t) * 60/(2*pi));
            MaxRPM = (MaxTangSpeed/PropDef.r_t) * 60/(2*pi);


            ODFltConDef.rpm1 = MinRPM;
            ODFltConDef.rpm2 = MaxRPM;
            ODFltConDef.rpmDel = 100;

            % ODPropDef.Pshaft_av = 134000;       % Available shaft-power (W)
            ODFltConDef.P = 0;
            ODFltConDef.dBeta = 0;
            ODFltConDef.T =  0.1;

            % Test the off-design condition(s)
            % Do not run QMIL, run QPROP
            casename = sprintf('RPMSweep_%s',Cases.CaseName{i});
            RunQMIL = 0;
            RunQPROP = 1;
            Analysis = 'RPM';

            % run QPROP in RPM sweep mode
            [~,QPROP2] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);

        end


        % if the case is still open and power has been specified
        if(CaseClosed == 0 && Cases.Pabs_kW(i)~=0)

            Cases.Specified(i) = {'Power'};

            inds = find(unique(QPROP2.Pshaft_W));
            QPROP2 = QPROP2(inds,:);

            % find RPM for required power
            RPMsol = interp1(QPROP2.Pshaft_W, QPROP2.rpm, Cases.Pabs_kW(i) * 1000, 'linear');

            Cases.RPM(i,1) = RPMsol;
            ODFltConDef.rpm = RPMsol;
            casename = sprintf('OffDesCase_%s',Cases.CaseName{i});
            RunQMIL = 0;
            RunQPROP = 1;
            Analysis = 'PointPerf';
            [~,QPROP_final] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);

            % compute tip Mach number
            TipSpeed = sqrt(ODFltConDef.Vel^2 + (RPMsol * (2*pi/60) * PropDef.r_t)^2);

            Cases.Mtip(i,1) = TipSpeed/a;

            % zero out quantities that were ignored
            Cases.LDRatio(i) = 0;
            Cases.ROC_fpm(i) = 0;
            Cases.TWR(i) = 0;
            Cases.Treq_N(i) = 0;

            % close the case
            CaseClosed = 1;

        end

        % if the case is still open and thrust has been specified
        if(CaseClosed == 0 && Cases.Treq_N(i)~=0)

            Cases.Specified(i) = {'Thrust'};

            inds = find(unique(QPROP2.T_N));
            QPROP2 = QPROP2(inds,:);

            % find RPM for required thrust
            RPMsol = interp1(QPROP2.T_N, QPROP2.rpm, Cases.Treq_N(i), 'linear');

            Cases.RPM(i,1) = RPMsol;
            ODFltConDef.rpm = RPMsol;
            casename = sprintf('OffDesCase_%s',Cases.CaseName{i});
            RunQMIL = 0;
            RunQPROP = 1;
            Analysis = 'PointPerf';
            [~,QPROP_final] = QMIL_PROP(ODFltConDef,PropDef,RunQMIL,RunQPROP,Analysis,casename);

            % compute tip Mach number
            TipSpeed = sqrt(ODFltConDef.Vel^2 + (RPMsol * (2*pi/60) * PropDef.r_t)^2);

            Cases.Mtip(i,1) = TipSpeed/a;

            % close the case
            CaseClosed = 1;

        end

        % log final solution
        Cases.Tact_N(i,1) = QPROP_final.T_N;
        Cases.P_kW(i,1) = QPROP_final.Pshaft_W/1000;
        Cases.Cl_avg(i,1) = QPROP_final.cl_avg;


        % blade pitch at 75%R is fixed
        Cases.BP75R(i,1) = Cases.BP75R(1,1);

    end
end



% compute efficiency
Cases.eta = Cases.Tact_N.*Cases.V./(Cases.P_kW*1000);

% compute disc loading
Cases.DL_psf = (Cases.Tact_N * 0.2248089431)/(pi * (PropDef.r_t * 3.28084)^2);

% compute actual thrust-to-weight ratio
Cases.TWR = Cases.Tact_N.*Cases.nProps./(Cases.Mass_kg*9.81);
Cases.TWR(isnan(Cases.TWR)) = 0;

Cases.kW_kg = Cases.P_kW./(Cases.Tact_N/9.81);
Cases.hp_klb = 1000 * (Cases.P_kW*1.34102)./(2.20462 * Cases.Tact_N/9.81);





% screen display the results
%Cases


% save the propeller/rotor setup
nRotor = Setupin.nRotor;
nrad = 10; %length(QMIL.r);   %This can be increase while generating ac3d otherwise 10 is good for simulation model
npsi = 8;
% add QMIL propeller definition to setup struct
Setup.BladeDef = QMIL;
Setup.AirfoilDef = PropDef;
Setup.nRotor = nRotor;
Setup.nrad = nrad;
Setup.npsi = npsi;
Setup.Type = PropDef.Type;
Setup.nBlades = PropDef.Nblades;
Setup.RotorRadius = PropDef.r_t;
Setup.RotorLoc = Setupin.RotorLoc;
Setup.SpinDir = Setupin.SpinDir;
Setup.RotorAxisPhi = Setupin.RotorAxisPhi;
Setup.RotorAxisTheta = Setupin.RotorAxisTheta;
Setup.HealthStatus = Setupin.HealthStatus;
[Setup] = CompleteRotorSetup(Setup,nRotor,nrad,npsi);
% [Setup] = CompleteRotorSetup(PropDef,QMIL,nRotor, nrad, npsi);



% save(Settings.SaveName,'Setup')


%%% GENERATING AC3D GEOMETRY of PROP/ROTOR, DUCT, and CENTERBODY
% if Settings.GenAC3DFile
%
%     SpecSetup = Setup;
%     SpecSetup.Chord = Setup.ChordDist;
%     SpecSetup.Pitch = Setup.PitchDistDeg;
%
%     [Geom.Rotor] = GenerateRotors(Setup.RotorLoc',0,0,1,SpecSetup,Setup.Airfoil,1,[1,0,0],'blank',Settings.MCSFlag);
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
%     [VertColl,VertColl_FSBLWL]=makeAC3D(Geom, Settings.AC3DSaveName, 1,1,[]);
% end




