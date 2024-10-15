


% Bring in the EvalRotorFM.m function
% Test whether the generated rotor setup runs for this function
% This needs to be tested before the rotor setup can be used in MADCASP or
% within the sizing framework


% This file is meant to be run while the workspace from execution of
% "PropDesignMaster" is loaded


% To multiply the rotor to get "n" rotors
nRotor = 8;

% set the number of radial points
nrad = 10;

% set the number of azimuthal points
npsi = 8;


% nrad = [0.1439
% 0.1731
% 0.2023
% 0.2316
% 0.2608
% 0.29
% 0.3192
% 0.3485
% 0.3777
% 0.4069
% 0.4362
% 0.4654
% 0.4946
% 0.5239
% 0.5531
% 0.5823
% 0.6115
% 0.6408
% 0.67
% 0.6992
% 0.7285
% 0.7577
% 0.7869
% 0.8162
% 0.8454]'/QMIL.r(end);


% nrad = 15;

[Setup] = CompleteRotorSetup(PropDef,QMIL, nRotor, nrad, npsi);




Setup.ThrustScaler = 0.96;
Setup.TorqueScaler = 1.08;


CompTbl = table();
CompTbl.Name = Cases.CaseName;
CompTbl.V = Cases.V;
CompTbl.Alt_ft = Cases.Alt_ft;
CompTbl.RPM = Cases.RPM;
CompTbl.BP75R = Cases.BP75R;
CompTbl.TQPROP = Cases.Tact_N;
CompTbl.TBEMT = zeros(height(Cases),1);
CompTbl.TRatio = zeros(height(Cases),1);
CompTbl.TDiff = zeros(height(Cases),1);
CompTbl.QQPROP = (Cases.P_kW*1000)./(Cases.RPM*2*pi/60);
CompTbl.QBEMT = zeros(height(Cases),1);
CompTbl.QRatio =zeros(height(Cases),1);
CompTbl.QDiff =zeros(height(Cases),1);

for i = 1:height(Cases)

    [~,FlowCon.SpeedOfSound,~,FlowCon.rho] = atmosisa(CompTbl.Alt_ft(i)/3.28);
    FlowCon.DynVisc = 0.17000E-04;

    RPM = CompTbl.RPM(i) * ones(1,nRotor);
    BladePitch = CompTbl.BP75R(i)*ones(1,nRotor);
    XInflow = [];

    Vaxial = CompTbl.V(i);
    Vhub_RA = repmat([0;0;-Vaxial] , [1,1,nRotor]);

    Om_RA = zeros(3,1,nRotor);
    HealthStatus = ones(1,nRotor);
    EvalMode = 'report';

    % Call EvalRotorFM
    [F_RA,M_RA,Xrotor_dot,Summary,ElemWise,XInflow_out] = EvalRotorFM(Setup,FlowCon,RPM,BladePitch,XInflow,Vhub_RA,Om_RA,HealthStatus,EvalMode);

    CompTbl.TBEMT(i) = -F_RA(3,1);
    CompTbl.QBEMT(i) = -M_RA(3,1);

end


CompTbl.TRatio = CompTbl.TBEMT./CompTbl.TQPROP;
CompTbl.TDiff = CompTbl.TBEMT-CompTbl.TQPROP;

CompTbl.QRatio = CompTbl.QBEMT./CompTbl.QQPROP;
CompTbl.QDiff = CompTbl.QBEMT-CompTbl.QQPROP;

CompTbl.etaBEMT = CompTbl.TBEMT.*CompTbl.V./(CompTbl.QBEMT.*CompTbl.RPM*2*pi/60);


CompTbl
