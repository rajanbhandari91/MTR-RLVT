function [Fbf,Mbf,Hr,Hrdot,AddlRes,Xrotor_dot,AddlOutputs] = FMComp(States,Controls,XInflow,Veh,m,r_cg,EvalMode)

if isempty(Veh)
    global Vehicle;
else
    Vehicle = Veh;
end

g = 9.81;

AddlRes = [];

% Activate (1)/Deactivate(0) for testing purposes
ActivateFusLoad = 1;
ActivateLGLoad = 1;
ActivateMotorAeroLoad = 1;
ActivateLSLoad = 1;

% set up the control vector
%% CAPTURE Control vector
% RAVEN-SWFT
%%% CONTROL SURFACES
% (1)	df1     Left outboard flaperon      (deg, +TED)
% (2)	df2     Left midboard flaperon      (deg, +TED)
% (3)	df3     Left inboard flaperon       (deg, +TED)
% (4)	df4     Right inboard flaperon      (deg, +TED)
% (5)	df5     Right midboard flaperon     (deg, +TED)
% (6)	df6     Right outboard flaperon     (deg, +TED)
% (7)	ds      Elevator                    (deg, +TED)
% (8)	dr      Rudder                      (deg, +TER)
%%% NACELLE ANGLES
% (9)	dt1     Left wingtip nacelle angle
% (10)	dt2     Right wingtip nacelle angle
% (11)	dt3     Left forward boom prop tilt angle
% (12)	dt4     Right forward boom prop tilt angle
%%% PROPELLER RPM
% (13)	n1      Prop 1 RPM
% (14)	n2      Prop 2 RPM
% (15)	n3      Prop 3 RPM
% (16)	n4      Prop 4 RPM
% (17)	n5      Prop 5 RPM
% (18)	n6      Prop 6 RPM
%%% PROPELLER BLADE PITCH
% (19)	dc1     Prop 1 collective
% (20)	dc2     Prop 2 collective
% (21)	dc3     Prop 3 collective
% (22)	dc4     Prop 4 collective
% (23)	dc5     Prop 5 collective
% (24)	dc6     Prop 6 collective





% capture flaperon deflections
dF1 = Controls(1);
dF2 = Controls(2);
dF3 = Controls(3);
dF4 = Controls(4);
dF5 = Controls(5);
dF6 = Controls(6);

% Capture elevator deflections
dE1 = Controls(7);
dE2 = Controls(8);

% capture rudder deflections
dR = Controls(9);

% capture prop deflection angle
dt1 = Controls(10);
dt2 = Controls(11);
dt3 = Controls(12);
dt4 = Controls(13);


% Blade RPMs
CPRPMs = Controls(14:17) ;
LPRPMs = Controls(18:19) ;

% Blade Pitch
CPBladePitch = Controls(20:23);
LPBladePitch = Controls(24:25);


LiftPropCutoffRPM = 100;
PropCutoffRPM = 50;



%%% INFLOW STATES
if~isempty(XInflow)
    XInflow_CP = XInflow(:,1:4);
    XInflow_LP = XInflow(:,5:6);
end
if isempty(XInflow)
    XInflow_CP = [];
    XInflow_LP = [];
end

%% CAPTURE STATES
% AOA = States.AirData.AOA;
AOADeg = States.AirData.AOADeg;
% AOAdot = States.AirData.AOAdot;
BETA = States.AirData.BETA;
BETADeg = BETA*180/pi;
TAS = max(0.1,States.AirData.TAS);
% EAS = States.AirData.EAS;
uw = States.AirData.uw;
vw = States.AirData.vw;
ww = States.AirData.ww;
pw = States.AirData.pw;
qw = States.AirData.qw;
rw = States.AirData.rw;
% phat = pw * Vehicle.Aero.RefLengthLat / (2*TAS);
% qhat = qw * Vehicle.Aero.RefLengthLong / (2*TAS);
% rhat = rw * Vehicle.Aero.RefLengthLat / (2*TAS);
% AOAdothat = AOAdot * cbar / (2*TAS);
a = States.AirData.a;
qbar = States.AirData.qbar;
rho = States.AirData.rho;
ph = States.Attitude.ph;
th = States.Attitude.th;
% Patm_Pa = States.AirData.P;
% Tatm_K = States.AirData.OAT;
% ps = States.Attitude.ps;
%Mach = States.AirData.MACH;
%AltMSL = States.Posn.AltMSL;
%AltAGL = States.Posn.AltAGL;


%% CAPTURE STRIP INDICES and PROPERTIES
% Left wing, root to tip
LWIndices = Vehicle.Aero.Indices.LWing;
% Right wing, root to tip
RWIndices = Vehicle.Aero.Indices.RWing;
% Left horizontal stab, root to tip
LHTIndices = Vehicle.Aero.Indices.LHTail;
% Right horizontal stab, root to tip
RHTIndices = Vehicle.Aero.Indices.RHTail;
% Left Vertical stab, root to tip
VSIndices = Vehicle.Aero.Indices.VTail;


% CONTROL SURFACE LOCATIONS
OBDLWFlaperon = Vehicle.Aero.Indices.Outboard_Flaperon_on_LWing;
MBDLWFlaperon = Vehicle.Aero.Indices.Midboard_Flaperon_on_LWing;
IBDLWFlaperon = Vehicle.Aero.Indices.Inboard_Flaperon_on_LWing;
IBDRWFlaperon = Vehicle.Aero.Indices.Inboard_Flaperon_on_RWing;
MBDRWFlaperon = Vehicle.Aero.Indices.Midboard_Flaperon_on_RWing;
OBDRWFlaperon = Vehicle.Aero.Indices.Outboard_Flaperon_on_RWing;
RudIndices = Vehicle.Aero.Indices.Rudder_on_VTail;



% Extracting Quarter Chord Points
xAC = Vehicle.Aero.StripDefn.xAC;       % m
yAC = Vehicle.Aero.StripDefn.yAC;       % m
zAC = Vehicle.Aero.StripDefn.zAC;       % m

% this is the order of elements
% [r11, r12, r13, r21, r22, r23, r31, r32, r33];
R_SB = Vehicle.Aero.StripDefn.R_SB;
R_BS = Vehicle.Aero.StripDefn.R_BS;


% account for outboard flaperon deflections
[R_BS(LWIndices(OBDLWFlaperon),:), R_SB(LWIndices(OBDLWFlaperon),:)] = ApplySingleAxisRotation(R_BS(LWIndices(OBDLWFlaperon),:), dF1, 2);
[R_BS(RWIndices(OBDRWFlaperon),:), R_SB(RWIndices(OBDRWFlaperon),:)] = ApplySingleAxisRotation(R_BS(RWIndices(OBDRWFlaperon),:), dF6, 2);



% Deal into individual direction cosine variables
[R_SB_11, R_SB_12, R_SB_13, R_SB_21, R_SB_22, R_SB_23, R_SB_31, R_SB_32, R_SB_33]...
    = deal(R_SB(:,1),R_SB(:,2),R_SB(:,3),R_SB(:,4),R_SB(:,5),R_SB(:,6),R_SB(:,7),R_SB(:,8),R_SB(:,9));


%% GRAVITY LOADS
Fxgrav = -m*g*sin(th);
Fygrav =  m*g*sin(ph)*cos(th);
Fzgrav =  m*g*cos(ph)*cos(th);
F_grav = [Fxgrav;Fygrav;Fzgrav];

M_grav = [...
    0,          -r_cg(3),       r_cg(2)
    r_cg(3),        0,          -r_cg(1)
    -r_cg(2),   r_cg(1),        0] * F_grav;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LIFT PROPULSOR LOADS

F_liftprops = zeros(3,1);
M_liftprops = zeros(3,1);
Hr_liftprops = [0;0;0];
FxRPlp_bf = zeros(1,2); 
MxRPlp_bf = zeros(1,2); 
FyRPlp_bf = zeros(1,2); 
MyRPlp_bf = zeros(1,2);
FzRPlp_bf = zeros(1,2); 
MzRPlp_bf = zeros(1,2);
Xrotor_dot_LP = zeros(7,2);
LiftPropTorques = zeros(1,2);
LiftPropPwrkW = zeros(1,2);
CPPwrkW = zeros(1,4) ;
FxRP_bf = zeros(1,4);
FyRP_bf = zeros(1,4);
FzRP_bf = zeros(1,4);
MxRP_bf = zeros(1,4);
MyRP_bf = zeros(1,4);
MzRP_bf = zeros(1,4);
F_prop   = zeros(3,1);
M_prop   = zeros(3,1); 
Hr_props = [0;0;0];
Xrotor_dot_CP = zeros(7,4);


if ~all(LPRPMs < LiftPropCutoffRPM)

    FlowCon.rho = rho;
    FlowCon.SpeedOfSound = a;
    FlowCon.DynVisc = 0.17000E-04;

    LPSetup = Vehicle.Propulsion.LPSetup;
    th_RA = LPSetup.RotorAxisTheta;
    ph_RA = LPSetup.RotorAxisPhi;

    % propulsor hub locations
    xloc = Vehicle.Propulsion.LPSetup.RotorLoc(1,:);
    yloc = Vehicle.Propulsion.LPSetup.RotorLoc(2,:);
    zloc = Vehicle.Propulsion.LPSetup.RotorLoc(3,:);

    % Find the indices of individual rotors that fall below threshold RPM
    % (if any). Set the RPM of these to the threshold RPM for running the
    % BEMT code. Then, set the computed forces and moments for these rotors
    % to zero after the fact
    InactiveRotors = LPRPMs<=LiftPropCutoffRPM;

    % account for failed rotors
    HealthStatus = Vehicle.Propulsion.LPSetup.HealthStatus; %ones(1,LPSetup.nRotor);
    % if health status = 0 (failed), then inactive status = 1 (true)
    InactiveRotors = HealthStatus==0;

    % Set inactive rotors to the cutoff RPM
    LPRPMs(InactiveRotors==1) = LiftPropCutoffRPM;

    % compute velocities seen at the rotor hubs, resolved in body-fixed axes
    uhub_bf = uw  +  0.*xloc   - rw.*yloc   +  qw.*zloc;
    vhub_bf = vw  + rw.*xloc   +  0.*yloc   -  pw.*zloc;
    whub_bf = ww  - qw.*xloc   + pw.*yloc   +   0.*zloc;

    % convert these velocities from body-fixed axes to rotor aligned axes
    u_RA =              cosd(th_RA).*uhub_bf                                          - sind(th_RA).*whub_bf;
    v_RA = sind(th_RA).*sind(ph_RA).*uhub_bf  + cosd(ph_RA).*vhub_bf     + cosd(th_RA).*sind(ph_RA).*whub_bf;
    w_RA = sind(th_RA).*cosd(ph_RA).*uhub_bf  - sind(ph_RA).*vhub_bf     + cosd(th_RA).*cosd(ph_RA).*whub_bf;
    Vhub_RA = reshape([u_RA;v_RA;w_RA],[3,1,LPSetup.nRotor]);

    % convert vehicle angular velocity from body-fixed axes to rotor aligned axes
    p_RA =              cosd(th_RA).*pw                                  - sind(th_RA).*rw;
    q_RA = sind(th_RA).*sind(ph_RA).*pw   + cosd(ph_RA).*qw + cosd(th_RA).*sind(ph_RA).*rw;
    r_RA = sind(th_RA).*cosd(ph_RA).*pw   - sind(ph_RA).*qw + cosd(th_RA).*cosd(ph_RA).*rw;
    Om_RA = [p_RA;q_RA;r_RA];
    Om_RA = repmat(Om_RA,[1,1,LPSetup.nRotor]);

    LPSetup.InflowGuess = interp1([0 Vehicle.Operations.VC],[15 1],TAS/0.51444,'linear','extrap') .* ones(1,LPSetup.nRotor);
    % Get forces and moments at the rotor hubs, resolved in rotor axes (RA)
    % I/O: [F_RA,M_RA,L_ss,Summary,ElemWise] = EvalRotorFM(CPSetup,FlowCon,RPM,VaneAngles,VInflow,Vhub_RA,Om_RA,HealthStatus,EvalMode)
    [Fhub_RA,Mhub_RA,Xrotor_dot_LP,Summary,ElemWise,XInflowOut_LP] = EvalRotorFM(LPSetup,FlowCon,LPRPMs',LPBladePitch,XInflow_LP,Vhub_RA,Om_RA,HealthStatus,EvalMode);

    % Inflow velocity at rotors
    wLP = max(0,XInflowOut_LP(1,:));
    wLP(InactiveRotors==1)  = 0 ;
    % Vtest = -Vhub_RA(3,:)
    % Ttest = -Fhub_RA(3,:);
    %
    % wtest = (1/2)*(-Vtest + sqrt(Vtest.^2 + 2*Ttest./(rho.*Vehicle.Propulsion.LPSetup.DiskArea)))

    if EvalMode == 3
        save('SavedLPStates.mat','Summary','ElemWise','States');
    end

    % Download due to the booms below the lift props, as a "Thrust Loss"
    % Assumptions:
    % 1. Booms have the same width (c) along majority of the longitudinal
    % axis calculation based on the max-width of the boom
    % 2. All the lift props are of same radius (R)
    % 3. The download is assumed to be affecting the rotor axes forces in
    %
    % 'z' direction only

    TipMount = 1;                                   % (1) if the props are mounted at tip
    R = Vehicle.Propulsion.LPSetup.RotorRadius;     % Lift Prop Radius
    c = Vehicle.Geom.Pylons_1.MaxWidth;

    Download_TfactLP = BoomDownloadFactor(R,c,TipMount);

    Fxhub_RA = Fhub_RA(1,:); Fyhub_RA = Fhub_RA(2,:); Fzhub_RA = Fhub_RA(3,:) * Download_TfactLP;
    Mxhub_RA = Mhub_RA(1,:); Myhub_RA = Mhub_RA(2,:); Mzhub_RA = Mhub_RA(3,:);

    % zero out forces and moments of inactive rotors
    Fxhub_RA(InactiveRotors) = 0;
    Fyhub_RA(InactiveRotors) = 0;
    Fzhub_RA(InactiveRotors) = 0;
    Mxhub_RA(InactiveRotors) = 0;
    Myhub_RA(InactiveRotors) = 0;
    Mzhub_RA(InactiveRotors) = 0;

    % aerodynamic drag torque on rotors
    LiftPropTorques = abs(Mzhub_RA);


    % Convert hub forces and moments to body-fixed axes
    Fxhub_bf =  cosd(th_RA).*Fxhub_RA  +  sind(th_RA).*sind(ph_RA).*Fyhub_RA  + sind(th_RA).*cosd(ph_RA).*Fzhub_RA;
    Fyhub_bf =                                       + cosd(ph_RA).*Fyhub_RA               - sind(ph_RA).*Fzhub_RA;
    Fzhub_bf = -sind(th_RA).*Fxhub_RA  +  cosd(th_RA).*sind(ph_RA).*Fyhub_RA  + cosd(th_RA).*cosd(ph_RA).*Fzhub_RA;

    Mxhub_bf =  cosd(th_RA).*Mxhub_RA  +  sind(th_RA).*sind(ph_RA).*Myhub_RA  + sind(th_RA).*cosd(ph_RA).*Mzhub_RA;
    Myhub_bf =                                       + cosd(ph_RA).*Myhub_RA               - sind(ph_RA).*Mzhub_RA;
    Mzhub_bf = -sind(th_RA).*Mxhub_RA  +  cosd(th_RA).*sind(ph_RA).*Myhub_RA  + cosd(th_RA).*cosd(ph_RA).*Mzhub_RA;


    % compute the rotor forces and moments with respect to vehicle reference
    % point (RP)
    FxRPlp_bf = Fxhub_bf; FyRPlp_bf = Fyhub_bf; FzRPlp_bf = Fzhub_bf;

    MxRPlp_bf = Mxhub_bf                    + yloc.*Fzhub_bf   - zloc.*Fyhub_bf;
    MyRPlp_bf = Myhub_bf  - xloc.*Fzhub_bf                     + zloc.*Fxhub_bf;
    MzRPlp_bf = Mzhub_bf  + xloc.*Fyhub_bf  - yloc.*Fxhub_bf;



    % Sum the forces and moments generated by cruise propulsors
    F_liftprops = [sum(FxRPlp_bf);sum(FyRPlp_bf);sum(FzRPlp_bf)];
    M_liftprops = [sum(MxRPlp_bf);sum(MyRPlp_bf);sum(MzRPlp_bf)];

    % Angular momentum of spinning rotors
    N = LPSetup.SpinDir.*LPRPMs'*2*pi/60;
    Hr_RA = Vehicle.Geom.Prop_5.Izz_ca * N;

    LiftPropPwrkW = abs(N.*LiftPropTorques)/1000;

    % Convert angular momentum and its derivative to body-fixed axes
    Hrx_bf =  + sind(th_RA).*cosd(ph_RA).*Hr_RA;
    Hry_bf =  - sind(ph_RA).*Hr_RA;
    Hrz_bf =  + cosd(th_RA).*cosd(ph_RA).*Hr_RA;
    % sum these up
    Hr_liftprops = [sum(Hrx_bf);sum(Hry_bf);sum(Hrz_bf)];

    % reshape XInflow_out if being evaluated directly from MATLAB
    if EvalMode~=1
        %XInflow_out = reshape(Summary.XInflow,[1,7*LPSetup.nRotor]);
    end

end


% dt = 0:2:85;
% for ic = 1:length(dt) 
% dt2 = dt(ic);
%% CRUISE PROPULSOR LOADS
if ~all(CPRPMs < PropCutoffRPM)
    FlowCon.rho = rho;
    FlowCon.SpeedOfSound = a;
    FlowCon.DynVisc = 0.17000E-04;
    
    % Note 1: th_ra = 0 when rotor axis points "up"
    % Note 2: th_ra = -90 when rotor axis points "forward
    % Note 3: dW wing angles are = 0 in fwd position, = 90 in vertical posn
   
    dth = [dt1 dt2 dt3 dt4];
    
    cosd_dth = cosd(dth); sind_dth = sind(dth);
    
    CPSetup = Vehicle.Propulsion.CPSetup;
    
    th_RA = CPSetup.RotorAxisTheta + dth;
    ph_RA = CPSetup.RotorAxisPhi;
    
    
    % propulsor hub locations
    xloc = Vehicle.Propulsion.CPSetup.HingeLoc(1,:) + ...
        Vehicle.Propulsion.CPSetup.HingeToHub(1,:).*cosd_dth + ...
        Vehicle.Propulsion.CPSetup.HingeToHub(3,:).*sind_dth;
    
    yloc = Vehicle.Propulsion.CPSetup.HingeLoc(2,:);
    
    zloc = Vehicle.Propulsion.CPSetup.HingeLoc(3,:) + ...
        - Vehicle.Propulsion.CPSetup.HingeToHub(1,:).*sind_dth + ...
        Vehicle.Propulsion.CPSetup.HingeToHub(3,:).*cosd_dth;
    
    HealthStatus = Vehicle.Propulsion.CPSetup.HealthStatus; %ones(1,LPSetup.nRotor);
    InactiveRotors = HealthStatus==0;
    
    % Find the indices of individual rotors that fall below threshold RPM
    % (if any). Set the RPM of these to the threshold RPM for running the
    % BEMT code. Then, set the computed forces and moments for these rotors
    % to zero after the fact
    
    % compute velocities seen at the rotor hubs, resolved in body-fixed axes
    uhub_bf = uw  +  0.*xloc   - rw.*yloc   +  qw.*zloc;
    vhub_bf = vw  + rw.*xloc   +  0.*yloc   -  pw.*zloc;
    whub_bf = ww  - qw.*xloc   + pw.*yloc   +   0.*zloc;
    
    % convert these velocities from body-fixed axes to rotor aligned axes
    u_RA =              cosd(th_RA).*uhub_bf                                          - sind(th_RA).*whub_bf;
    v_RA = sind(th_RA).*sind(ph_RA).*uhub_bf  + cosd(ph_RA).*vhub_bf     + cosd(th_RA).*sind(ph_RA).*whub_bf;
    w_RA = sind(th_RA).*cosd(ph_RA).*uhub_bf  - sind(ph_RA).*vhub_bf     + cosd(th_RA).*cosd(ph_RA).*whub_bf;
    Vhub_RA = reshape([u_RA;v_RA;w_RA],[3,1,CPSetup.nRotor]);
    
    % convert vehicle angular velocity from body-fixed axes to rotor aligned axes
    p_RA =              cosd(th_RA).*pw                                  - sind(th_RA).*rw;
    q_RA = sind(th_RA).*sind(ph_RA).*pw   + cosd(ph_RA).*qw + cosd(th_RA).*sind(ph_RA).*rw;
    r_RA = sind(th_RA).*cosd(ph_RA).*pw   - sind(ph_RA).*qw + cosd(th_RA).*cosd(ph_RA).*rw;
    Om_RA = [p_RA;q_RA;r_RA];
    Om_RA = repmat(Om_RA,[1,1,CPSetup.nRotor]);
    
    CPSetup.InflowGuess = interp1([0 Vehicle.Operations.VC],[15 1],TAS/0.51444,'linear','extrap') .* ones(1,CPSetup.nRotor);
    % Get forces and moments at the rotor hubs, resolved in rotor axes (RA)
    [Fhub_RA,Mhub_RA,Xrotor_dot_CP,Summary,ElemWise,XInflowCP_out] = EvalRotorFM(CPSetup,FlowCon,CPRPMs,CPBladePitch,XInflow_CP,Vhub_RA,Om_RA,HealthStatus,EvalMode);
    
    % % Inflow velocity at rotors
    wCP = max(0,XInflowCP_out(1,:));
    wCP(InactiveRotors==1)  = 0 ;
    vi = mean(wCP);
    
    % wtest = (1/2)*(-Vtest + sqrt(Vtest.^2 + 2*Ttest./(rho.*Vehicle.Propulsion.CPSetup.DiskArea)));
    
    % % Download due to the booms below the lift props, as a "Thrust Loss"
    % % Assumptions:
    % % 1. Booms have the same width (c) along majority of the longitudinal
    % % axis calculation based on the max-width of the boom
    % % 2. All the lift props are of same radius (R)
    % % 3. The download is assumed to be affecting the rotor axes forces in
    % % 'z' direction only
    
    TipMount = 1;                                     % (1) if the props are mounted at tip
    R = Vehicle.Propulsion.CPSetup.RotorRadius;       % Prop Radius
    c23 = Vehicle.Geom.Pylons_2.MaxWidth;
    
    % the effective radius will be sin of prop incidence angle
    R2 = R * sind(dt2);
    R3 = R * sind(dt3);
    
    Download_Tfact_CP2 = BoomDownloadFactor(R2,c23,TipMount);
    Download_Tfact_CP3 = BoomDownloadFactor(R3,c23,TipMount);
    
    % % Apply to prop 2 and 3 since they are boom mounted
    Fhub_RA(3,2) = Fhub_RA(3,2) .* Download_Tfact_CP2;
    Fhub_RA(3,3) = Fhub_RA(3,3) .* Download_Tfact_CP3;
    
    % % For prop 1 and 4, the download will be because of the interference from
    % % downstream wing.
    
    c14 = Vehicle.Geom.LWing.Stn.c(end);
    
    % the effective radius will be sine of prop incidence angle
    R1 = R * sind(dt1);
    R4 = R * sind(dt4);
    
    Download_Tfact_CP1 = BoomDownloadFactor(R1,c14,TipMount);
    Download_Tfact_CP4 = BoomDownloadFactor(R4,c14,TipMount);
    
    % % Apply to prop 2 and 3 since they are boom mounted
    Fhub_RA(3,1) = Fhub_RA(3,1) .* Download_Tfact_CP1;
    Fhub_RA(3,4) = Fhub_RA(3,4) .* Download_Tfact_CP4;
    
    % %
    
    Fxhub_RA = Fhub_RA(1,:); Fyhub_RA = Fhub_RA(2,:); Fzhub_RA = Fhub_RA(3,:);
    Mxhub_RA = Mhub_RA(1,:); Myhub_RA = Mhub_RA(2,:); Mzhub_RA = Mhub_RA(3,:);
    
    % zero out forces and moments of inactive rotors
    Fxhub_RA(InactiveRotors) = 0;
    Fyhub_RA(InactiveRotors) = 0;
    Fzhub_RA(InactiveRotors) = 0;
    Mxhub_RA(InactiveRotors) = 0;
    Myhub_RA(InactiveRotors) = 0;
    Mzhub_RA(InactiveRotors) = 0;
    
    % aerodynamic drag torque on rotors
    PropTorques = abs(Mzhub_RA);
    
    % Convert hub forces and moments to body-fixed axes
    Fxhub_bf =  cosd(th_RA).*Fxhub_RA  +  sind(th_RA).*sind(ph_RA).*Fyhub_RA  + sind(th_RA).*cosd(ph_RA).*Fzhub_RA;
    Fyhub_bf =                                       + cosd(ph_RA).*Fyhub_RA               - sind(ph_RA).*Fzhub_RA;
    Fzhub_bf = -sind(th_RA).*Fxhub_RA  +  cosd(th_RA).*sind(ph_RA).*Fyhub_RA  + cosd(th_RA).*cosd(ph_RA).*Fzhub_RA;
    
    Mxhub_bf =  cosd(th_RA).*Mxhub_RA  +  sind(th_RA).*sind(ph_RA).*Myhub_RA  + sind(th_RA).*cosd(ph_RA).*Mzhub_RA;
    Myhub_bf =                                       + cosd(ph_RA).*Myhub_RA               - sind(ph_RA).*Mzhub_RA;
    Mzhub_bf = -sind(th_RA).*Mxhub_RA  +  cosd(th_RA).*sind(ph_RA).*Myhub_RA  + cosd(th_RA).*cosd(ph_RA).*Mzhub_RA;
    
    % compute the rotor forces and moments with respect to vehicle reference
    % point (RP)
    FxRP_bf = Fxhub_bf; FyRP_bf = Fyhub_bf; FzRP_bf = Fzhub_bf;
    
    MxRP_bf = Mxhub_bf                    + yloc.*Fzhub_bf   - zloc.*Fyhub_bf;
    MyRP_bf = Myhub_bf  - xloc.*Fzhub_bf                     + zloc.*Fxhub_bf;
    MzRP_bf = Mzhub_bf  + xloc.*Fyhub_bf  - yloc.*Fxhub_bf;
    
    % Sum the forces and moments generated by cruise propulsors
    F_prop = [sum(FxRP_bf);sum(FyRP_bf);sum(FzRP_bf)];
    M_prop = [sum(MxRP_bf);sum(MyRP_bf);sum(MzRP_bf)];
    
    
    % Angular momentum of spinning rotors
    N = CPSetup.SpinDir.*CPRPMs'*2*pi/60;
    
    Vehicle.Propulsion.PropInertia_kgm2 = Vehicle.Geom.Prop_1.Izz_ca;
    
    Hr_RA = Vehicle.Propulsion.PropInertia_kgm2 * N;
    
    
    CPPwrkW = (abs(N.*PropTorques))/1000;
    CruisePropTorques = PropTorques;
    
    
    
    % Convert angular momentum and its derivative to body-fixed axes
    Hrx_bf =  + sind(th_RA).*cosd(ph_RA).*Hr_RA;
    Hry_bf =  - sind(ph_RA).*Hr_RA;
    Hrz_bf =  + cosd(th_RA).*cosd(ph_RA).*Hr_RA;
    % sum these up
    Hr_props = [sum(Hrx_bf);sum(Hry_bf);sum(Hrz_bf)];
    
    % reshape XInflow_out if being evaluated directly from MATLAB
    if EvalMode~=1
        %     XInflow_out = reshape(Summary.XInflow,[1,7*6]);
    end
end


TotPwrkW = sum(CPPwrkW) + sum(LiftPropPwrkW);



%% NON-STRIP GEOMETRY AERODYNAMIC LOADS - FUSELAGE
CX_Fus = ActivateFusLoad*Vehicle.Aero.CX_Fus(AOADeg,abs(BETADeg)); % symmetric
CY_Fus = ActivateFusLoad*Vehicle.Aero.CY_Fus(AOADeg,abs(BETADeg))*sign(BETADeg); % direction accounted for
CZ_Fus = ActivateFusLoad*Vehicle.Aero.CZ_Fus(AOADeg,abs(BETADeg)); % symmetric
Cmx_Fus = ActivateFusLoad*Vehicle.Aero.Cmx_Fus(AOADeg,abs(BETADeg))*sign(BETADeg); % direction accounted for
Cmy_Fus = ActivateFusLoad*Vehicle.Aero.Cmy_Fus(AOADeg,abs(BETADeg)); % symmetric
Cmz_Fus = ActivateFusLoad*Vehicle.Aero.Cmz_Fus(AOADeg,abs(BETADeg))*sign(BETADeg); % direction accounted for

F_aero_Fus = qbar * Vehicle.Aero.FuselageRefArea * [CX_Fus ; CY_Fus ; CZ_Fus ];
M_aero_Fus = qbar * Vehicle.Aero.FuselageRefArea * Vehicle.Aero.FuselageRefLengthLong * [Cmx_Fus; Cmy_Fus; Cmz_Fus];



%% LANDING GEAR
% body to wind axis transformation (Snippet from Aashutosh, 2024.09.02) 
R_Beta = [cosd(BETADeg) sind(BETADeg) 0;
            -sind(BETADeg) cosd(BETADeg) 0;
            0 0 1];
R_minusalpha = [cosd(AOADeg) 0 sind(AOADeg);
           0 1 0
           -sind(AOADeg) 0 cosd(AOADeg)];
RWB =  R_Beta * R_minusalpha;
RBW = RWB';


CD_MG = ActivateLGLoad * Vehicle.Aero.MLG_dragcoeff;
CD_NG = ActivateLGLoad * Vehicle.Aero.NLG_dragcoeff;
S_ref = Vehicle.Recalcs.WingArea_TOT_m2;
F_LMG_W = qbar * S_ref * [-CD_MG;0;0];
F_RMG_W =  qbar * S_ref * [-CD_MG;0;0];
F_NG_W = qbar * S_ref * [-CD_NG;0;0];

F_LMG = RBW * F_LMG_W;
F_RMG = RBW * F_RMG_W;
F_NG = RBW * F_NG_W;
DragCP_MG = ceil(Vehicle.Geom.LandingGear_Main_1.Stn.i(end) /2) + 1;
DragCP_NG = ceil(Vehicle.Geom.LandingGear_Nose.Stn.i(end) /2) + 1;


r_LMG = [Vehicle.Geom.LandingGear_Main_1.Stn.xQC(DragCP_MG); Vehicle.Geom.LandingGear_Main_1.Stn.yQC(DragCP_MG); Vehicle.Geom.LandingGear_Main_1.Stn.zQC(DragCP_MG)];
r_RMG = [Vehicle.Geom.LandingGear_Main_2.Stn.xQC(DragCP_MG); Vehicle.Geom.LandingGear_Main_2.Stn.yQC(DragCP_MG); Vehicle.Geom.LandingGear_Main_2.Stn.zQC(DragCP_MG)];
r_NG = [Vehicle.Geom.LandingGear_Nose.Stn.xQC(DragCP_NG); Vehicle.Geom.LandingGear_Nose.Stn.yQC(DragCP_NG); Vehicle.Geom.LandingGear_Nose.Stn.zQC(DragCP_NG)];

M_LMG = cross(r_LMG,F_LMG);
M_RMG = cross(r_RMG,F_RMG);
M_NG = cross(r_NG,F_NG);

F_aero_LG = F_LMG + F_RMG + F_NG;
M_aero_LG = M_LMG + M_RMG + M_NG;


%% SUMMATION OF NON-STRIP GEOMETRY LOADS
F_aero_NLS = F_aero_Fus + ActivateLGLoad*F_aero_LG;
M_aero_NLS = M_aero_Fus + ActivateLGLoad*M_aero_LG;


%% DIRECT FLOW CALCULATION
% calculate velocity components at 25% chords of strips, expressed in aircraft body-fixed basis
us_b = uw  +  0.*xAC   - rw.*yAC   +  qw.*zAC;
vs_b = vw  + rw.*xAC   +  0.*yAC   -  pw.*zAC;
ws_b = ww  - qw.*xAC   + pw.*yAC   +   0.*zAC;

% Rotation from body fixed frame [B] to strip frame [S] --> R_BS
% X_S = R_SB * X_B
us_s = R_SB_11.*us_b + R_SB_12.*vs_b + R_SB_13.*ws_b;
%vs_s = Vehicle.Aero.StripDefn.R_SB_21.*us_b + Vehicle.Aero.StripDefn.R_SB_22.*vs_b + Vehicle.Aero.StripDefn.R_SB_23.*ws_b;
ws_s = R_SB_31.*us_b + R_SB_32.*vs_b + R_SB_33.*ws_b;

% compute geometric AOA accounting for direct flow
AOA_dir = atan2d(ws_s,(us_s+0.000000000001));

% compute dynamic pressure at each strip (ignore cross-flowt, use only xs- and zs- components)
V_s = sqrt(us_s.^2 + ws_s.^2);

%% Prop Slip Stream Calculation

% % Note 1: th_ra = 0 when rotor axis points "up"
% % Note 2: th_ra = -90 when rotor axis points "forward
% % Note 3: dW wing angles are = 0 in fwd position, = 90 in vertical posn
% dth = [dt1 dt2 dt3 dt4];
% 
% cosd_dth = cosd(dth); sind_dth = sind(dth);
% 
% CPSetup = Vehicle.Propulsion.CPSetup;
% 
% th_RA = CPSetup.RotorAxisTheta + dth;
% ph_RA = CPSetup.RotorAxisPhi;

% %%% ACCOUNT FOR FLOW ACCELERATION AND TURNING THROUGH PROP DISCS
% % Left Wing
KFactor = Vehicle.Propulsion.CPSetup.kdxbeta; 

dth = [dt1 dt2 dt3 dt4];
RotorAxisTheta = Vehicle.Propulsion.CPSetup.RotorAxisTheta + dth;
WingIncidenceAngle = Vehicle.Geom.RWing.RootIncidence ;

i_p = RotorAxisTheta + 90 - WingIncidenceAngle;

i_p(1:4) = 0;



% be mindful of the order of 'w' and 'KFactor', change it as follows
% order: O/B to I/B LW -->> I/B to O/B RW

% %%% ACCOUNT FOR FLOW ACCELERATION AND TURNING THROUGH PROP DISCS
% 1 --> 2 --> 3 --> 4
% Prop 1
IndP1 = LWIndices(Vehicle.Geom.LWing.Prop1.StripIndices);
V0 = V_s(IndP1); AOA0 = AOA_dir(IndP1); w0 = wCP(1); vip = w0 * KFactor(1);
V_s(IndP1) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));
AOA_dir(IndP1) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(1)),(V0.*cosd(AOA0) + vip.*cosd(i_p(1))));


%%% for Prop 2 and Prop 3, rotating slipstream is implemented
% compute slipstream axis grids from the prop disc as a function of "s", all the way
% to Horz Tail
% to prevent the interpolation blow up at 90 deg
dt2(dt2 >= 89.99) = 89.9999;  
dt3(dt3 >= 89.99) = 89.9999;


sP2 = linspace(0,abs(xloc(2) - Vehicle.Geom.LHTail.RefPtLocation(1)),20);
sP3 =  linspace(0,abs(xloc(3) - Vehicle.Geom.LHTail.RefPtLocation(1)),20);
sGrid = [sP2; sP3];


% % calculate kd parameters: kd = 1 + s/sqrt(s^2+R^2)
kd = 1 + sGrid./sqrt(sGrid.^2 + Vehicle.Propulsion.CPSetup.RotorRadius^2);

% % beta
beta = Vehicle.Propulsion.CPSetup.beta;


% % Prop 2
IndP2 = LWIndices(Vehicle.Geom.LWing.Prop2.StripIndices); % prop 2, left wing, inboard
V0 = V_s(IndP2); AOA0 = AOA_dir(IndP2); w0 = wCP(2); vip = w0 * KFactor(2);
AOA_s(IndP2) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(2)),(V0.*cosd(AOA0) + vip.*cosd(i_p(2))));


% % compute inclination wrt nacelle incidence using this AOA
AOA_slipstream = AOA_s;

% % Slipstream radius as a function of "s"
Rads_P2 = sqrt((V0(1).*cosd(AOA0(1)) + w0)./(V0(1).*cosd(AOA0(1)) + w0.*kd(1,:).*beta)) .* Vehicle.Geom.Prop_2.Diam/2;

% theta = slip stream inclination wrt nacelle angle
gamma = AOA_slipstream(1) - dt2;   

% Vertical component of contracted radius of the slipstream
zP2 = Rads_P2 .* cosd(gamma);       

% The axis will rotate by angle theta
sP2finalX = sP2 .* cosd(gamma);
sP2finalZ = sP2 .* sind(gamma);

% % capture X and Z coordinates after rotation 
xGridAfterRotation = xloc(2) - sP2finalX; 
zGridAfterRotation = zloc(2) - sP2finalZ;

% % coordinates of upper surface in XZ plane (axis Z coordinates + vertical comp of radius)
upperSurfOfSlipstream = zGridAfterRotation - zP2;

% query points, straight untapered wing (same x and z as ref point location)
zQuery = Vehicle.Geom.LWing.RefPtLocation(3);  

% compute query points at LE and TE
xQueryLE = Vehicle.Geom.LWing.Prop2.StripDef.xAC(1) + 0.25*Vehicle.Geom.LWing.Prop2.StripDef.Chord(1);
xQueryTE = xQueryLE - Vehicle.Geom.LWing.Prop2.StripDef.Chord(1);

zUpperSurfaceLE = interp1(xGridAfterRotation, upperSurfOfSlipstream, xQueryLE, 'nearest', 'extrap');
zUpperSurfaceTE = interp1(xGridAfterRotation, upperSurfOfSlipstream, xQueryTE, 'nearest', 'extrap');



% xGrid = Vehicle.Geom.Prop_2.RefPtLocation(1) - sP2;
% intersectX = interp1(upperSurfOfSlipstream, xGrid, zQuery,'nearest', 'extrap');
% interX(ic) = intersectX;

% if the upper surface includes zQuery at LE and TE
if zUpperSurfaceLE < zQuery && zUpperSurfaceTE < zQuery

    AOA_dir(IndP2) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(2)),(V0.*cosd(AOA0) + vip.*cosd(i_p(2))));
    V_s(IndP2) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));

else  

    xGrid = Vehicle.Geom.Prop_2.RefPtLocation(1) - sP2;
    intersectX = interp1(upperSurfOfSlipstream, xGrid, zQuery,'nearest', 'extrap');
    % interX(ic) = intersectX;

    if (intersectX < xQueryLE) && (intersectX > xQueryTE)

         % find chord fraction immersed in the streamtube        
        cImmersed = abs(xQueryLE - intersectX)/(xQueryLE - xQueryTE);

        vip = vip * cImmersed;

        AOA_dir(IndP2) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(2)),(V0.*cosd(AOA0) + vip.*cosd(i_p(2))));
        V_s(IndP2) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));    

    end    

end



% zUpperLE(ic) = zUpperSurfaceLE;
% zUpperTE(ic) = zUpperSurfaceTE;


% figure(1)
% hold on
% x = xloc(2) - sP2;
% y = yloc(2)*ones(length(x),1);
% % 
% % 
% scatter3(xQueryLE, y(1), zUpperSurfaceLE, 'filled');
% scatter3(xQueryTE, y(1), zUpperSurfaceTE, 'filled');
% 
% plot3(x, y, zGridAfterRotation, '--k','LineWidth',1.5);         % axis
% plot3(x, y, upperSurfOfSlipstream, '.-','LineWidth',1.5);    % upper surface
% end

% % Prop 3
IndP3 = RWIndices(Vehicle.Geom.RWing.Prop2.StripIndices); % prop 3, right wing, inboard
V0 = V_s(IndP3); AOA0 = AOA_dir(IndP3); w0 = wCP(3); vip = w0 * KFactor(3);

AOA_s(IndP3) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(3)),(V0.*cosd(AOA0) + vip.*cosd(i_p(3))));

% % compute inclination wrt nacelle incidence using this AOA
AOA_slipstream = AOA_s;

% % Slipstream radius as a function of "s"
Rads_P3 = sqrt((V0(1).*cosd(AOA0(1)) + w0)./(V0(1).*cosd(AOA0(1)) + w0.*kd(2,:).*beta)) .* Vehicle.Geom.Prop_3.Diam/2;

% theta = slip stream inclination wrt nacelle angle
gamma = AOA_slipstream(1) - dt3 ;   

% Vertical component of contracted radius of the slipstream
zP3 = Rads_P3 .* cosd(gamma);       

% The axis will rotate by angle theta
sP3finalX = sP3 .* cosd(gamma);
sP3finalZ = sP3 .* sind(gamma);

% % capture X and Z coordinates after rotation 
xGridAfterRotation = Vehicle.Geom.Prop_3.RefPtLocation(1) - sP3finalX; 
zGridAfterRotation = Vehicle.Geom.Prop_3.RefPtLocation(3) - sP3finalZ;

% % coordinates of upper surface in XZ plane (axis Z coordinates + vertical comp of radius)
upperSurfOfSlipstream = zGridAfterRotation - zP3;

% query points, straight untapered wing (same x and z as ref point location) 
zQuery = Vehicle.Geom.RWing.RefPtLocation(3);   


% compute query points at LE and TE
xQueryLE = Vehicle.Geom.RWing.Prop2.StripDef.xAC(1) + 0.25*Vehicle.Geom.RWing.Prop2.StripDef.Chord(1);
xQueryTE = xQueryLE - Vehicle.Geom.RWing.Prop2.StripDef.Chord(1);


zUpperSurfaceLE = interp1(xGridAfterRotation, upperSurfOfSlipstream, xQueryLE, 'nearest', 'extrap');
zUpperSurfaceTE = interp1(xGridAfterRotation, upperSurfOfSlipstream, xQueryTE, 'nearest', 'extrap');


% if the upper surface includes zQuery at LE and TE
if zUpperSurfaceLE < zQuery && zUpperSurfaceTE < zQuery

    AOA_dir(IndP3) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(3)),(V0.*cosd(AOA0) + vip.*cosd(i_p(3))));
    V_s(IndP3) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));

else  
   
    xGrid = Vehicle.Geom.Prop_3.RefPtLocation(1) - sP3;
    intersectX = interp1(upperSurfOfSlipstream, xGrid, zQuery);    

    if (intersectX < xQueryLE) && (intersectX > xQueryTE)

         % find chord fraction immersed in the streamtube        
        cImmersed = abs(xQueryLE - intersectX)/(xQueryLE - xQueryTE);
        
        vip = vip * cImmersed;

        AOA_dir(IndP3) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(3)),(V0.*cosd(AOA0) + vip.*cosd(i_p(3))));
        V_s(IndP3) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));    

    end    

end


% Prop 4
IndP4 = RWIndices(Vehicle.Geom.RWing.Prop1.StripIndices);
V0 = V_s(IndP4); AOA0 = AOA_dir(IndP4); w0 = wCP(4); vip = w0 * KFactor(4);
V_s(IndP4) = sqrt( (vip + V0.*cosd(AOA0)).^2 + (V0.*sind(AOA0).^2));
AOA_dir(IndP4) = atan2d(V0.*sind(AOA0) - vip.*sind(i_p(4)),(V0.*cosd(AOA0) + vip.*cosd(i_p(4))));

% Calling the  Function to calculate the loads at the strips
[dFx_AC_b,dFy_AC_b,dFz_AC_b,dMx_AC_b,dMy_AC_b,dMz_AC_b,AOA_eff] = EMPAERO_MainAeroFcnCall(rho,Vehicle,V_s,AOA_dir,Controls,R_BS);


% zero out loads from blocked strips
dFx_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;
dFy_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;
dFz_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;
dMx_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;
dMy_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;
dMz_AC_b(Vehicle.Aero.StripDefn.BLOCK==1) = 0;



% Transfer these loads from the AC of the individual strips to the
% force/moment reference point (RP) of the aircraft
dFx_RP_b = dFx_AC_b;
dFy_RP_b = dFy_AC_b;
dFz_RP_b = dFz_AC_b;


% dFz_RP_b(IndP1) = dFz_RP_b(IndP1) ;  % download P1
% dFz_RP_b(IndP3) = dFz_RP_b(IndP3) ;  % download P4

dMx_RP_b = dMx_AC_b                     - zAC.*dFy_RP_b     + yAC.*dFz_RP_b;
dMy_RP_b = dMy_AC_b  + zAC.*dFx_RP_b                        - xAC.*dFz_RP_b;
dMz_RP_b = dMz_AC_b  - yAC.*dFx_RP_b    + xAC.*dFy_RP_b                    ;


% sum these forces and moments to obtain the summed values
Fx_RP_b_strips = sum(dFx_RP_b);
Fy_RP_b_strips = sum(dFy_RP_b);
Fz_RP_b_strips = sum(dFz_RP_b);

Mx_RP_b_strips = sum(dMx_RP_b);
My_RP_b_strips = sum(dMy_RP_b);
Mz_RP_b_strips = sum(dMz_RP_b);

% Get 3 x 1 force vector
F_aero_strips = [Fx_RP_b_strips; Fy_RP_b_strips; Fz_RP_b_strips];
% Get 3 x 1 moment vector
M_aero_strips = [Mx_RP_b_strips; My_RP_b_strips; Mz_RP_b_strips];






%% AERO LOADS SUMMATION
% total aero loads: sum of loads from lifting and non-lifting components
F_aero = F_aero_NLS + ActivateLSLoad*F_aero_strips;
M_aero = M_aero_NLS + ActivateLSLoad*M_aero_strips;



%% FORCE, MOMENT, and ANGULAR MOMENTA SUMMATION
Fbf = F_aero + F_grav  + F_prop  + F_liftprops;
Mbf = M_aero + M_grav  + M_prop  + M_liftprops;

Hr = Hr_liftprops + Hr_props;
Hrdot = [0;0;0];



%% inflow
Xrotor_dot = [Xrotor_dot_LP , Xrotor_dot_CP];


%% ADDITIONAL OUTPUTS
% If performing AOA sweep 
Lift = (F_aero(1)*sind(AOADeg) - F_aero(3)*cosd(AOADeg));
Drag = (-F_aero(1)*cosd(AOADeg) - F_aero(3)*sind(AOADeg));
Moment = M_aero(2);

% If performing Beta sweep (also see Cm_CG calculation)
% Lift = F_aero(1)*sind(BETADeg) - F_aero(2)*cosd(BETADeg);
% Drag = -F_aero(1)*cosd(BETADeg) - F_aero(2)*sind(BETADeg);
% Moment = M_aero(3);

LDRatio = Lift/Drag;
CL = Lift/(qbar * Vehicle.Aero.RefArea);
CD = Drag/(qbar * Vehicle.Aero.RefArea);

Cmy = Moment/(qbar * Vehicle.Aero.RefArea * Vehicle.Aero.RefLengthLong);




r_cg_tilde = [...
    0,          -r_cg(3),       r_cg(2)
    r_cg(3),        0,          -r_cg(1)
    -r_cg(2),   r_cg(1),        0];
Fxgrav = -m*g*sin(th);
Fygrav =  m*g*sin(ph)*cos(th);
Fzgrav =  m*g*cos(ph)*cos(th);
F_grav = [Fxgrav;Fygrav;Fzgrav];
Fbf0 = F_aero + F_grav  + F_prop + F_liftprops;

M_grav = [...
    0,          -r_cg(3),       r_cg(2)
    r_cg(3),        0,          -r_cg(1)
    -r_cg(2),   r_cg(1),        0] * F_grav;


Mbf0 = M_aero + M_grav  + M_prop + M_liftprops;


LoadTable.AOA = repmat(AOADeg,[6,1]);


LoadTable.Fus = [F_aero_Fus; M_aero_Fus - r_cg_tilde * F_aero_Fus];
LoadTable.LG = [F_aero_LG; M_aero_LG - r_cg_tilde * F_aero_LG];

% Left Wing
LWingForces = [sum(dFx_RP_b(LWIndices)); sum(dFy_RP_b(LWIndices)); sum(dFz_RP_b(LWIndices))];
LWingMoments_RP =   [sum(dMx_RP_b(LWIndices)); sum(dMy_RP_b(LWIndices)); sum(dMz_RP_b(LWIndices))];
LWingMoments_CG = LWingMoments_RP - r_cg_tilde * LWingForces;
LoadTable.LWing = [LWingForces; LWingMoments_CG];


% Right Wing
RWingForces = [sum(dFx_RP_b(RWIndices)); sum(dFy_RP_b(RWIndices)); sum(dFz_RP_b(RWIndices))];
RWingMoments_RP =   [sum(dMx_RP_b(RWIndices)); sum(dMy_RP_b(RWIndices)); sum(dMz_RP_b(RWIndices))];
RWingMoments_CG = RWingMoments_RP - r_cg_tilde * RWingForces;
LoadTable.RWing = [RWingForces; RWingMoments_CG];

% Left HT 
LHTForces = [sum(dFx_RP_b(LHTIndices)); sum(dFy_RP_b(LHTIndices)); sum(dFz_RP_b(LHTIndices))];
LHTMoments_RP =   [sum(dMx_RP_b(LHTIndices)); sum(dMy_RP_b(LHTIndices)); sum(dMz_RP_b(LHTIndices))];
LHTMoments_CG = LHTMoments_RP - r_cg_tilde * LHTForces;
LoadTable.LHT = [LHTForces; LHTMoments_CG];

% Rihgt HT 
RHTForces = [sum(dFx_RP_b(RHTIndices)); sum(dFy_RP_b(RHTIndices)); sum(dFz_RP_b(RHTIndices))];
RHTMoments_RP =   [sum(dMx_RP_b(RHTIndices)); sum(dMy_RP_b(RHTIndices)); sum(dMz_RP_b(RHTIndices))];
RHTMoments_CG = RHTMoments_RP - r_cg_tilde * RHTForces;
LoadTable.RHT = [RHTForces; RHTMoments_CG];

% vertical tail
VSForces = [sum(dFx_RP_b(VSIndices)); sum(dFy_RP_b(VSIndices)); sum(dFz_RP_b(VSIndices))];
VSMoments_RP =   [sum(dMx_RP_b(VSIndices)); sum(dMy_RP_b(VSIndices)); sum(dMz_RP_b(VSIndices))];
VSMoments_CG = VSMoments_RP - r_cg_tilde * VSForces;
LoadTable.VS = [VSForces; VSMoments_CG];


LoadTable.AERO = [F_aero; M_aero];


% main prop loads
MPForces = [FxRP_bf; FyRP_bf; FzRP_bf];
MPMoments = [MxRP_bf;MyRP_bf;MzRP_bf];
MPMoments_CG = [MPMoments(:,1) - r_cg_tilde * MPForces(:,1), MPMoments(:,2) - r_cg_tilde * MPForces(:,2), MPMoments(:,3) - r_cg_tilde * MPForces(:,3), MPMoments(:,4) - r_cg_tilde * MPForces(:,4)];
LoadTable.MP = [MPForces; MPMoments_CG];


% lift prop loads
LPForces = [FxRPlp_bf; FyRPlp_bf; FzRPlp_bf];
LPMoments = [MxRPlp_bf;MyRPlp_bf; MzRPlp_bf];
LPMoments_CG = [LPMoments(:,1) - r_cg_tilde * LPForces(:,1), LPMoments(:,2) - r_cg_tilde * LPForces(:,2)];
LoadTable.LP = [LPForces; LPMoments_CG];


% total prop loads
LoadTable.PROPS = [F_prop + F_liftprops; (M_prop + M_liftprops) - r_cg_tilde * (F_prop + F_liftprops)];

LoadTable.Grav = [F_grav; M_grav - r_cg_tilde * F_grav];

LoadTable.TOT = [Fbf0; Mbf0 - r_cg_tilde * Fbf0];

% LoadTable = struct2table(LoadTable);
 
% BP = Controls(19:24)'



 save('LoadCase.mat','LoadTable','AOADeg')



%% ADDITIONAL OUTPUTS TO BE PROPAGATED
LDe = Vehicle.MassProp.MTOM_kg * g * TAS / (TotPwrkW*1000);
AddlOutputs = [LiftPropPwrkW, CPPwrkW, TotPwrkW, CL, LDRatio, LDe, LPRPMs', CPRPMs', CD, Cmy];

stopper = 1;


% % Download on the booms under the props, giving a thrust factor causing
% % reduction in total thrust from the props
    function Download_Tfact = BoomDownloadFactor(R,c,TipMount)

        %% Input
        % R = Radius of the rotor
        % c = chord or width of the boom
        % TipMount = 0,  (1 = tip mounted props)

        %% calculation based on eqn (9) [Ref: AIAA Aviation 2019, Kraenzler, Matthias et al]
        if R > c
            phi = 2*asin(0.5*c / R);

            if TipMount == 1
                Download_Tfact = (1 - phi / (2*pi)) ^ (1/3);
            else
                % [accounting for the fact that prop interferes with boom twice per rotation]
                Download_Tfact = ((1 - phi / (pi)) ^ (1/3));
            end
        else
            Download_Tfact = 1;  % if R == 0, no interference
        end


    end

end


% hold on
% for i = 1:length(s)
%     r = Rads_P1(1,i) ;
%     teta=-pi:0.01:pi;
%     y=r*cos(teta);
%     z=r*sin(teta);
%     scatter3(zeros(1,numel(y)) + Vehicle.Geom.Rotor_1.RefPtLocation(1) - s(1,i), y + Vehicle.Geom.Rotor_1.RefPtLocation(2), z + Vehicle.Geom.Rotor_1.RefPtLocation(3),'.')
% end

% 
% % % uncomment to check slipstream surface
% 
% figure(1)
% hold on
% x = Vehicle.Geom.Rotor_2.RefPtLocation(1) - sP2;
% y = Vehicle.Geom.Rotor_2.RefPtLocation(2)*ones(length(x),1);
% 
% 
% 
% scatter3(xQueryLE, y(1), zUpperSurfaceLE, 'filled');
% scatter3(xQuery,   y(1), zUpperSurface, 'filled');
% scatter3(xQueryTE, y(1), zUpperSurfaceTE, 'filled');
% 
% plot3(x, y, zGridAfterRotation, '--k','LineWidth',1.5);         % axis
% plot3(x, y, upperSurfOfSlipstream, '.-','LineWidth',1.5);    % upper surface


% %%% Download from P1 on wing strips
% % Reader page: 15
% % https://dspace-erf.nlr.nl/server/api/core/bitstreams/365aba6d-8c8e-4669-b515-e8301911d4ae/content 
% 
% beta = Vehicle.Propulsion.CPSetup.beta;
% kd14 = [Vehicle.Propulsion.CPSetup.kd(1) Vehicle.Propulsion.CPSetup.kd(4)];
% 
% % % Slipstream radius
% Rs1 = sqrt((V0(1).*cosd(AOA0(1)) + w0)./(V0(1).*cosd(AOA0(1)) + w0.*kd14(1).*beta)) .* Vehicle.Geom.Rotor_2.Diam/2;
% 
% A1 = 0.5 * rho * (0.5*Vehicle.Geom.Rotor_1.Diam)^3 / Rs1^3;  % terms out of integral
% 
% c1 = Vehicle.Geom.LWing.Rotor_1.StripDef.Chord;              % flaps up chord
% 
% Cd1 = Vehicle.Aero.GI_Cd_AOA_wing(AOA_dir(IndP1));           % sectional drag  
% 
% ds1 =  Vehicle.Geom.LWing.Rotor_1.StripDef.db_mid;
% 
% B1 =  c1.*sind(dt1) .* Cd1 .* (w0 * length(c1)).^2 .* ds1;
% 
% % % Download
% DL1 = A1 .* B1;
