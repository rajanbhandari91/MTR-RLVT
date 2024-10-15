function [Residuals,Soln] = CalcResiduals(TrimVec,FltCon,Vehicle,m,r_cg,EvalMode,Settings)

% initializations
Soln = [];



% hardcoded constants
g = 9.81;                   % acceleration due to gravity, m/s/s


% Vehicle mass and CG location are inputs to this function.
% Capture the vehicle's moment of inertia tensor
Icb = Vehicle.MassProp.MOI;




% capture the flight condition
h = FltCon.Alt_m;           % altitude (m)
V = FltCon.TAS;             % velocity (m/s)
FPADeg = FltCon.FPA;        % flightpath angle (deg)
RV = FltCon.RV;             % radius of curvature in vertical plane (m)

% capture wind conditions
WindSpd = FltCon.WindSpd * 0.5144;      % Convert wind speed from kt to m/s
WindDir = FltCon.WindDir;               % Wind direction (deg)
Updraft = FltCon.Updraft * 0.5144;      % convert from kt to m/s

% calculate Vwind in inertial frame
Vwi = [-WindSpd*cosd(WindDir);-WindSpd*sind(WindDir);-Updraft];

% download the elements of the TrimVector
AOADeg = TrimVec(1);        % angle of attack, deg
BETADeg = TrimVec(2);       % sideslip angle, deg
phDeg = TrimVec(3);         % deg
psdotDegs = TrimVec(4);     % deg/s
psDeg = TrimVec(5);         % deg
ctr = 5;

% Note: The above 5 elements will always be solved for during trim. 
% In addition, if FPA is not specified, it will also be solved for
if isempty(FPADeg)
    FPADeg = TrimVec(6);
    ctr = 6;
end



% Based on the control mapping being used, determine the elements of the
% control vector U
UTrimVec = TrimVec(ctr+1:end);
SpecControls = FltCon.SpecControls;
SpecAddlStates = FltCon.SpecAddlStates;
MappingNum = Settings.MappingNum;

[U,AddlStates,LB,UB,UNames] = ControlMapper(UTrimVec,SpecControls,SpecAddlStates,FltCon,MappingNum,TrimVec);
Controls = U;

% unit changes:
AOA = AOADeg * pi/180;     % deg to rad
BETA = BETADeg * pi/180;   % deg to rad
ph = phDeg * pi/180;       % deg to rad
ps = psDeg * pi/180;       % deg to rad
psdot = psdotDegs * pi/180; % deg/s to rad/s
FPA = FPADeg * pi/180;     % deg to rad










%% KINEMATIC RELATIONSHIPS (VEHICLE INDEPENDENT)

% order of state vector elements
% u, v, w, p, q, r, xdot, ydot, zdot, phdot, thdot, psdot

% set theta as given in formula for turning flight
th = FPA + AOA*cos(ph) + BETA*sin(ph);

D321 = [cos(ps),-sin(ps),0;sin(ps),cos(ps),0;0,0,1]*[cos(th),0,sin(th);0,1,0;-sin(th),0,cos(th)]*[1,0,0;0,cos(ph),-sin(ph);0,sin(ph),cos(ph)];
Vwb = D321' * Vwi;

% compute body-fixed velocities u, v, w
u = V*cos(BETA)*cos(AOA)    +Vwb(1);
v = V*sin(BETA)             +Vwb(2);
w = V*cos(BETA)*sin(AOA)    +Vwb(3);

% calculate theta-dot and psi-dot based on flight path vertical and
% horizontal radii of curvature
% psdot = V/RH;       % rad/sec
thdot = V/RV;       % rad/sec

% set up dimensional roll, pitch, and yaw rates
p = -psdot*sin(th);
q = thdot*cos(ph) + psdot*sin(ph)*cos(th);
r = -thdot*sin(ph) + psdot*cos(ph)*cos(th);

States.AirData.AOA = AOA;
States.AirData.AOADeg = AOA*180/pi;
States.AirData.AOAdot = 0;
States.AirData.BETA = BETA;

States.AirData.TAS = V;
States.AirData.MACH = FltCon.Mach;
States.AirData.pw = p;
States.AirData.qw = q;
States.AirData.rw = r;

States.AirData.uw = V*cos(BETA)*cos(AOA);
States.AirData.vw = V*sin(BETA);
States.AirData.ww = V*cos(BETA)*sin(AOA);

[T,a,P,rho] = atmosisa(h);
States.AirData.a = a;
States.AirData.rho = rho;
States.AirData.qbar = 0.5*rho*V^2;
States.AirData.EAS = States.AirData.TAS * sqrt(States.AirData.rho/1.225);
States.AirData.P = P;
States.AirData.OAT = T;

States.Attitude.ph = ph;
States.Attitude.th = th;

States.Posn.AltMSL = h;
States.Posn.AltAGL = h;


%% CALCULATE FORCES AND MOMENTS
XInflow = [];
[Fbf,Mbf,Hr,Hrdot,AddlRes,VehStatesDot,AddlOutputs] = FMComp(States,Controls,XInflow,Vehicle,m,r_cg,EvalMode);
%[Fbf,Mbf,Hr,Hrdot,Xrotor_dot,AddlRes,AddlOutputs] = FMComp(States,Controls,XInflow,Vehicle,m,r_cg,EvalMode);
%[Fbf,Mbf,Hr,Hrdot,AddlRes,AddlOP] = FMComp(States,Controls,Vehicle,m,r_cg,EvalMode);


%% EVALUATE EQUATIONS OF MOTION (VEHICLE INDEPENDENT)

Icbdot = zeros(3,3);        % rate of change of inertia tensor, set to zero
v_cg = zeros(3,1);          % CG velocity, set to zero
a_cg = zeros(3,1);          % CG acceleration, set to zero

Om = [p;q;r];               % Body fixed angular velocity vector
Vb = [u;v;w];               % Body fixed translational velocity vector
LLR = [0;0;0];              % Latitude, Longitude, Geocentric Radius, set to zero for trimming

[~,~,~,~,Quats] = ConvEulerToQuats(ph,th,ps);

[Vbdot,Omdot,~,~] =External_AircraftEOM_OffsetFromCG(Fbf,Mbf,m,Icb,Icbdot,r_cg,v_cg,a_cg,Om,Quats,Vb,LLR,Hr,Hrdot);

phi = ph; theta = th; psi = ps;
LBV = [...
    cos(theta)*cos(psi),                                cos(theta)*sin(psi),                                -sin(theta);
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     sin(phi)*cos(theta);
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),     cos(phi)*cos(theta)];
LVB = LBV';

Xedot = LVB*Vb;


%% LOAD FACTOR CALCULATIONS
% gravity resolved in body axes
gb = g*[-sin(th);cos(th)*sin(ph);cos(th)*cos(ph)];

% acceleration of CG resolved in body axes % Vbdot + omega x Vb
acb = Vbdot + cross([p;q;r],[u;v;w]);

% load factors - calculate as per formula
nvec = (gb-acb)/g;

% find load factor in the x-z plane
nxz = sqrt(nvec(3)^2+nvec(1)^2);

% load factor in the body-fixed y-direction (zero for coordinated flight)
ny = nvec(2);


%%  SET RESIDUALS
% Take the first six elements of X_Dot
% These are the residuals of the equations of motion
% udot, vdot, wdot, pdot, qdot, rdot (must all be driven to zero)
EOMRes = [Vbdot;Omdot];

% turn coordination residual (i.e., drive ny to zero)
coordres = ny;
CustomResidual = coordres;

% However, if heading is specified, then it may not be possible to achieve
% coordinated flight. In that case, replace with heading residual
if ~isempty(FltCon.HDG)
    CustomResidual = FltCon.Heading*pi/180 - ps;
end

% Also, if sideslip is specified, then it may not be possible to achieve
% coordinated flight. In that case, replace with sideslip residual
if ~isempty(FltCon.BETA)
    CustomResidual = FltCon.BETA*pi/180 - BETA;
end

% calculate trajectory radii in horizontal and vertical planes
TurnRadius = V*cos(FPA)/psdot;
VertRadius = V*cos(FPA)/thdot;
TurnResidual = [];

% if bank angle is given, set turn residual based on bank angle
if strcmpi(FltCon.Turn{1},'bank')
    TurnResidual = FltCon.Turn{2} - ph*180/pi;
end
% if turn rate is given, set turn residual based on turn rate
if strcmpi(FltCon.Turn{1},'turnrate')
    TurnResidual = FltCon.Turn{2} - psdot*180/pi;
end
% if turn radius is given, set turn residual based on turn radius
if strcmpi(FltCon.Turn{1},'turnradius')
    TurnResidual = FltCon.Turn{2} - TurnRadius;
end
% if turn load factor is given, set turn residual based on load factor
if strcmpi(FltCon.Turn{1},'loadfactor')
    TurnResidual = FltCon.Turn{2} - nxz;
end

% calculate flight path angle
FPACalc = -atand(Xedot(3)/(sqrt(Xedot(1)^2+Xedot(2)^2)));
FPACalc(isnan(FPACalc))=0;

% calculate aircraft track (TRK) and track residual
TRKCalc = mod(atan2d(Xedot(2),Xedot(1)),360);
TRKResidual = TRKCalc - FltCon.TRK;


% AOA residual
AOARes = [];
if ~isempty(FltCon.AOA)
   AOARes = FltCon.AOA -  AOADeg;
end

% Collect all applicable residuals into residual vector
Residuals = [EOMRes;CustomResidual;TRKResidual;TurnResidual;AOARes;AddlRes];

% EOMRes
% sto = 1;

if strcmpi(EvalMode,'AddlOP')
    Soln = AddlOP;
end

%% Gather solution (if called in 'report' mode)
if strcmpi(EvalMode,'report')
    
    TrimStates.AOA = AOADeg;
    TrimStates.BETA = BETADeg;
    TrimStates.FPA = FPACalc;
    TrimStates.FPM = (V*sind(TrimStates.FPA))*60*3.28;
    TrimStates = struct2table(TrimStates);
    
    
    TrimCtrl = array2table(UTrimVec');
    TrimCtrl.Properties.VariableNames = UNames.CTRL;
    TrimCtrl.UTrimVec = UTrimVec';
    
    
    
    Stat.PHI = ph*180/pi;
    Stat.HDG = ps*180/pi;
    Stat.TRK = TRKCalc;
    Stat.TurnRate = psdot*180/pi;
    Stat.nxz = nxz;
    Stat.ny = ny;
    Stat.RH = TurnRadius;
    Stat.RV = VertRadius;
    
    % capturing any additional outputs to be printed out
    if~isempty(Settings.AddlOPNames)
        for i = 1:length(Settings.AddlOPNames)
            Stat.(Settings.AddlOPNames{i}) = AddlOutputs(i);
        end
    end
    
    Stat.Controls = Controls';
    x = 0;
    y = 0;
    z = -h;
    Stat.X = [u,v,w,p,q,r,x,y,z,phi,theta,psi,Quats'];
    
    Stat.Residuals = Residuals';
    Stat = struct2table(Stat);
    
    
    Soln = [TrimStates,TrimCtrl,Stat];
    
    if~isempty(FltCon.SpecControls)
        SpecCtrl = array2table(FltCon.SpecControls(:,2)');
        SpecCtrl.Properties.VariableNames = UNames.SPECCTRL;
        Soln = [TrimStates,TrimCtrl,SpecCtrl,Stat];
    end
    
    
    
end