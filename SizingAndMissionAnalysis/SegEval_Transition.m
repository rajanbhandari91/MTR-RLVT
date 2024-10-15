function [H, Vehicle] = SegEval_Transition(Mission,i,History,Vehicle)

% Profile options:
% 1: specified EAS or Mach
% 2: time-optimal profile
% 3: energy-optimal profile

% Termination options:
% 1. specified end altitude
% 2. end altitude based on optimum condition for following segment

% constants
g = 9.81;

% default evalmode
EvalMode = 'accel';

% number of propulsor types
PropTypes = Mission.PropTypes;

% any ISA deviation
dISA = Mission.dISA;

% initial number of points
S = Mission.Segments(i);
np = S.Points;


[H] = SegmentInitializer(S,i,PropTypes,History);
[~,nprop] = size(H.dEdt);

% Initialize all masses to segment start mass
H.Mass = H.Mass(1) * ones(np,1);



% 1. set all altitudes to segment start altitude
H.Alt = H.Alt(1) * ones(np,1);

% 2. set up linspace of TAS
H.TAS = linspace(H.TAS(1),S.EndVel,np)';
VelType = 'tas';
SegVel = H.TAS;

if H.TAS(end) > H.TAS(1)
    Throttle = S.PowerSplit(1);   % acceleration
else
    Throttle = 0.0001;   % deceleration
end


% 3. if dhdt is given, calculate FPA. If FPA is given, calculate dhdt
if ~isempty(S.dhdt)
    H.dhdt = S.dhdt * ones(np,1);
    H.FPA = asin(H.dhdt./H.TAS);
    H.FPA(H.TAS==0)=0;
end
if ~isempty(S.FPA)
    H.FPA = S.FPA * ones(np,1);
    H.dhdt = H.TAS.*sin(H.FPA);
end

% 4. if end altitude is given
if ~isempty(S.EndAlt)
    H.FPA = zeros(np,1);
    H.dhdt = zeros(np,1);
    
end



% initial guess for warm starting
X0 = [];

SolverType = 1;




% calculate energy height
H.EnHt = H.Alt + H.TAS.^2/(2*g);



[H.EAS,H.TAS,H.Mach,~,~,H.rho] = SetFltCon(H.Alt,dISA,VelType,SegVel);

% if acceleration is specified, use it
if ~isempty(S.dVdt)
    H.dVdt = S.dVdt*ones(np,1);
    EvalMode = 'power';
end



% query vehicle aero-propulsive performance
for j = 1:1:np
    
    %[t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13] = SolveFltCon(H.Mass(j),FullThrottle,H.rho(j),H.TAS(j),H.FPA(j),H.dFPAdt(j),H.dVdt(j),'perf',Vehicle);
    [H.Ps(j),~,~,H.dVdt(j),H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,1),X0] = SolveFltCon(H.Mass(j),Throttle,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),EvalMode,Vehicle,SolverType);
    
    if Throttle == 0
        H.dVdt(j) = min(H.dVdt(j),-0.4);
    end
    
    % update mass for next timestep
    if j<np
        
        % calculate time increments
        H.dTime(j) = max(0,  (H.TAS(j+1)-H.TAS(j))/H.dVdt(j));
        
        H.dMass(j) = sum(H.dMdt(j,:)) *  H.dTime(j);
        H.Mass(j+1) = H.Mass(j) + H.dMass(j);

        % integrate state derivatives
        H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * H.dTime(j);        
        % log current state
        Vehicle.States = H.States(j+1,:);

        % energy
        H.Energy(j+1,:) = H.Energy(j,:) + H.dEdt(j,:) * H.dTime(j);        
    end
    
end




% calculate altitude increments
H.Alt(2:np) = H.Alt(1) + cumsum(H.dhdt(1:np-1).*H.dTime(1:np-1));


% Find average energy rate
% dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));

% Compute energy decrements per time interval
H.dEnergy(1:np-1,:) = H.dEdt(1:np-1,:).*H.dTime(1:np-1);

% Compute energy remaining
% H.Energy(2:np,:) = H.Energy(1,:) + cumsum(H.dEnergy(1:np-1,:));

% Compute energy "mass"
H.EMass = H.Energy./Vehicle.SOTA.SpecificEnergy_MJkg;

% Compute ground speed
H.GS = H.TAS.*cos(H.FPA);

% Compute distances traversed per time interval
H.dDist = [0.5 * (H.GS(1:end-1) + H.GS(2:end)).*H.dTime(1:end-1);0];

% Compute cumulative distance after each time interval
H.Dist(2:np) = H.Dist(1)+ cumsum(H.dDist(1:np-1));

% Compute cumulative time after each time interval
H.Time(2:np) = H.Time(1) + cumsum(H.dTime(1:np-1));

% calculate dFPA/dt
H.dFPAdt(1:np-1) = diff(H.FPA)./H.dTime(1:np-1);

H.FLAG = FLAG;


fprintf(' %0.0f. %s->',i,S.Name)

