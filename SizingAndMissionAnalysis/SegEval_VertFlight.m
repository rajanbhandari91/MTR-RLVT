function [H, Vehicle] = SegEval_VertFlight(Mission,i,History,Vehicle)

% Profile options:
% 1: specified EAS or Mach
% 2: best range cruise
% 3: best endurance cruise

% constants
g = 9.81;

% number of propulsor types
PropTypes = Mission.PropTypes;

% any ISA deviation
dISA = Mission.dISA;

% initial number of points
S = Mission.Segments(i);
np = S.Points;

% account for segment power split
ThrottleRatio = S.PowerSplit/sum(S.PowerSplit);

[H] = SegmentInitializer(S,i,PropTypes,History);
[~,nprop] = size(H.dEdt);


% Initialize all masses to segment start mass
H.Mass = H.Mass(1) * ones(np,1);

% If no speed information is given, initialize all TAS to segment start TAS
if 1%isempty(S.EAS)&&isempty(S.TAS)&&isempty(S.Mach)
    H.TAS = H.TAS(1) * ones(np,1);
    [H.EAS,H.TAS,H.Mach,~,~,H.rho] = SetFltCon(H.Alt,dISA,'TAS',H.TAS);
end


% if end altitude information is given, create a linear variation of
% altitudes from initial segment altitude to given final altitude
if ~isempty(S.EndAlt)
    H.Alt = linspace(H.Alt(1),S.EndAlt,np)';
end


% if speed setting is given as either EAS, TAS, or Mach, use it.
if ~isempty(S.EAS)
    SegVel = S.EAS*ones(np,1);
    VelType = 'EAS';
end
if ~isempty(S.TAS)
    SegVel = S.TAS*ones(np,1);
    VelType = 'TAS';
end
if ~isempty(S.Mach)
    SegVel = S.Mach*ones(np,1);
    VelType = 'Mach';
end


% check if segment duration itself is given. In that case, set the
% segment duration accordingly. There is no need for an outer
% convergence loop, so set outerconvergence flag to 1

TimeFound=0;

if Mission.Durations(i,i+1)~=0
    TimeInMode_min = Mission.Durations(i,i+1);
    TimeFound = 1;
end

if any(Mission.Durations(:,i+1))&&TimeFound==0
    [r,~] = find(Mission.Durations(:,i+1));
    STemp = History(History.SegNum==r,:);
    HackTime = STemp.Time(1);
    TimeInMode_min = Mission.Durations(r,i+1)- (History.Time(end)-HackTime);
end

TimeInMode = TimeInMode_min * 60;

% Set up linearly spaced time grid points
H.Time = linspace(H.Time(1),H.Time(1)+TimeInMode,np)';

% initial guess for warm starting
X0 = [];

SolverType = 1;



[H.EAS(2:end),H.TAS(2:end),H.Mach(2:end),~,~,H.rho(2:end)] = SetFltCon(H.Alt(2:end),dISA,VelType,SegVel(2:end));


% compute time increments
H.dTime(1:np-1) = diff(H.Time);

% Calculate dh/dt
H.dhdt(1:np-1) = diff(H.Alt)./diff(H.Time);

% calculate the flight path angle and ground speed
H.FPA = asin(H.dhdt./(H.TAS+0.0000001));

% compute acceleration dVdt
H.dVdt(1:np-1) = diff(H.TAS)./H.dTime(1:np-1);

% calculate dFPA/dt
H.dFPAdt(1:np-1) = 0; %diff(H.FPA)./H.dTime(1:np-1);

% query vehicle aero-propulsive performance
for j = 1:1:np
    [H.Ps(j),H.FPA(j),H.TAS(j),~,H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,1),X0] = SolveFltCon(H.Mass(j),ThrottleRatio,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'power',Vehicle,SolverType,X0);
    
    % update mass for next timestep
    if j<np
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

% Find average energy rate
% dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));

% Compute energy decrements per time interval
H.dEnergy(1:np-1,:) = H.dEdt(1:np-1,:).*H.dTime(1:np-1);

% Compute energy remaining
% H.Energy(2:np,:) = H.Energy(1,:) + cumsum(H.dEnergy(1:np-1,:));

% Compute energy "mass"
H.EMass = H.Energy./Vehicle.SOTA.SpecificEnergy_MJkg;

% % Compute ground speed
% H.GS = H.TAS.*cos(H.FPA);

% Compute distances traversed per time interval
H.dDist(1:np) = 0;

% Compute cumulative distance after each time interval
H.Dist(2:np) = H.Dist(1)+ cumsum(H.dDist(1:np-1));

% Compute cumulative time after each time interval
H.Time(2:np) = H.Time(1) + cumsum(H.dTime(1:np-1));

% Compute energy height
H.EnHt = H.Alt + H.TAS.^2/(2*g);

H.FLAG = FLAG;

fprintf(' %0.0f. %s->',i,S.Name)

