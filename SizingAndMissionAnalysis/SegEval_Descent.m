function [H, Vehicle] = SegEval_Descent(Mission,i,History,Vehicle)

% Profile options:
% 1: specified EAS or Mach
% 2: time-optimal profile
% 3: energy-optimal profile

% Termination options:
% 1. specified end altitude
% 2. end altitude based on optimum condition for following segment

% constants
g = 9.81;

% number of propulsor types
PropTypes = Mission.PropTypes;

% any ISA deviation
dISA = Mission.dISA;

% initial number of points
S = Mission.Segments(i);
np = S.Points;

[H] = SegmentInitializer(S,i,PropTypes,History);
[~,nprop] = size(H.dEdt);

% hardcodes
% hardcodes
ThrottleSetting = 0;
% if~isempty(S.PowerSetting)
%     ThrottleSetting = S.PowerSetting;
% end

% Initialize all masses to segment start mass
H.Mass = H.Mass(1) * ones(np,1);

% set eng altitude
Hfin = S.EndAlt;

% If no speed information is given, initialize all TAS to segment start TAS
if isempty(S.EAS)&&isempty(S.TAS)&&isempty(S.Mach)
    H.TAS = H.TAS(1) * ones(np,1);
    SpecifiedVelocity = 0;
end


% if speed setting is given as either EAS, TAS, or Mach, use it.
if ~isempty(S.EAS)
    SegVel = S.EAS*ones(np,1);
    VelType = 'EAS';
    SpecifiedVelocity = 1;
end
if ~isempty(S.TAS)
    SegVel = S.TAS*ones(np,1);
    VelType = 'TAS';
    SpecifiedVelocity = 1;
end
if ~isempty(S.Mach)
    SegVel = S.Mach*ones(np,1);
    VelType = 'Mach';
    SpecifiedVelocity = 1;
end


% Set up linearly spaced altitude grid points
H.Alt = [H.Alt(1);linspace(H.Alt(1),Hfin,np-1)'];

% initial guess for warm starting
X0 = [];

SolverType = 1;

% calculate energy height
H.EnHt = H.Alt + H.TAS.^2/(2*g);

if SpecifiedVelocity==0
    % get optimal velocity, given gross weight and energy height
    % leave first row unchanged
    
    H.TAS(2:end) = Mission.Optim.Descent.TAS_fn_GW_He(H.Mass(2:end),H.EnHt(2:end));
    [H.EAS,~,H.Mach,~,~,H.rho] = SetFltCon(H.Alt,dISA,'TAS',H.TAS);
end

if SpecifiedVelocity==1
    n = 2:length(H.Alt);
    [H.EAS(2:end),H.TAS(2:end),H.Mach(2:end),~,~,H.rho(2:end)] = SetFltCon(H.Alt(n),dISA,VelType,SegVel(n));
end

if ~isempty(S.ROD)
    H.FPA = -asin(S.ROD./H.TAS);
    H.dhdt = -S.ROD*ones(np,1);
    EvalMode = 'power';
else
    EvalMode = 'perf';
end


% take care of acceleration/deceleration
[H,Vehicle] = SegEval_Accel(Mission,i,H,Vehicle);


%query vehicle aero-propulsive performance
for j = 2:1:np
    [H.Ps(j),H.FPA(j),~,~,H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,1),X0] = SolveFltCon(H.Mass(j),ThrottleSetting,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),EvalMode,Vehicle,SolverType,X0);

    % calculate vertical speed
    H.dhdt(j) = H.TAS(j).*sin(H.FPA(j));
    
    % update mass for next timestep
    if j<np
        % calculate time increments
        H.dTime(j) = (H.Alt(j+1)-H.Alt(j))./H.dhdt(j);
        
        % calculate dV/dt
        H.dVdt(j) = (H.TAS(j+1)-H.TAS(j))./H.dTime(j);
        
        H.dMass(j) = sum(H.dMdt(j,:)) *  H.dTime(j);
        H.Mass(j+1) = H.Mass(j) + H.dMass(j);

        % integrate state derivatives
        H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * H.dTime(j);        
        % log current state
        Vehicle.States =  H.States(j+1,:);

        % energy
        H.Energy(j+1,:) = H.Energy(j,:) + H.dEdt(j,:) * H.dTime(j);

    end
    
    if(j>1&&j<np)
        % calculate dFPA/dt
        H.dFPAdt(j) = (H.FPA(j)-H.FPA(j-1))/H.dTime(j);
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

% Compute rate of climb
% H.dhdt(1:np-1) = diff(H.Alt)./H.dTime(1:np-1);

% Compute ground speed
H.GS = H.TAS.*cos(H.FPA);

% Compute distances traversed per time interval
H.dDist = [0.5 * (H.GS(1:end-1) + H.GS(2:end)).*H.dTime(1:end-1);0];

% Compute cumulative distance after each time interval
H.Dist(2:np) = H.Dist(1)+ cumsum(H.dDist(1:np-1));

% Compute cumulative time after each time interval
H.Time(2:np) = H.Time(1) + cumsum(H.dTime(1:np-1));

H.FLAG = FLAG;


fprintf(' %0.0f. %s->',i,S.Name)



% commented out:
% % set convergence flag and convergence counter
% converged = 0;
% k = 1;
%
% % begin segment convergence loop
% while converged == 0 && k <= maxiter
%
%     % calculate time increments
% %     if strcmpi(EvalMode,'perf')
% %         H.dTime(1:np-1) = diff(H.EnHt)./PsAvg;
% %     end
% %     if strcmpi(EvalMode,'power')
% %         H.dTime(1:np-1) = diff(H.Alt)./(H.TAS(1:np-1).*sin(H.FPA(1:np-1)));
% %     end
%
%     % check whether masses at each point on the segment have converged to
%     % specified tolerance
%     if(all(abs(H.Mass-OldMasses)<MassConvergenceThreshold))
%         converged = 1;
%     else
%         k=k+1;
%     end
%
%     % If final altitude is not specified, then update final altitude
%     % target based on updated final mass
%     if VarFinAlt==1
%         H.Alt(end) = Mission.Optim.Cruise.Alt_fn_GW(H.Mass(end));
%     end

% % convergence and iteration settings for climb convergence
% MassConvergenceThreshold = .1;
% maxiter =25;
% 
% if~isempty(forcedmaxiter)
%     maxiter = forcedmaxiter;
% end
% 
% % if end altitude is specified, then use it. If it is not specified, then
% % calculate the end altitude based on optimality of subsequent segment
% if ~isempty(S.EndAlt)
%     Hfin = S.EndAlt;
% else
%     Hfin = Mission.Optim.Cruise.Alt_fn_GW(S.StartMass);
%     VarFinAlt = 1;
% end
% 
% % average specific excess power
% PsAvg = 0.5*(H.Ps(1:np-1)+H.Ps(2:np));
% 
% % average mass change rate
% dMdtavg = 0.5*(sum(H.dMdt(1:np-1,:),2) +sum(H.dMdt(2:np,:),2));
% 
% % calculate mass decrements
% H.dMass(1:np-1) = H.dTime(1:np-1).*dMdtavg;
% 
% % update masses
% OldMasses = H.Mass;     % store old masses for convergence testing
% H.Mass(2:np) = H.Mass(1) + cumsum(H.dMass(1:np-1));
