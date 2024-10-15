function [H, Vehicle] = SegEval_Climb(Mission,i,History,Vehicle)

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

% hardcodes
% FullThrottle = ones(1,Mission.PropTypes).*S.PowerSplit/sum(S.PowerSplit);

FullThrottle = 1;

% default eval mode
EvalMode = 'perf';

[H] = SegmentInitializer(S,i,PropTypes,History);
[~,nprop] = size(H.dEdt);


[H.EAS(1),H.TAS(1),H.Mach(1),~,~,H.rho(1)] = SetFltCon(H.Alt(1),dISA,'TAS',H.TAS(1));


if(isfield(S,'dMass'))
    if~isempty(S.dMass)
        H.Mass(1) = H.Mass(1)+S.dMass;
    end
end



% Initialize all masses to segment start mass
H.Mass = H.Mass(1) * ones(np,1);

% set eng altitude
Hfin = S.EndAlt;


% If no speed information is given, initialize all TAS to segment start TAS
if isempty(S.EAS)&&isempty(S.TAS)&&isempty(S.Mach)
    H.TAS = H.TAS(1) * ones(np,1);
    SpecifiedVelocity = 0;
end
H.TAS = H.TAS(1) * ones(np,1);

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



% calculate energy height
H.EnHt = H.Alt + H.TAS.^2/(2*g);

% protect against decreasing energy height;
DiffEH = max(0,diff(H.EnHt));

H.EnHt(2:end) = H.EnHt(1) + cumsum(DiffEH);

H.Alt = H.EnHt - H.TAS.^2/(2*g);



if SpecifiedVelocity==1
    [H.EAS(2:end),H.TAS(2:end),H.Mach(2:end),~,~,H.rho(2:end)] = SetFltCon(H.Alt(2:end),dISA,VelType,SegVel(2:end));
end

if ~isempty(S.ROC)
    H.FPA = asin(S.ROC./H.TAS);
    H.dhdt = S.ROC*ones(np,1);
    EvalMode = 'power';
end

SolverType = 1;





% % take care of acceleration/deceleration
% if H.TAS(2) > H.TAS(1)
%     [H,Vehicle] = SegEval_Accel(Mission,i,H,Vehicle);
% end
% 
% 
% 
% 
% % query vehicle aero-propulsive performance
% for j = 2:1:np
%     %[t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13] = SolveFltCon(H.Mass(j),FullThrottle,H.rho(j),H.TAS(j),H.FPA(j),H.dFPAdt(j),H.dVdt(j),'perf',Vehicle);
%     [H.Ps(j),H.FPA(j),~,~,H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,1),X0] = SolveFltCon(H.Mass(j),FullThrottle,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),EvalMode,Vehicle,X0);
% 
%     % calculate vertical speed
%     H.dhdt(j) = H.TAS(j).*sin(H.FPA(j));
% 
% 
% 
%     % update mass for next timestep
%     if j<np
%         % calculate time increments
%         H.dTime(j) = (H.Alt(j+1)-H.Alt(j))./H.dhdt(j);
% 
%         % calculate dV/dt
%         H.dVdt(j) = (H.TAS(j+1)-H.TAS(j))./H.dTime(j);
% 
%         H.dMass(j) = sum(H.dMdt(j,:)) *  H.dTime(j);
%         H.Mass(j+1) = H.Mass(j) + H.dMass(j);
% 
%         % integrate state derivatives
%         H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * H.dTime(j);       
%         % log current state
%         Vehicle.States =  H.States(j+1,:); 
% 
%         % energy
%         H.Energy(j+1,:) = H.Energy(j,:) + H.dEdt(j,:) * H.dTime(j);        
%     end
% 
%     if(j>1&&j<np)
%         % calculate dFPA/dt
%         H.dFPAdt(j) = (H.FPA(j)-H.FPA(j-1))/H.dTime(j);
%     end
% 
% end
% 
% 
% 
% 
% % Find average energy rate
% % dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));
% 
% % Compute energy decrements per time interval
% H.dEnergy(1:np-1,:) = H.dEdt(1:np-1,:).*H.dTime(1:np-1);
% 
% % Compute energy remaining
% % H.Energy(2:np,:) = H.Energy(1,:) + cumsum(H.dEnergy(1:np-1,:));
% 
% % Compute energy "mass"
% H.EMass = H.Energy./Vehicle.SOTA.SpecificEnergy_MJkg;
% 
% % Compute ground speed
% H.GS = H.TAS.*cos(H.FPA);
% 
% % Compute distances traversed per time interval
% H.dDist = [0.5 * (H.GS(1:end-1) + H.GS(2:end)).*H.dTime(1:end-1);0];
% 
% % Compute cumulative distance after each time interval
% H.Dist(2:np) = H.Dist(1)+ cumsum(H.dDist(1:np-1));
% 
% % Compute cumulative time after each time interval
% H.Time(2:np) = H.Time(1) + cumsum(H.dTime(1:np-1));
% 
% H.FLAG = FLAG;
% 
% 
% fprintf(' %0.0f. %s->',i,S.Name)


% query vehicle aero-propulsive performance
for j = 1:1:np
    %[t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13] = SolveFltCon(H.Mass(j),FullThrottle,H.rho(j),H.TAS(j),H.FPA(j),H.dFPAdt(j),H.dVdt(j),'perf',Vehicle);
    [H.Ps(j),H.FPA(j),~,~,H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,:),X0] = SolveFltCon(H.Mass(j),FullThrottle,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),EvalMode,Vehicle,SolverType,X0);

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




% % Find average energy rate
% dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));

% Compute energy decrements per time interval
H.dEnergy(1:np-1,:) = H.dEdt(1:np-1,:).*H.dTime(1:np-1);

% Compute energy remaining
H.Energy(2:np,:) = H.Energy(1,:) + cumsum(H.dEnergy(1:np-1,:));

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

H.FLAG = FLAG;


fprintf(' %0.0f. %s->',i,S.Name)

