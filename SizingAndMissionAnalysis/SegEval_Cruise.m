function [H, Vehicle] = SegEval_Cruise(Mission,i,History,Vehicle)

% Profile options:
% 1: specified EAS or Mach
% 2: best range cruise
% 3: best endurance cruise

% constants
g = 9.81;
SpecifiedPower = 0;

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

[H.EAS(1),H.TAS(1),H.Mach(1),~,~,H.rho(1)] = SetFltCon(H.Alt(1),dISA,'TAS',H.TAS(1));


% Initialize all masses to segment start mass
H.Mass = H.Mass(1) * ones(np,1);

% If no speed information is given, initialize all TAS to segment start TAS
if isempty(S.EAS)&&isempty(S.TAS)&&isempty(S.Mach)
    H.TAS = H.TAS(1) * ones(np,1);
    [H.EAS,H.TAS,H.Mach,~,~,H.rho] = SetFltCon(H.Alt,dISA,'TAS',H.TAS);
    SpecifiedVelocity = 0;
end


% if end altitude information is given, create a linear variation of
% altitudes from intiial segment altitude to given final altitude
SpecifiedEndAlt = 0;
if ~isempty(S.EndAlt)
    H.Alt = linspace(H.Alt(1),S.EndAlt,np)';
    SpecifiedEndAlt = 1;
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

% initial guess for warm starting
X0 = [];

SolverType = 1;



% this is segment 'i'. It is bounded by nodes 'i' and 'i+1'.

% check if segment distance itself is given. In that case, set the
% cruise distance accordingly. There is no need for an outer
% convergence loop, so set outerconvergence flag to 1
if Mission.Dists(i,i+1)~=0
    CruiseDist = Mission.Dists(i,i+1);
end

% check if cumulative distance at the end of climb + cruise is given. In that
% case, set cruise distance to given cumulative distance at end
% of segment less cumulative distance thus far. No need for outer
% convergence loop, so set outerconvergence flag to 1
if Mission.Dists(i-1,i+1)~=0
    PrevSegDist = History(History.SegNum==i-1,:).Dist(end)-History(History.SegNum==i-1,:).Dist(1);
    CruiseDist = Mission.Dists(i-1,i+1) - PrevSegDist;
end

% check if cumulative distance at the end of climb + cruise + descent
% is given. In that case, compute the cruise + descent distance
if Mission.Dists (i-1,i+2)~=0
    % capture previous segment
    PrevSegDist = History(History.SegNum==i-1,:).Dist(end)-History(History.SegNum==i-1,:).Dist(1);
    CruisePlusDescentDist = Mission.Dists(i-1,i+2) - PrevSegDist;
    
    
    % Case 1: the rate of descent is specified
    Snext = Mission.Segments(i+1);
    if ~isempty(Snext.ROD)
        
        % create a linspace over altitude range to descend through
        hvec = linspace(Snext.EndAlt, S.EndAlt,10); hvec = hvec';
        
        % find what true airspeed occurs over descent range
        [~,tas,~,~,~,~] = SetFltCon(hvec,dISA,'EAS',Snext.EAS);
        
        % find flightpath angles, and compute 2-point averages
        FPAvec = asind(Snext.ROD./tas);
        
    end
    
    % Case 2: the rate of descent is not specified, but the power
    % setting is specified
    if isempty(Snext.ROD)
        
        % create a linspace over altitude range to descend through
        hvec = linspace(Snext.EndAlt, S.EndAlt,3); 
        
        % find what true airspeed occurs over descent range
        [~,tas,~,~,~,rho] = SetFltCon(hvec,dISA,'EAS',Snext.EAS);
        
        % set descent segment power setting
        ThrottleSetting = Snext.PowerSetting;
        
        fpa = zeros(size(hvec));
        for j = 1:length(hvec)
            [~,fpa(j),~,~,~,~,~,~,~,~,~,~,~] = SolveFltCon(H.Mass(1),ThrottleSetting,rho(j),tas(j),0,0,0,'perf',Vehicle,SolverType,[]);
        end
        
        % rad--> deg unit change, transpose, and sign change
        FPAvec = -fpa*180/pi;
        
    end
    
    FPAmean = (FPAvec(1:end-1) + FPAvec(2:end))/2;
    
    % find distances traversed over altitude linspace
    dx = diff(hvec)./tand(FPAmean);
    
    % sum to find descent distance
    DescentDist = sum(dx);
    
    
    % compute cruise distance
    CruiseDist = CruisePlusDescentDist - DescentDist;    
end


% Set up linearly spaced distance grid points
H.Dist = linspace(H.Dist(1),H.Dist(1)+ CruiseDist,np)';


if SpecifiedVelocity==0
    % get optimal velocity, given gross weight and energy height
    % leave first row unchanged
    H.TAS(2:end) = Mission.Optim.Cruise.TAS_fn_GW(H.Mass(2:end));
    [H.EAS,~,H.Mach,~,~,H.rho] = SetFltCon(H.Alt,dISA,'TAS',H.TAS);
end

if SpecifiedVelocity==1
    n = 2:length(H.Alt);
    [H.EAS(2:end),H.TAS(2:end),H.Mach(2:end),~,~,H.rho(2:end)] = SetFltCon(H.Alt(n),dISA,VelType,SegVel(n));
end

% take care of acceleration/deceleration
[H,Vehicle] = SegEval_Accel(Mission,i,H,Vehicle);

% if end altitude is not given, then find optimal altitude based on
% gross weight
if SpecifiedEndAlt == 0
    H.Alt(2:end) = Mission.Optim.Cruise.Alt_fn_GW(H.Mass(2:end));
end

% compute distance increments
H.dDist(1:np-1) = diff(H.Dist);

% calculate the flight path angle and ground speed
H.FPA(1:end-1) = atan(diff(H.Alt)./diff(H.Dist));

% compute average ground speed between time points
% Compute ground speed
H.GS = H.TAS.*cos(H.FPA);
GSAvg = 0.5*(H.GS(1:np-1)+H.GS(2:np));

% compute time increments
H.dTime(1:np-1) = H.dDist(1:np-1)./GSAvg;

% compute acceleration dVdt
H.dVdt(1:np-1) = diff(H.TAS)./H.dTime(1:np-1);

% compute dhdt
H.dhdt(1:np-1) = diff(H.Alt)./H.dTime(1:np-1);

% calculate dFPA/dt
H.dFPAdt(1:np-1) = diff(H.FPA)./H.dTime(1:np-1);

% query vehicle aero-propulsive performance

for j = 2:1:np
    if SpecifiedPower == 0
        [H.Ps(j),H.FPA(j),H.TAS(j),~,H.THROT(j,:),H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,FLAG(j,1),X0]= SolveFltCon(H.Mass(j),ThrottleRatio,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'power',Vehicle, SolverType, X0);
    end
    
    % update mass for next timestep
    if j<np
        H.dMass(j) = sum(H.dMdt(j,:)) *  H.dTime(j);
        H.Mass(j+1) = H.Mass(j) + H.dMass(j);

        % integrate state derivatives
        H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * H.dTime(j);
        % log current state
        Vehicle.States =  H.States(j+1,:);

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

% re-compute dDist distance increments
H.dDist(1:np-1) = diff(H.Dist);
H.dDist(np,1) = 0;

% Compute cumulative time after each time interval
H.Time(2:np) = H.Time(1) + cumsum(H.dTime(1:np-1));

% Compute energy height
H.EnHt = H.Alt + H.TAS.^2/(2*g);

% re-compute dt time increments
H.dTime(1:np-1) = diff(H.Time);
H.dTime(np,1) = 0;


H.FLAG = FLAG;

fprintf(' %0.0f. %s->',i,S.Name)



end




% commented out:
% while outerconvergence==0 && outerk<5
% outerconvergence = 0;
% outerk = 1;
%     % set convergence flag and convergence counter
%     converged = 0;
%     k = 1;

    

    
    % begin segment convergence loop
%     while converged == 0 && k <= maxiter
        

%         % average mass change rate
%         dMdtavg = 0.5*(sum(H.dMdt(1:np-1,:),2) +sum(H.dMdt(2:np,:),2));
%         
%         
%         % calculate mass decrements
%         H.dMass(1:np-1) = H.dTime(1:np-1).*dMdtavg;
        
        % update masses
%         OldMasses = H.Mass;     % store old masses for convergence testing
%         H.Mass(2:np) = H.Mass(1) + cumsum(H.dMass(1:np-1));
        
        %     check whether masses at each point on the segment have converged to
        %     specified tolerance
%         test = abs(H.Mass-OldMasses)<MassConvergenceThreshold
%         if(all(abs(H.Mass-OldMasses)<MassConvergenceThreshold))
%             converged = 1;
%         else
%             k=k+1;
%         end
%         H
        
%     end
    
    
%     H = [HP;H];


    %
    %     % Find average energy rate
    %     dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));
    %
    %     % Compute energy decrements per time interval
    %     H.dEnergy(1:np-1,:) = dEdtavg.*repmat(H.dTime(1:np-1),[1,nprop]);
    %
    %     % Compute energy remaining
    %     H.Energy(2:np,:) = H.Energy(1,:) + cumsum(H.dEnergy(1:np-1,:));
    %
    %     % Compute energy "mass"
    %     H.EMass = H.Energy./Vehicle.SOTA.EnergyDensity_MJkg;
    

%     iter = 1;
%     
%     History0 = [History;H];
%     H0 = H;
%     SeekDist = H.Dist(end);
    
%     while outerconvergence == 0 && iter<5
%         forcedmaxiter = 2;
%         
%         % Save existing tables
%         %History0 = History;
%         H0 = H;
%         
% 
%         
%         
%         [H2] = SegEval_Descent(Mission,i+1,H0,Vehicle,forcedmaxiter);
%         
%         %         H2
%         
%         
%         TestDist = H2.Dist(end) - Mission.DistTable(1,i-1);
%         
%         %         TestDist
%         %         TargetDist
%         
%         DistErr = (TestDist-TargetDist);
%         
%         %History = [History0;H0];
%         fprintf('\n C-D iteration %0.0f, dist err %0.2f',iter,DistErr);        
%         
%         SeekDist = SeekDist - DistErr;
%         
%         if abs(DistErr)<300
%             outerconvergence = 1;
%         else
%             IndexVar = [1:1:height(H0)]';
%             
%             
%             CatchPoint = interp1(H0.Dist,IndexVar,SeekDist,'linear','extrap');
%             if CatchPoint > IndexVar(end)
%                 CatchPoint = IndexVar(end);
%             end
%             
%             P1 = floor(CatchPoint);
%             P2 = ceil(CatchPoint);
%             
% %             if P1==13
% %                 uu=1;
% %             end
%             
%             f = CatchPoint-P1;
%             
%             %             CatchPoint
%             %             f
%             %             H0
%             if CatchPoint<height(H0)
%                 %                 P1
%                 %                 P2
%                 if P1>0&&P2>0
%                     [OutRow] = InterpolateBetweenTableRows(H0(P1,:),H0(P2,:),f);
%                     H = [H0(1:P1,:);OutRow];
%                 end
%             else
% 
%                 if P1>0
%                     H = [H0(1:P1,:)];
%                 end
%             end
%             
%             if P1==0||P2==0
%                 clear H
%                 H = [];
%                 outerconvergence = 1;
%             end
%             
%                        
%             
%         end
%         
%         iter = iter + 1;
%  
%     end
%     
% 
%     
%     if~isempty(H)