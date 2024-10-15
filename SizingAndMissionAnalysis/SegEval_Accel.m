function [H, Vehicle] = SegEval_Accel(Mission,i,H,Vehicle)

SolverType = 1;

if H.TAS(2)>H.TAS(1)
    Throttles = 1;
else
    Throttles = 0;
end

for j = 1:2
    [Ps(j),~,~,acc(j),~,AOA(j),CTRL(j,:),dMdt(j,:),dEdt(j,:),ADDL(j,:),~,~,~]= SolveFltCon(H.Mass(j),Throttles,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'accel',Vehicle,SolverType);
end

dVdtavg = mean(acc) ;
if H.TAS(2) == H.TAS(1)
  dVdtavg = 0;
end

dt = (H.TAS(2) - H.TAS(1))/dVdtavg;
dt(isnan(dt)) = 0.0001 ;

for j = 1:2
    [H.Ps(j),~,~,H.dVdt(j),~,H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,H.FLAG(j,1),~]= SolveFltCon(H.Mass(j),Throttles,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'accel',Vehicle,SolverType);
        
        if j < 2
        % integrate state derivatives
        H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * dt;
        % log current state
        Vehicle.States =  H.States(j+1,:);
        end
end

[~,nprop] = size(H.dEdt);
H.dTime(1) = dt;
np = 2;



% Find average energy rate
% dEdtavg = 0.5*(H.dEdt(1:np-1,:) + H.dEdt(2:np,:));

% Compute energy decrements per time interval
H.dEnergy(1:np-1,:) = H.dEdt(1:np-1,:).*H.dTime(1:np-1);

% Compute energy remaining
H.Energy(2,:) = H.Energy(1,:) + H.dEdt(1,:) * H.dTime(1);

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

H.FLAG(1:2) = H.FLAG(1:2);




% dt = (H.TAS(2) - H.TAS(1))/dVdtavg;
% 
% Vavg = (H.TAS(1) + H.TAS(2))/2;
% 
% H.Dist(2) = H.Dist(1) + Vavg * dt;
% H.Time(2) = H.Time(1) + dt;
% 
% H.AOA(1) = mean(AOA);
% H.CTRL(1,:) = mean(CTRL);
% H.Ps(1) = mean(Ps);
% H.ADDL(1,:) = mean(ADDL);
% H.dMdt(1,:) = mean(dMdt);
% H.dEdt(1,:) = mean(dEdt);
% H.THROT(1,:) = Throttles;
% 
% H.dMass(1) = sum(H.dMdt(1,:)) *  dt;
% H.Mass(2) = H.Mass(1) + H.dMass(1);
% H.dTime(1) = dt;
% 
% 
% % for j = 1:2
% %     [~,~,~,acc(j),~,~,~,~,~,~,~,~,~,~]= SolveFltCon(H.Mass(j),Throttles,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'accel',Vehicle);
% % 
% % 
% % end
% % 
% % dVdtavg = mean(acc);
% % dt = (H.TAS(2) - H.TAS(1))/dVdtavg;
% % 
% for j = 1:2
%     [H.Ps(j),~,~,H.dVdt(j),~,H.AOA(j),H.CTRL(j,:),H.dMdt(j,:),H.dEdt(j,:),H.ADDL(j,:),H.ddtStates(j,:),~,H.FLAG(j,1),~]= SolveFltCon(H.Mass(j),Throttles,H.rho(j),H.TAS(j),H.dhdt(j),H.dFPAdt(j),H.dVdt(j),'accel',Vehicle);
% 
%         if j < 2
%         % integrate state derivatives
%         H.States(j+1,:) = H.States(j,:) + H.ddtStates(j,:) * dt;
%         % log current state
%         Vehicle.States =  H.States(j+1,:);
%         end
% end
% 
% [~,nprop] = size(H.dEdt);
% H.dTime(1) = dt;
% np = 2;
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
% H.Energy(2,:) = H.Energy(1,:) + H.dEdt(1,:) * H.dTime(1);
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
% H.FLAG(1:2) = H.FLAG(1:2);










