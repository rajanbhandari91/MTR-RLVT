clc
% clear
% close all
% 
% 
% %%%% Script to run QPROP over a sweep, then create GI's of nondimensional
% %%%% thrust and torque coefficients versus advance ratio
% 
% 
% load DesignedProp
% % DesignedPropFileName = ['Output\OP_mil_DesignedProp.out'];
% % copyfile(DesignedPropFileName,'Output\OP_mil.out');        
% foil = 'clarky';                                           

PropDefSweep = PropDef;
beta75 = interp1(QMIL.r_R,QMIL.beta, 0.75,'linear')


fprintf('\n\nDefine Lower Limit, Upper Limit and step-size respectively\n\n')
PropDefSweep.rpm1 = 1800;
PropDefSweep.rpm2 = 2300;
PropDefSweep.rpmDel = 100;
PropDefSweep.Vel1 = 0;
PropDefSweep.Vel2 = 100;
PropDefSweep.VelDel = 10;
PropDefSweep.dBeta1 = -26;
PropDefSweep.dBeta2 = 24;
PropDefSweep.dBetaDel = 2;
PropDefSweep.Alt = 0;
PropDefSweep.Sig = 1;
PropDefSweep.T = 0.1;

% Run the sweep
% Do not run QMIL, run QPROP
casename = 'DesPropSweep';
RunQMIL = 0;
RunQPROP = 1;
Analysis = 'RPM-Vel-Beta';

[~,QPROPSweep1] = QMIL_PROP(foil,PropDefSweep,RunQMIL,RunQPROP,Analysis,casename);
QPROPSweep1.rho = (1.225 * PropDefSweep.Sig)*ones(height(QPROPSweep1),1);


PropDefSweep.Alt = 0000/3.28;
[~,~,~,PropDefSweep.rho] = atmosisa(PropDefSweep.Alt);
PropDefSweep.Sig = PropDefSweep.rho / 1.225;


[~,QPROPSweep2] = QMIL_PROP(foil,PropDefSweep,RunQMIL,RunQPROP,Analysis,casename);
QPROPSweep2.rho = (1.225 * PropDefSweep.Sig)*ones(height(QPROPSweep2),1);



QPROPSweep = [QPROPSweep1; QPROPSweep2];







% remove rows with NANs in them.
nanT=find(isnan(QPROPSweep.T_N));
nanQ=find(isnan(QPROPSweep.Q_Nm));
nanP=find(isnan(QPROPSweep.Pshaft_W));


nanrows = unique([nanT;nanQ;nanP]);
cleanrows = setdiff(1:1:height(QPROPSweep),nanrows)';
QSC = QPROPSweep(cleanrows,:);

% compute advance ratio, thrust coefficient, and torque coefficient
QSC.J = QSC.V./((QSC.rpm/60)*2*PropDefSweep.r_t);
QSC.CT = QSC.T_N./(QPROPSweep.rho .* (QSC.rpm/60).^2. * (2*PropDefSweep.r_t)^4);
QSC.CQ = QSC.Q_Nm./(QPROPSweep.rho .* (QSC.rpm/60).^2. * (2*PropDefSweep.r_t)^5).*sign(QSC.CT);


% unique blade pitch settings
betas = unique(QSC.Dbeta);
nbetas = length(betas);


% number of reference advance ratios for GI
numJ = 30;
Junique = unique(QSC.J);
Jvec = linspace(0,max(Junique),numJ)';


colorvec = 'kcmgbrkcmgbrkcmgbrkcmgbrkcmgbrkcmgbr';

figure
hold on
set(gca,'FontSize',18) 
subplot(1,3,1)
hold on
xlabel('Advance Ratio')
ylabel('Thrust Coefficient')
hold off

subplot(1,3,2)
hold on
xlabel('Advance Ratio')
ylabel('Torque Coefficient')
hold off

subplot(1,3,3)
hold on
xlabel('Advance Ratio')
ylabel('Efficiency')
hold off




for i = 1:nbetas
   % capture entries that have a particular value of Dbeta 
   Tbl = QSC(QSC.Dbeta == betas(i),:); 
   


   % create a table of J, CT, CQ with unique entries for J
   % if there are multiple entries in Tbl with the same J, take the average
   for j = 1:length(Junique)
      
       Tbl2 = Tbl(Tbl.J == Junique(j),:);
       
       NDTbl.J(j,i) = Junique(j);
   
       NDTbl.CT(j,i) = mean(Tbl2.CT);
       NDTbl.CQ(j,i) = mean(Tbl2.CQ);
       NDTbl.eff(j,i) = NDTbl.CT(j,i) * NDTbl.J(j,i)/(2*pi*NDTbl.CQ(j,i));
       
       
       % for any negative thrust, set propeller efficiency to zero
       if NDTbl.CT(j,i)<0
           NDTbl.eff(j,i)=0;
       end
       
   end
   
   % interpolate only at the chosen advance ratios
   GITbl.J(:,i) = Jvec;
   GITbl.beta(:,i) = betas(i)*ones(size(Jvec));
   GITbl.CT(:,i) = interp1(NDTbl.J(:,i),NDTbl.CT(:,i),Jvec,'linear');
   GITbl.CQ(:,i) = interp1(NDTbl.J(:,i),NDTbl.CQ(:,i),Jvec,'linear');
   GITbl.eff(:,i) = interp1(NDTbl.J(:,i),NDTbl.eff(:,i),Jvec,'linear');
   
   
   % plotting
   subplot(1,3,1)
   hold on
   scatter(NDTbl.J(:,i),NDTbl.CT(:,i),colorvec(i),'.'); 
   plot(GITbl.J(:,i),GITbl.CT(:,i),colorvec(i));
   hold off
   
   subplot(1,3,2)
   hold on
   scatter(NDTbl.J(:,i),NDTbl.CQ(:,i),colorvec(i),'.'); 
   plot(GITbl.J(:,i),GITbl.CQ(:,i),colorvec(i));
   hold off
   
   subplot(1,3,3)
   hold on
   scatter(NDTbl.J(:,i),NDTbl.eff(:,i),colorvec(i),'.'); 
   plot(GITbl.J(:,i),GITbl.eff(:,i),colorvec(i));
end
% tightfig

nCQ = 20;
CQRange = linspace(-0.09,0.09,nCQ);
CQRef = repmat(CQRange,[length(Jvec),1]);
JRef = repmat(Jvec,[1,nCQ]);

for i = 1:length(Jvec)
    
    BetaRef(i,:) = interp1(GITbl.CQ(i,:),GITbl.beta(i,:),CQRef(i,:),'linear','extrap');
    
    
end




%%% Create gridded interpolants
Prop.Betas = betas'+beta75;
Prop.GI_CT_J_beta = griddedInterpolant(GITbl.J,GITbl.beta+beta75,GITbl.CT,'linear','nearest');
Prop.GI_CQ_J_beta = griddedInterpolant(GITbl.J,GITbl.beta+beta75,GITbl.CQ,'linear','nearest');
Prop.GI_eta_J_beta = griddedInterpolant(GITbl.J,GITbl.beta+beta75,GITbl.eff,'linear','nearest');
% Prop.GI_beta_J_CQ =  griddedInterpolant(JRef,CQRef,BetaRef,'linear','nearest');

