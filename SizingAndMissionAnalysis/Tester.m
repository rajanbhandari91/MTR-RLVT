
clearvars -except Vehicle

mass =  Vehicle.MassProp.MTOM_kg;

FPA = 00*pi/180;
vdot = 0;
fpadot = 0;
EvalType = 'trim';
dt = [1]
[~,~,~,rho] = atmosisa(0000/3.28);
Vehicle.Transition = 0;
Vehicle.TransitionAOA = 0*pi/180;
Vtest_kt = [50:2:60];


Vtest = Vtest_kt * 0.51444;

for i =  1:length(Vtest)

fprintf('\n Case %0.0f',i)
V(i,1) = Vtest(i);
dhdt_in = 00/(3.28*60);
vdot_in = 0;

% [a,b,c,d,e,f,g,h,aa,bb,cc,dd,ee]=SolveFltCon(mass,dt,rho,V(i),dhdt_in,fpadot,vdot_in,EvalType,Vehicle)

[Ps(i,1),FPA(i,1),V_out,vdot(i,1),dt_out(i,:),AOA(i,1),CTRL(i,:),dmdt,dEdt(i,:),ADDL(i,:),fval,exitflag(i,1),output] = SolveFltCon(mass,dt,rho,V(i),dhdt_in,fpadot,vdot_in,EvalType,Vehicle);
dhdt(i,1) = V_out * sin(FPA(i,1)) * 60 * 3.28;

f(i,1) = norm(fval);

V(i,1) = V_out;
fprintf('... complete')
end

% clc


Res= table();

AddlNames = {'CP1kW','CP2kW','CP3kW','CP4kW','LP1kW','LP2kW','TotalPwrKW','CL', 'LDRatio', 'CPRPM1', 'CPRPM2', 'CPRPM3', 'CPRPM4', 'LPRPM1', 'LPRPM2', 'TWH', 'TWV', 'FLAG'};
AddlTbl = array2table(ADDL);
AddlTbl.Properties.VariableNames = AddlNames;

Res.Vkt = V/0.51444;
Res.Vmph = Res.Vkt * 1.15078;
Res.AOA = AOA*180/pi;
Res.CTRL = CTRL;
Res.dt = dt_out;
Res.dhdt = dhdt;
Res.Ps = Ps*3.28*60;

Res.FPA = FPA*180/pi;
Res.vdot = vdot;
Res.flag = exitflag;
Res.f = f;


Res = [Res,AddlTbl]

% Res.TotalPower = Res.PCP + Res.PLR;
% 
% 
% Res.minE = Res.Ps./abs(sum(dEdt(i,:)));
% Res.MaxR = Res.Vkt./Res.TotPwrkW;
% 
% 
% plot(Res.Vkt,Res.TotPwrkW/mass,'green')
% hold on
% plot(Res.Vkt,Res.PCP/mass,'blue')
% plot(Res.Vkt,Res.PLR/mass,'red')

Res



% 
% % find best endurance flight condition
% [~,indBE] = min(Res.TotPwrkW)
% VBE = Res.Vkt(indBE);
% P_BE = Res.TotPwrkW(indBE);
% 
% % find best range flight condition
% [~,indBR] = max(Res.MaxR)
% VBR = Res.Vkt(indBR);
% P_BR = Res.TotPwrkW(indBR);
% 
% 
% fprintf('Best endurance at %0.0f KTAS, Power reqd = %0.1f kW\n',VBE, P_BE);
% fprintf('Best     range at %0.0f KTAS, Power reqd = %0.1f kW\n',VBR, P_BR);
% 
% 



% 
% 
% V = 87.45
% AOA = 2.0799 *pi/180
% dt_out = [0.8, 0]
% CTRL = [04.84*pi/180]
% 
% 
% 
% 
% [Fxap,Fzap,Myap,dmdt,dEdt,ADDL] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,0,Vehicle)







% AOA = 3;
% de = 0;
% dt= 1;
% rho = 1.225;
% V = 60
% 
% [Fxap,Fzap,Myap,dmdt,dEdt,ADDL] = AeroPropPerf(AOA,de,dt,rho,V,[],Vehicle)

% 
% 
% 
% % OEI Climb
% mass = 1230;
% rho = 1.225
% V = 84*0.5144;
% FPA = 0;
% vdot = 0;
% fpadot = 0;
% EvalType = 'perf'
% dt = [0.5 0]
% [Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,LDE,fval,exitflag,output] = SolveFltCon(mass,dt,rho,V,FPA,fpadot,vdot,EvalType,Vehicle)
% dhdt = V_out * sin(FPA) * 60 * 3.28
% LDE(1)

% 
% % cruise
% mass = 1750;
% rho = 1.225
% 
% FPA = 0;
% vdot = 0;
% fpadot = 0;
% EvalType = 'power'
% dt = [1 0]
% 
% 
% Vtest = [70:10:150]*0.5144;
% 
% for i = 1:1:length(Vtest)
% 
% V(i,1) = Vtest(i);
% [Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,LDE,fval,exitflag,output] = SolveFltCon(mass,dt,rho,V(i),FPA,fpadot,vdot,EvalType,Vehicle)
% dhdt = V_out * sin(FPA) * 60 * 3.28
% LDRatio(i,1) = LDE(1);
% end
% 
% Res= table();
% Res.Vkt = V/0.5144;
% Res.LDRatio = LDRatio;
% 


% % cruise
% mass = 1750;
% rho = 1.225
% 
% FPA = 00*pi/180;
% vdot = 0;
% fpadot = 0;
% EvalType = 'transition'
% dt = [1 0]
% 
% Vehicle.Transition = 1;
% Vehicle.TransitionAOA = 0*pi/180;
% Vtest = 30;
% 
% for i = 1:1:length(Vtest)
% 
% V(i,1) = Vtest(i);
% [Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,LDE,fval,exitflag,output] = SolveFltCon(mass,dt,rho,V(i),FPA,fpadot,vdot,EvalType,Vehicle)
% dhdt = V_out * sin(FPA) * 60 * 3.28
% LDRatio(i,1) = LDE(1);
% end
% 
% Res= table();
% Res.Vkt = V/0.5144;
% Res.LDRatio = LDRatio;
% 
% AOADeg = AOA*180/pi

% 
% mass = 1230;
% dt = 1;
% rho = 1.225
% V = 80*0.5144;
% FPA = 0;
% vdot = 0;
% fpadot = 0;
% EvalType = 'perf'
% 
% 
% [Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,LDE,fval,exitflag,output] = SolveFltCon(mass,dt,rho,V,FPA,fpadot,vdot,EvalType,Vehicle)
% 
% fpm = V_out * sin(FPA) * 60 * 3.28
