
clc
% clear
% close all 
clearvars -except Vehicle
load RAVEN_Vehicle

Vehicle.Aero.LoadTableCounter = 1;

AOA_deg = 2;

TestAxis = 'lon';
Val = 0;
ulon = Val; ulat = 0; udir = 0;


dt = [0,0];
rho = 1.225;
V = 23; % m/s
Mode = [];


Vehicle.Aero.DesSurf = [];

filename = [Vehicle.Aero.DesSurf, '_' TestAxis, '_', num2str(Val),'.png'];

%% Effect of AOA sweep
for i = 1:length(AOA_deg)

AOA(i,1) = AOA_deg(i)*pi/180;
rho = 1.225;

% for j = 1: length(udir)
dcon =[ulon, ulat, udir];
[Fxap,Fzap,Myap,dmdt,dEdt,ADDL(i,:)] = AeroPropPerf(AOA(i),dcon,dt,rho,V,Mode,Vehicle);
CL(i) = ADDL(i,8);
CD(i) = ADDL(i,18);
Cm(i) = ADDL(i,19);

h(i) = load('LoadCase.mat');
end 

for i = 1:length(AOA_deg)
    Fx(i,:) = h(i).LoadTable(1,:);
    Fy(i,:) = h(i).LoadTable(2,:);
    Fz(i,:) = h(i).LoadTable(3,:);
    Mx(i,:) = h(i).LoadTable(4,:);
    My(i,:) = h(i).LoadTable(5,:);
    Mz(i,:) = h(i).LoadTable(6,:);
end


figure

subplot(3,3,1)
hold on 
grid on 

xlabel('AOA (deg)')
ylabel('Fx(N)')
plot(Fx.AOA, Fx.Fus,'m.-','displayname','Fuselage')
% plot(Fx.AOA, Fx.LG,'bx','displayname','LG')
% plot(Fx.AOA, Fx.MotorAero,'b--','displayname','Motor Aero')
plot(Fx.AOA, Fx.LWing,'bx','displayname','L-Wing')
plot(Fx.AOA, Fx.RWing,'b--','displayname','R-Wing')

plot(Fx.AOA, Fx.LHT,'rx','displayname','L-HTail')
plot(Fx.AOA, Fx.RHT,'r--','displayname','R-HTail')

plot(Fx.AOA, Fx.VS,'g','displayname','VS')
plot(Fx.AOA, Fx.TOT,'k','linewidth',2,'displayname','TOTAL')

subplot(3,3,2)
hold on 
grid on 

xlabel('AOA (deg)')
ylabel('Fy(N)')
plot(Fy.AOA, Fy.Fus,'m.-','displayname','Fuselage')
% plot(Fy.AOA, Fy.LG,'bx','displayname','LG')
% plot(Fy.AOA, Fy.MotorAero,'b--','displayname','Motor Aero')
plot(Fy.AOA, Fy.LWing,'bx','displayname','L-Wing')
plot(Fy.AOA, Fy.RWing,'b--','displayname','R-Wing')

plot(Fy.AOA, Fy.LHT,'rx','displayname','L-HTail')
plot(Fy.AOA, Fy.RHT,'r--','displayname','R-HTail')

plot(Fy.AOA, Fy.VS,'g','displayname','VS')
plot(Fy.AOA, Fy.TOT,'k','linewidth',2,'displayname','TOTAL')


subplot(3,3,3)
hold on 
grid on 

xlabel('AOA (deg)')
ylabel('Fz(N)')
plot(Fz.AOA, Fz.Fus,'m.-','displayname','Fuselage')
% plot(Fz.AOA, Fz.LG,'bx','displayname','LG')
% plot(Fz.AOA, Fz.MotorAero,'b--','displayname','Motor Aero')
plot(Fz.AOA, Fz.LWing,'bx','displayname','L-Wing')
plot(Fz.AOA, Fz.RWing,'b--','displayname','R-Wing')

plot(Fz.AOA, Fz.LHT,'rx','displayname','L-HTail')
plot(Fz.AOA, Fz.RHT,'r--','displayname','R-HTail')

plot(Fz.AOA, Fz.VS,'g','displayname','VS')
plot(Fz.AOA, Fz.TOT,'k','linewidth',2,'displayname','TOTAL')


subplot(3,3,4)
hold on 
grid on 

xlabel('AOA (deg)')
ylabel('Mx(Nm)')
plot(Mx.AOA, Mx.Fus,'m.-','displayname','Fuselage')
% plot(Mx.AOA, Mx.LG,'bx','displayname','LG')
% plot(Mx.AOA, Mx.MotorAero,'b--','displayname','Motor Aero')
plot(Mx.AOA, Mx.LWing,'bx','displayname','L-Wing')
plot(Mx.AOA, Mx.RWing,'b--','displayname','R-Wing')

plot(Mx.AOA, Mx.LHT,'rx','displayname','L-HTail')
plot(Mx.AOA, Mx.RHT,'r--','displayname','R-HTail')

plot(Mx.AOA, Mx.VS,'g','displayname','VS')
plot(Mx.AOA, Mx.TOT,'k','linewidth',2,'displayname','TOTAL')

subplot(3,3,5)
hold on
grid on
xlabel('AOA (deg)')
ylabel('My(Nm)')

plot(My.AOA, My.Fus,'m.-','displayname','Fuselage')
% plot(My.AOA, My.LG,'bx','displayname','LG')
% plot(My.AOA, My.MotorAero,'b--','displayname','Motor Aero')
plot(My.AOA, My.LWing,'bx','displayname','L-Wing')
plot(My.AOA, My.RWing,'b--','displayname','R-Wing')

plot(My.AOA, My.LHT,'rx','displayname','L-HTail')
plot(My.AOA, My.RHT,'r--','displayname','R-HTail')

plot(My.AOA, My.VS,'g','displayname','VS')
plot(My.AOA, My.TOT,'k','linewidth',2,'displayname','TOTAL')

subplot(3,3,6)
hold on 
grid on 

xlabel('AOA (deg)')
ylabel('Mz(Nm)')
plot(Mz.AOA, Mz.Fus,'m.-','displayname','Fuselage')
% plot(Mz.AOA, Mz.LG,'bx','displayname','LG')
% plot(Mz.AOA, Mz.MotorAero,'b--','displayname','Motor Aero')
plot(Mz.AOA, Mz.LWing,'bx','displayname','L-Wing')
plot(Mz.AOA, Mz.RWing,'b--','displayname','R-Wing')

plot(Mz.AOA, Mz.LHT,'rx','displayname','L-HTail')
plot(Mz.AOA, Mz.RHT,'r--','displayname','R-HTail')

plot(Mz.AOA, Mz.VS,'g','displayname','VS')
plot(Mz.AOA, Mz.TOT,'k','linewidth',2,'displayname','TOTAL')
legend('show', 'Location','best', 'Position',[0.128245018845478 0.136764490665905 0.0755208318742613 0.203530627588122]);


sgtitle(['LoadTable plot: ulon = ' num2str(Val)]);

fig = gcf;
fig.Position = [-1919 41 1920 963];
location = 'Results and Plots\';
fullfilename = [location filename];
exportgraphics(fig, fullfilename ,'Resolution',300)