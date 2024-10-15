
clc
% clear
% close all 
% clearvars -except Vehicle
load Vehicle

Vehicle.Aero.LoadTableCounter = 1;

AOA_deg = [-10:2:20];

TestAxis = 'lon';
Val = 0;
ulon = Val; ulat = 0; udir = 0;


dt = [0,0];
rho = 1.225;
V = 150*0.51444;
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
LD(i) = ADDL(i,12);
CL(i) = ADDL(i,13);
% CD(i) = ADDL(i,17);
% Cm(i) = ADDL(i,19);

% h(i) = load('LoadCase.mat');
end 


%% Effect of control input 
% ulon = 0.5; ulat = 0; udir = 0;
% AOA_deg = 0;
% for i = 1:length(AOA_deg)
% 
% AOA(i,1) = AOA_deg(i)*pi/180;
% rho = 1.225;
% 
% for j = 1: length(udir)
% dcon =[ulon, ulat, udir];
% [Fxap,Fzap,Myap,dmdt,dEdt,ADDL(j,:)] = AeroPropPerf(AOA(i),dcon,dt,rho,V,Mode,Vehicle);
% CL(j) = ADDL(j,8);
% CD(j) = ADDL(j,18);
% Cm(j) = ADDL(j,19);
% end 
% end

% figure;
plot(AOA_deg,LD)
% hold on
% plot(AOA,CL)

subplot(2,3,1)
plot(AOA_deg, CL, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('CL')
grid on
hold on
set(gca, 'FontWeight', 'bold')

subplot(2,3,2)
plot(AOA_deg, CD, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('CD')
grid on
hold on
set(gca, 'FontWeight', 'bold')

subplot(2,3,3)
plot(AOA_deg, CL./CD, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('CL/CD')
grid on
hold on
set(gca, 'FontWeight', 'bold')

subplot(2,3,4)
plot(AOA_deg, sign (CL) .* abs(CL).^(3/2)./CD, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('(CL32/CD')
grid on
hold on
set(gca, 'FontWeight', 'bold')

subplot(2,3,5)
plot(AOA_deg, Cm, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('Cm')
grid on
hold on
set(gca, 'FontWeight', 'bold')

fig = gcf;
fig.Position = [2526 293 560 420];

% sgtitle('aerodynamic coefficients', 'FontWeight', 'bold');
% 
% % legned
% subplot(2,3,1)
% legend({'LWing ulat:0', 'LWing ulat:1'})


% subplot(2,3,1)
% legend({'LHTail ulon:0', 'LHTail ulon:0.2', 'LHTail ulon: -0.2'})
% 
% fig = gcf;
% fig.Position = [295 171 1029 540];
% location = 'Results and Plots\';
% fullfilename = [location filename];
% exportgraphics(fig, fullfilename ,'Resolution',300)


% AOA_deg_q = [0 0.2];
% CL_q = interp1(AOA_deg,CL,AOA_deg_q);
% Cm_q = interp1(AOA_deg,Cm,AOA_deg_q);

% SM = - diff(Cm_q)/diff(CL_q) * 100;
% 
% table(SM)