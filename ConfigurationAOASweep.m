
KTAS = 200;
Alt_ft = 0;
mass =  Vehicle.MassProp.MTOM_kg;

AOATestRange = [-10:2:30];

% FPA = 0;
% vdot = 0;
fpadot = 0;
EvalType = 'trim';
dt = [1];
[~,~,~,rho] = atmosisa(Alt_ft/3.28);
Vehicle.Transition = 0;
Vehicle.TransitionAOA = 0*pi/180;


V = KTAS * 0.51444;
dhdt_in = 0;
vdot_in = 0;

[Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,ADDL,fval,exitflag,output] = SolveFltCon(mass,dt,rho,V,dhdt_in,fpadot,vdot_in,EvalType,Vehicle);



AOAtrim = AOA*180/pi;




AOATestVec = sort([AOATestRange, AOAtrim],'ascend');




for i = 1:length(AOATestVec)

    AOA = AOATestVec(i)*pi/180;
[Fxap,Fzap,M_out,dmdt,dEdt,ADDL_high,ppdata] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,FPA,Vehicle);

h(i) = load('LoadCase.mat');

end



% extract pitching moment table
PitchingMoments = table();

for i = 1:length(AOATestVec)

    PitchingMoments(i,:) = h(i).LoadTable(5,:);

end

figure
hold on
xlabel('AOA (deg)')
ylabel('Pitching moment (Nm)')

plot(PitchingMoments.AOA, PitchingMoments.Fus,'m.-','displayname','Fuselage')
plot(PitchingMoments.AOA, PitchingMoments.LWing,'bx','displayname','L-Wing')
plot(PitchingMoments.AOA, PitchingMoments.RWing,'b--','displayname','R-Wing')
plot(PitchingMoments.AOA, PitchingMoments.LCan,'rx','displayname','L-Can')
plot(PitchingMoments.AOA, PitchingMoments.RCan,'r--','displayname','R-Can')
plot(PitchingMoments.AOA, PitchingMoments.VS,'g','displayname','VS')
plot(PitchingMoments.AOA, PitchingMoments.Eng1,'mx','displayname','Eng_1')
plot(PitchingMoments.AOA, PitchingMoments.Eng2,'m--','displayname','Eng_2')
plot(PitchingMoments.AOA, PitchingMoments.TOT,'k','linewidth',2,'displayname','TOTAL')

legend('show')
