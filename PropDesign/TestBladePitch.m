clear all
close all
load GI_T_beta_J.mat

Thrust = linspace(0,6000,100)';
BP = zeros(length(Thrust),1);
for i = 1 : length(Thrust)
    LBP(i) = GI_T_beta_J(Thrust(i),0);
    % CBP(i) = GI_T_beta_J(Thrust(i),0);
end

figure(1)
% subplot(121)
scatter(LBP,Thrust)
xlabel('LP blade pitch')
ylabel('Thrust (N)')

% subplot(122)
% scatter(CBP,Thrust)
% xlabel('MP blade pitch')
% ylabel('Thrust (N)')

set(gcf,'Position',[253 204 1498 674],'Color','w')