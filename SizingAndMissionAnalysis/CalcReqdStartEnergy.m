function [ReqdStartEnergy_MJ, EnergyUsed_MJ, RequiredReserveEnergy_MJ] = CalcReqdStartEnergy(Mission)


History = Mission.History;

% % calculate the required reserve energy based on start energy and
% % the required reserve energy fraction

for i = 1:Mission.PropTypes

    EnergyUsed_MJ(1,i) = History.Energy(1,i)-History.Energy(end,i);

    % if reserve energy is specified as %-age of start energy
    if strcmpi(Mission.ResEnergyType{i},'start')
        FinalToUsedRatio = (Mission.ResEnergyPerc(i)/100)/(1-Mission.ResEnergyPerc(i)/100);
        RequiredReserveEnergy_MJ(i) = FinalToUsedRatio* (History.Energy(1,i)-History.Energy(end,i));
    end

    % if reserve energy is specified as %-age of consumed energy
    if strcmpi(Mission.ResEnergyType{i},'used')
        RequiredReserveEnergy_MJ(i) = (Mission.ResEnergyPerc(i)/100) * (History.Energy(1,i)-History.Energy(end,i));
    end

end

% find required start energy, accounting for energy used + required
% reserves, but not (yet) considering minimum energy mass requirement
ReqdStartEnergy_MJ = EnergyUsed_MJ + RequiredReserveEnergy_MJ;

end