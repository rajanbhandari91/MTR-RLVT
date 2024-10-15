function [Vehicle] = MassPropCalc(WTO_kg,EnergyMass_kg,Vehicle,Mission)

% conversion factors
conv_kg_to_lb = 1/0.4536;
conv_lb_to_kg = 1/conv_kg_to_lb;

% convert gross weight from kg to lb
WTO_lb = WTO_kg * conv_kg_to_lb;          

% convert energy and payload masses from kg to lb for weight estimation code
EnergyMass_lb = EnergyMass_kg * conv_kg_to_lb;
Payload_lb = Mission.Payload_kg * conv_kg_to_lb;

iter = 0;
wtconvergence = 0;

    fprintf('Vehicle update: iter - ')
while wtconvergence == 0 && iter < 10
    iter = iter + 1;
    
    % NOTE: Calculations within "VehicleWeightBuildup" use English units
    [WeightBuildup] = VehicleWeightBuildup(Vehicle,Payload_lb,EnergyMass_lb,WTO_lb);
    WTO_lbprev = WTO_lb;
    WTO_lb = WeightBuildup.Weights_lb(end);
    
    Vehicle.MassProp.TOGM_kg = WTO_lb * conv_lb_to_kg;
     [Vehicle] = VehicleSystemUpdate(Vehicle);

    dWTO = WTO_lb - WTO_lbprev;
    
    fprintf('%0.0f / %0.3f lb,...',iter,dWTO);
    
    if abs(dWTO)<0.5
        wtconvergence = 1;
        fprintf('--> converged!\n');
    end
%     WeightBuildup
        
    % convert gross weight back to kg
    WTO_kg = WTO_lb * conv_lb_to_kg;
    %fprintf('Mass iter %0.2f \n',WTO_kg);
end

% capture empty weight
p=WeightBuildup(strcmpi(WeightBuildup{:,1},'=== EMPTY WEIGHT   ==='),:);
Vehicle.MassProp.OEM_kg = p.Weights_kg;

p=WeightBuildup(strcmpi(WeightBuildup{:,1},'Batteries'),:);
Vehicle.MassProp.Batteries_kg = p.Weights_kg;

Vehicle.MassProp.OEMLessNCE_kg = Vehicle.MassProp.OEM_kg - Vehicle.MassProp.Batteries_kg;



Vehicle.MassProp.TOGM_kg = WTO_kg;
Vehicle.MassProp.Mass = WTO_kg;
Vehicle.MassProp.WeightBuildup = WeightBuildup;
Vehicle.MassProp.MassBuildupConvergence = wtconvergence;
Vehicle.Init.Mass_kg = WTO_kg;


end





