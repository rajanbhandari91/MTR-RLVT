function [Vehicle] = EMPAERO_ObtainOptCond(Vehicle)
%% opearating condition 
mu = 18.13*1e-6;                                                      % at 20 deg C
rhoSL = 1.225;
M = Vehicle.DesignPoint.DesignMach;
Vehicle.Aero.OptCondition.Beta = sqrt(1-M^2);                                   % Prandtl-Galauert compressibility orrection factor [ B = sqrt(1-M^2), M: Mach]
Re_pul = rhoSL*(Vehicle.Operations.VC*0.514444)/mu;                         % Reynolds number per unit length
Vehicle.Aero.OptCondition.Re_pul =  Re_pul;                                       

end 