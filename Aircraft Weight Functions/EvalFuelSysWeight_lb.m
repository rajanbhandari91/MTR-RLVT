function FS_lb = EvalFuelSysWeight_lb(Qtot,Qint,Ntank,Neng)
    % Reviewed 11/19/21 2102 hrs

    %% Nomenclature
    % Qtot = Total volume of fuel [usg]
    % Qint = Total volume of fuel in integral tanks [USG]
    % Ntank = Number of Fuel Tanks
    % Neng = Number of Engines
    
    FS_lb = 2.49*(Qtot^0.726)*(1/(1 + Qtot/Qint))^0.363...                   % 1. Raymer, pg 461 (also Nicolai and USAF methods)
        *Ntank^0.242*Neng^0.157;
end