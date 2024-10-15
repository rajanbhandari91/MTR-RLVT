function W_el_lb = EvalElecWeight_lb(W_avio,W_FS,WTO)
    % Reviewed 11/19/2021 2115 hrs

    %% Nomenclature
    % W_avio  = Weight of avionics [lb]
    % FS_lb =  Fuel System Weight [lb]
    % WTO = Takeoff Gross Weight [lb]



    W_el(1) = 12.57*(W_FS + W_avio)^0.51;                                   % 1. Raymer, pg 461 (also Nicolai and USAF)

    W_el(1) = 0.0268 * WTO;                                                 % 2. Cessna method, Roskam rpg 1549

    W_el_lb = mean(W_el) ;

end