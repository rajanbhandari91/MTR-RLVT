function W_fur_lb  = EvalFurWeight_lb(W_dg,NPAX,WTO)
    % Reviewed 11/19/21 2140 hrs

    %% Nomenclature
    % W_dg = Flight Design Gross Weight (We usally use WTO) [lb]
    % NPAX = Number of Passengers, including crew
    % WTO = Takeoff Gross Weight [lb]



    W_fur(1) = 0.0582*W_dg - 65;                                            % 1. Raymer, pg 461

    W_fur(2) = 0.412 * (NPAX^1.145) * (WTO^0.489);                          % 2. Cessna method, Roskam rpg 1555

    W_fur_lb = sum(W_fur)/2 ;
end