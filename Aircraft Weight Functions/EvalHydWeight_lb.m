function W_hyd_lb = EvalHydWeight_lb(W_dg,M)
    % Reviewed 11/19/2021 2129 hrs
    % 1. Raymer, General Aviation Weights, Pg. 576

    %% Nomenclature
    % W_dg = Design Flgiht Gross Weight (We usually assume WTO)
    % M = Design Mach Number


if M < 0.4
    K_h = 0.05;                                                            
elseif M >= 0.4 && M < 0.8
     K_h = 0.11;
elseif M >= 0.8
    K_h = 0.12;
else
    K_h = 0.013;
end

    W_hyd_lb = K_h*(W_dg^0.8)*(M^0.5);
end