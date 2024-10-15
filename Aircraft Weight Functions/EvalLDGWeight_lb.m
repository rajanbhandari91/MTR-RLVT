function [W_LG_lb, W_MLDG_each_lb, W_NLDG_each_lb] = EvalLDGWeight_lb(nult,Wldg,Lm,Ln,nMLDG,nNLDG)
    % Reviewed 11/19/21 2053 hrs
    % Raymer method, pg 460
    %% Nomenclature
    % nult = ultimate load factor
    % Wldg = Weight when landing (We usually use WTO)
    % Lm = Length of main gear
    % Ln Length of nose gear
    % nMLDG = Number of Main Landing Gear
    % nNLDG = Number of Nose Landing Gear

    if nMLDG > 0
        W_MLDG_lb = 0.095 * ((nult * Wldg)^0.768)*(Lm/12)^0.409;
        W_MLDG_each_lb = W_MLDG_lb/nMLDG;
    else
        W_MLDG_lb = 0;
    end

    if nNLDG > 0
        W_NLDG_lb = 0.125*((nult*Wldg)^0.566)*(Ln/12)^0.845;
        W_NLDG_each_lb = W_NLDG_lb/nNLDG;
    else
        W_NLDG_lb = 0;
        W_NLDG_each_lb = 0;
    end


    W_LG_lb = W_MLDG_lb + W_NLDG_lb;                   
                          
end