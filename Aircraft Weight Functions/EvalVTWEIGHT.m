function [W_VT_lb] = EvalVTWEIGHT(nVT, Sv, bv, lam25v, TOCv, TRv, Wing_5_Hv, trv, Av,q,Wdg,nult,WTO)
        % Reviewed 11/19/21 2042 hrs

        %% Nomenclature
        % nVT = number of vertical tails
        % Sv = Planform Area of one Vtail
        % bv = Vertical Tail Span
        % lam25v = Quater-Chord Sweep for Vertical Tail
        % TOCv = Vetical Tail Thickness-to-chord
        % Wing_5_Hv = Mounting Position of Htail onto Vtail
        % trv = Root chord thickness
        % Av = Vertical Tail Aspect Ratio
        % q = dynamic pressure at cruise
        % Wdg = Flight design Gross Weight (We usually use WTO)
        % nult = Ultimate Load factor
        % WTO = Takeoff Gross Weight


        %%% VERTICAL TAIL WEIGHT
        % 1. Cessna method - Roskam rpg 1519 (SOMETHING LIKELY WRONG WITH THIS EQUATION)
%         W_VT(1) = (1.68 * WTO^0.567 * Sv^1.249 *...
%             Av^0.482)/(15.6 * trv^0.747 * (cosd(lam25v))^0.882);

        % 2. USAF method - Roskam rpg 1520 (also reported as Nicolai method (Gudmundsson rpg 149)
        W_VT(1) = nVT * 98.5 * ( (WTO*nult/1e5)^0.87...
            * (Sv/100)^1.2  * 0.289 * (bv/trv)^0.5)^0.458;

        % 3. Raymer method, pg 460
        W_VT(2) = nVT * 0.073 * (1 + 0.2 * Wing_5_Hv) * ...
            (nult * Wdg)^0.376* q^0.122 * Sv^0.873 *...
            (100*TOCv/cosd(lam25v))^(-0.49) *...
            (Av/(cosd(lam25v))^2)^0.357 * TRv^0.039;

        W_VT_lb = mean(W_VT);

    end