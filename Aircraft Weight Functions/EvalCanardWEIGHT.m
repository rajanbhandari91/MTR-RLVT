function [W_wing_lb] = EvalCanardWEIGHT(b_ft,c_m,TR)

% Reviewed: Nov 19, 2021

% CAUTION:
% if Wfw comes in = 0, then set = 1
% Wfw(Wfw==0) = 1;

% WTO: takeoff weight, lb
% nult: ultimate load factor
% AR: aspect ratio
% S_ft2: planform area, ft2
% lam25: quarter chord sweep, deg
% lam50: midchord sweep, deg
% TR: taper ratio
% TOCmax: max thickness to chord ratio
% VmaxSLkt: sea level max speed, kt
% b_ft: wingspan, ft
% RootThickness, ft
% q: cruise dynamic pressure, lb/ft2

        %%% WING WEIGHT

        % 1. Cessna method - Roskam r pg 1515 (for strut braced wings)
        % note: no functional dependence on WTO
        %W_Wing(1) = 0.002933 * (S^1.018) * (AR^2.473) * (nult^0.611);     % this is for strut braced wings
%         W_Wing(1) = 0.04674 * (WTO^0.397) * (S_ft2^0.360) * (nult^0.397) * (AR^1.712); % this is for cantilever wings
% 
%         % 2. USAF method - Roskam rpg 1516
%         % (also reported as Nicolai method, Gudmundsson rpg 148, but cosd(lam25)
%         % term there has a square (2) power.
%         W_Wing(2) = 96.948 * ( (WTO*nult/1e5)^0.65  * (AR/cosd(lam25))^0.57  * (S_ft2/100)^0.61  * ((1+TR)/(2*TOCmax))^0.36  * (1 + VmaxSLkt/500)^0.5   )^0.993;
% 
%         % 3. Torenbeek method (special notes Sec 5.2.2) - Roskam rpg 1516
%         W_Wing(3) = 0.00125 * WTO * (b_ft/cosd(lam50))^0.75 * (1+(6.3*cosd(lam50)/b_ft)^0.5) * nult^0.55 * (b_ft*S_ft2/(RootThickness * WTO * cosd(lam50)))^0.30;
% 
%         % 4. Raymer method (pg 460)
%         W_Wing(4) = 0.036 * S_ft2^0.758 * (Wfw)^0.0035 * (AR/(cosd(lam25))^2)^0.6 * q^0.006 * TR^0.04 * (100*TOCmax/cosd(lam25))^(-0.3) * (nult * Wdg)^0.49;
            b_in = b_ft * 12;
            c_in = c_m * 3.28 * 12 ;
            W0 = 1.875 * (b_in-3) + 2.205 * (c_in - 9) + 40 ;
            W_t = W0 - 28 * (1 - TR);
            
%             W_wing_lb = sum(W_Wing(2:end))/3;

        W_wing_lb = W_t ;

end

    