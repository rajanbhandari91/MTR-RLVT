function W_HTail_lb = EvalHTailWeight_lb(WTO,nult,Sh,lh,bh, MaxRootThickness,Wdg,q,TOCh,Ah,lam25h,TRh)
 
% Reviewed: Nov 19, 2021

% WTO: takeoff weight, lb
% Wdg: design gross weight, lb
% nult: ultimate load factor
% Sh: tail planform area, ft2
% lh: tail moment arm, ft
% bh: tail span, ft
% TRh: taper ratio
% max root thickness, ft
% q: dynamic pressure, lb/ft2
% TOCh: thickness to chord ratio
% lam25h: quarter chord sweep
% Ah: aspect ratio
% TRh: taper ratio

    % 1. Cessna method - Roskam rpg 1519
    % W_HTAIL(1) = 3.184 * (WTO^0.887) * (Sh^0.101) * (Ah^0.138) / (57.5 * MaxRootThickness^0.223) ;
    % commented out, serious overprediction, predicts around 8.7 lb/ft2

    % 2. USAF method - Roskam rpg 1520 (also reported as Nicolai method (Gudmundsson rpg 149))
    W_HTAIL(2) = 127 * (((WTO*nult/1e5)^0.87) *...
        ((Sh/100)^1.2) * 0.289 * ((lh/10)^0.483) * ((bh/MaxRootThickness)^0.5)^0.458);
    % commented out, significant underprediction, predicts around 0.3 lb/ft2
    
    % 3. Raymer method pg 460
    W_HTAIL(3) = 0.016* ((nult*Wdg)^0.414)*...
        (q^0.168)*(Sh^0.896) * ((100*TOCh/cosd(lam25h))^(-0.12))...
        *((Ah/(cosd(lam25h))^2)^0.043)*(TRh^(-0.02));
    
    %  lb/ft2 check for testing: 
%     W_HTAIL/Sh
    
    W_HTail_lb = W_HTAIL(3);        % keeping only Raymer method
end