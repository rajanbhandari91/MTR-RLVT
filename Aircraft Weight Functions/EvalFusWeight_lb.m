function W_fus_lb = EvalFusWeight_lb(WTO,nult,Lfn,wf,hf,VC,Sfus,Wdg,lh,q)
    % Reviewed 11/20/21 1620 hrs

    %% Nomenclature
    % WTO = Takeoff Gross Weight [lb]
    % nult = Ultimate Load Factor
    % Lfn = Length of fuselage [ft]
    % wf = mean width of fuselage
    % hf = mean height of fuselage
    % VC = design cruise speed [kt]
    % Sfus = Fuselage Wetted Area [ft2]
    % Wdg = Flight Design Gross Weight (We usually assume WTO)
    % lh = Wing to Tail moment arm [ft]
    % q = cruise dyanamic pressure

    W_fus(1) = 200*((WTO*nult/1e5)^0.286*...                                % 2. USAF method - Roskam rpg 1524 (also reported as Nicolai method,
        (Lfn/10)^0.857*((wf + hf)/10)*...                                   % Gudmundsson rpg 149) - However, in place of VC, max level flight speed at
        (VC/100)^0.338)^1.1;                                                % sea level is used

    W_fus(2) = 0.052*Sfus^1.086*(nult*Wdg)^0.177...                         % 3. Raymer method, pg 460
        * lh^(-0.051) * (Lfn/wf)^(-0.072) * q^0.241;

    W_fus_lb = mean(W_fus);
end