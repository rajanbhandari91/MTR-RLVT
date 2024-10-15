function [P_kw_avail,EngMass_kg,EngLength_m,EngDiam_m,SFC_kgkwhr] = TS_IES(P_sl_hp,Velo_kts,Alt_ft,RTinterp,TLinterp)
        % Reviewed 11/19/21 2056 hrs

        %% Nomenclature
        % P_sl_hp = Sea level Available Horsepower
        % Velo_kts = Velocity in knots
        % Alt_ft = altitude in knots
        % RTinterp = Thust Residue Gridded Intepolant
        % TLinterp = Thrust Lapse Gridded Interpolant

        %% Getting Normalized Power Lapse and Residue Thrust
        alpha_P = TLinterp(Velo_kts,Alt_ft);                               
        normT_r = RTinterp(Velo_kts,Alt_ft);                               
        %% Calculating Power Available from the normalized Power lapse and Residule Thrust
        eta_inl = 0.98;                                                     % Inlet Efficiency (Roskam Given Value)
        Velo_fts = Velo_kts*1.68781;                                        % Converting Velocity to ft/sec
        T_r = 0;                                                            % Calculating actual residual thurst from norm (ZEROED OUT) ################################################
        P_av = (eta_inl*P_sl_hp*alpha_P) + eta_inl*T_r*Velo_fts/550;        % Combining Residual Thrust with Power Lapse to get Power Available [hp]
        %% Calculating Approximate Engine Dimensions
        P_kw_avail = P_av*0.7457;
        EngMass_kg = (-0.0001*(P_sl_hp)^2 + 0.5547*(P_sl_hp) - 20.519)*0.453592;  % [kg]                              
        EngLength_m = (3.8570*(P_sl_hp)^0.373)*0.0254;                      % [m]
        EngDiam_m = (10.112*(P_sl_hp)^0.12)*0.0254;                         % [m]
        SFC_kgkwhr = (1.8784*(P_sl_hp)^-0.173)*(0.4536/0.746);              % [kg/kW-hr] (lb -> 0.4536 kg) (hp -> 0.746 kW)
    end