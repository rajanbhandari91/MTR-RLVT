    function Nac_W_lb = NacWeight(K_ng,N_Lt,N_w,N_z,W_ec,N_en,S_n)
        % Reviewed 11/19/21 2109 hrs
        %% Nomenclature                                        From Raymer 6th edition, page 575 Nacelle Grouping Equation
        % K_ng = 1.017 - Pylon-Mounted Nacelle; 1 - Otherwise
        % N_Lt = Nacelle Length [ft]
        % N_w = Nacelle Width [ft]
        % nult = Ultimate Load Factor 1.5 x Limit Load factor
        % W_ec = Weight of contents [lb]
        % N_en = Number of Engines/Motors
        % S_n = Nacelle Wetted Area [ft2]

        Nac_W_lb = 0.6724*K_ng*(N_Lt^0.10)*(N_w^0.294)*...
            (N_z^0.119)*(W_ec^0.611)*(N_en^0.984)*(S_n^0.224);

    end