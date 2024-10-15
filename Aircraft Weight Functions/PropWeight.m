    function Prop_W_lb = PropWeight(KW,D_ft,B,AF,N,SHP,M)
        % Reviewed 11/19/2021 2111 hrs
        
        % Couldnt not find reference for this. It was an older empiracal
        % method it seemed

        %% Nomenclature
        % KW Blade Material Coefficient (Usually sits around 200)
        % D_ft Blade Diameter [ft]
        % B = Number of Blades
        % AF = Blade Activity Factor
        % N = Blade RPM
        % SHP = Shaft Horsepower [hp]
        % M = Design Mach Number
        
        Prop_W_lb = KW*((D_ft/10)^2*(B/4)^0.7*(AF/100)^0.75*...
            (N*D_ft/20000)^0.5*(SHP/(10*D_ft^2))^0.12*(M+1)^0.5);

    end