
function [Cl, Cir_CS]= EMPAERO_CalcDeltaClCS(Aero, ControlDef, StripVel,Chord, Vehicle)
span = (2*Aero.LS.Span);
CSType = Vehicle.Aero.ControlSurfaceType;

if ~isempty(Aero.CSname)
    [~,ncs] = size(Aero.CSname);

    % for each control, calculate incremental circulation due to deflection
    for j = 1:ncs
        CSname = Aero.CSname{j};
        G_to_delta = Aero.(CSname).G_to_delta;
        % control deflections are captured in order they are assigned in
%       VehicleDefinition_Baseline
        % Order of deflections anlge changed for CS of LS on the
        % left of symmetry before calling this function
        delta = ControlDef(j);
        FCR = Aero.ControlChordFracs.(CSname);

        if strcmp(CSType,'Plain')
            K_prime = Vehicle.Aero.TEFGI_K_prime(abs(delta),FCR);           % empirical correction for lift effectiveness of plain TE flap at high flap deflections
            alpha_d =  Aero.(CSname).alpha_d_over_Kprime * K_prime;
            G = G_to_delta * delta/57.3 * alpha_d;
            Gamma_CS_matrix(:,j) = G.*span.*StripVel;
        end

        if strcmp(CSType, 'SingleSlotted')
            alpha_d = Vehicle.Aero.SingleSlottedTEFlapLiftEffParamGI(abs(delta),FCR);
            G = G_to_delta * delta/57.3 * alpha_d;
            Gamma_CS_matrix(:,j) = G.*span.*StripVel;
        end

    end
    Cir_CS = sum(Gamma_CS_matrix,2);
else
    Cir_CS = 0;
end
Cl = (2*Cir_CS./StripVel)./Chord;
Cl(isnan(Cl)) = 0;
end