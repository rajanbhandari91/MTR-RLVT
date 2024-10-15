function [indAOA,ADDL] = EMPAERO_CalcInducedAOA(Aero,AOA,StripVel,symm,rho,Gamma_CS)
span = (2*Aero.LS.Span);                                                    % span of the lifting surface
zero_lift_AOA = Aero.zero_lift_AOA;                                         % zero lift AOA
AOA_from_zero_lift = AOA -zero_lift_AOA;                                    % absolute angle of attack
y = Aero.LS.y;                                                              % distance of exposed strip midpoints from root (y = ...StripDef.db_mid)
dy = Aero.LS.dy;                                                            % distance between strip midpoints
nStrips = height(y);                                                        % no. of strips on the lifting surface
G_to_alpha = Aero.LS.G_to_alpha;                                            % span loading per unit AOA in radian

% Note: the conversion of G/alphato  circulation over the half of the
% lifting surface requires multiplication by 2* exposed span by defn.

% The following form is maintained to compare with flightstream data
% Direct conversion : Gamma_LS = G_to_alpha .* span .* AOA_from_zero_lift/57.3 .*StripVel;
c0 = zeros(nStrips,1);                                                      % circulation, if any at zero wing angle of attack (zero for untwisted wing)
dCdA_rad = G_to_alpha*span;                                                 % per radian of angle of attack
dCdA = dCdA_rad * 1/57.3  ;                                                 % per degree of angle of attack
stripLen = Aero.LS.stripLen;                                                % length of each exposed strip
Gamma_LS = dCdA .* AOA_from_zero_lift.*StripVel;                            % circulation over the lifting surface due to lifting surface only
% Gamma_LS = 20*sqrt(1-(2*y/span).^2);                                      % Elliptical circulation for test
Gamma_Tot = Gamma_LS + Gamma_CS;                                              % circulation over the lifting surface due to lifting surface and control surface

% Compute induced AOA due to lifiting surface only
indAOA_LS = CalcInducedAOA_inner(Gamma_LS,StripVel,y,dy,stripLen,nStrips,symm,AOA_from_zero_lift);
indAOA = indAOA_LS;

% Compute induced AOA due to lifting surface and control surface
indAOA_Tot = CalcInducedAOA_inner(Gamma_Tot,StripVel,y,dy,stripLen,nStrips,symm,AOA_from_zero_lift);
ADDL.indAOA_Tot = indAOA_Tot;
%% calculate total induced drag considering the deflection of the control surfaces

% Direct calculation of Lift and induced drag
% Computed for analyis only. Not used downstream for aerodynamic forces and
% moments computation.
Lift_LS = rho * StripVel.*Gamma_LS.*stripLen;                                 % based on Anderson Pg. 441 (Reader pg. 441)
Lift_CS = rho * StripVel.*Gamma_CS.*stripLen;
InducedDrag_LS = Lift_LS.*sind(indAOA_Tot); 
InducedDrag_CS = Lift_CS.*sind(indAOA_Tot);

% InducedDrag_LS(InducedDrag_LS<0) = 0;
% InducedDrag_LS(InducedDrag_CS<0) = 0;

ADDL.InducedDrag_LS = InducedDrag_LS;
ADDL.InducedDrag_CS = InducedDrag_CS;

% Return lifitng surface only data for study
ADDL.Gamma = Gamma_LS;
ADDL.dCdA = dCdA;
ADDL.dCdA_dummy = dCdA;
ADDL.c0 = c0;
%%

% symm = 1 is evaluated
% (1) to have flexibility to  analyse lifting surface with a symmetric half
% (2) to compare the result with available symmetric wing data

% symm = 0 is set for all individual lifting surface components
% ipmplemetation based on Anderson Reader pg. 466

    function [indAOA] = CalcInducedAOA_inner(Gamma,StripVel,y,dy,stripLen,nStrips,symm,AOA_from_zero_lift)
        dGamma = diff(Gamma);                                               % difference in circulation between two adjacent strips (size: nStrips-1)
        dGamma_dy = dGamma./dy(1:nStrips-1);                                % circulation gradient (size: nStrips-1)
        % the implementaton requires dGamma_dy for all nStrips
        % extrapolate the gradient at the end strip based on slope on
        % preceeding two strips.
        dGamma_dy(nStrips) = dGamma_dy(nStrips-1) + (dGamma_dy(nStrips-1)-dGamma_dy(nStrips-2))/dy(nStrips-2) * dy(nStrips-1);
        %  something simple like this could also be done : dGamma_dy = [dGamma_dy;dGamma_dy(end)];
        if symm == 1
            dGamma_dy_symm = -dGamma_dy;
            y_symm = -y;
            % vector implementation
            % effect of strips on the same wing on the 'nth' strip
            % integrate from 0 to b/2
            den = y'-y;
            fun = dGamma_dy./den;

            % den becomes zero when y(n) = y
            % when singularity occurs
            fun(isinf(fun)) = NaN;
            sing_index = find(isnan(fun));
            fun(1) = fun(2);                                                % replace with value at adjacent strip
            fun(end) = fun(end-1);                                          % replace with value at adjacent strip
            fun(sing_index(2:end-1))= (fun(sing_index(2:end-1)+1) + fun(sing_index(2:end-1)-1))/2; % replace with average value based on two adjacent strips
            I1 = sum(fun.*stripLen);

            % effect of strips on the symmetric half on the 'nth' strip
            den = y'-y_symm;
            fun = dGamma_dy_symm./den;
            I2 = sum(fun.*stripLen);
            I = I1 + I2;
            w =  1/(4*pi)*I;                                                % downwash velocity (m/s)
        else

            % effect of strips on the same wing on 'nth' strip
            % integrate from 0 to b/2
            den = y'-y;
            fun = dGamma_dy./den;
            fun(isinf(fun)) = NaN;
            sing_index = find(isnan(fun));
            fun(1) = fun(2);                                                % replace with value at adjacent strip
            fun(end) = fun(end-1);                                          % replace with value at adjacent strip
            fun(sing_index(2:end-1))= (fun(sing_index(2:end-1)+1) + fun(sing_index(2:end-1)-1))/2; % replace with average value based on two adjacent strips
            I = sum(fun.*stripLen);
            w =  1/(4*pi)*I;                                                % downwash velocity (m/s)
        end
        w = w(:);
        indAOA = atan2d(w,StripVel + 0.0000000000000001);                   % degree
        % the outboard station of the last strip should have zero lift
        % this implies absolute angle of attack seen by last station = 0
        % which means induced angle of attack = absolute anlge of attack

        % interpolate the AOA of the penulimate strip midpoint based on the
        % inboard strip midpoint and the boundary condition
        indAOA(nStrips) = indAOA(nStrips-1)+...
            (AOA_from_zero_lift(nStrips)-indAOA(nStrips-1))*dy(nStrips-1)/(stripLen(nStrips-1)/2+stripLen(nStrips));

%        % Test 
%        % For elliptical circulation 
%        Gamma0 = 20;
%        alpha_i = Gamma0/(2*span*StripVel(1))*57.3;


    end

% table(y,Gamma_LS)
end