function [W_at] = PropWeight(X)
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
        
        %Prop_W_lb = KW*((D_ft/10)^2*(B/4)^0.7*(AF/100)^0.75*...
        %   (N*D_ft/20000)^0.5*(SHP/(10*D_ft^2))^0.12*(M+1)^0.5);

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% N_at = number of auxillary thrusters
% T_at = thrust per propellor {lb}
% P_at = power per propellor {hp}
% A_at = auxiliary thruster disk area {ft^2}
% N_blade = number of blades per propellor
% Omega_prop = propellor rotation sped at P_at {rpm}
% D_prop = propellor diameter {ft}
% f_m = material factor (composite = 1, wood = 1.2, aluminum spar = 1.31, aluminum construction = 1.44)
% X_at = auxillary propulsion/propellor factor (1 if present, else 0)

%% Unpack Inputs %%

N_at = X(1);
T_at = X(2);
P_at = X(3);
A_at = X(4);
N_blade = X(5);
Omega_prop = X(6);
D_prop = X(7);
f_m = X(8);

X_at = X(9);


%% AFDD Weight Model Propeller/Fan Installation %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.234

% AFDD82 Parametric Weight Equation
W_at(1) = X_at * 0.0809484 * N_at * (T_At^1.04771) * ((T_at/A_at)^-0.07821);

% AFDD10 Parametric Weight Equation
W_at(2) = X_at * 9.90350 * N_at * (P_at^0.91996) * (N_blade^-0.48578) * (Omega_prop^-0.459054) * (D_prop^0.15690) * f_m;

W_at = mean(W_at);


end