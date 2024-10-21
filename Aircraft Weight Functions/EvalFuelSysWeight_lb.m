function W_FS = EvalFuelSysWeight_lb(X)
    % Reviewed 11/19/21 2102 hrs

    %% Nomenclature
    % Qtot = Total volume of fuel [usg]
    % Qint = Total volume of fuel in integral tanks [USG]
    % Ntank = Number of Fuel Tanks
    % Neng = Number of Engines
    
    %FS_lb = 2.49*(Qtot^0.726)*(1/(1 + Qtot/Qint))^0.363...                   % 1. Raymer, pg 461 (also Nicolai and USAF methods)
    %    *Ntank^0.242*Neng^0.157;


% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% N_int = number of internal fuel tanks
% C_int = internal fuel tank capacity {gal}
% f_bt = ballistic tolerance factor (1.0-2.5)
% f_cw = ballisitc survivability level (1.3131 for ballistically survivable, else 1)
% N_plumb = total number of fuel tanks (internal and auxillary) for plumbing
% N_eng = number of main engines
% K0_plumb = plumbing weight, constant 0 (sum of weights for auxillary fuel systems)
% K1_plumb = plumbing weight, constant 1 (typical value = 2)
% F = fuel flow rate {lb/hr}
% f_tank = fuel tank weight (fraction fuel capacity)
% X_tank = fuel tank factor (1 if present, else 0)
% X_plumb fuel plumbing factor (1 if present, else 0)

%% Unpack Inputs %%

N_int = X(1);
C_int = X(2);
N_plumb = X(3);
N_eng = X(4);
K0_plumb = X(5);
K1_plumb = X(6);
F = X(7);

%f_tank = X(8);
f_bt = X(9);
f_cw = X(10);

X_tank = X(11);
X_plumb = X(12);


%% AFDD Weight Model Fuel System %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.235

% Fuel system consists of tanks, support structure, and fuel plumbing
% Two methods presented; parametric and fractional

%%%% Parametric Model (AFDD82) %%%%

% Fuel Tank Weight
W_tank(1) = X_tank * 0.4341 * (C_int^0.7717) * (N_int^0.5897) * (f_cw) * (f_bt^1.9491);

% Plumbing Weight
W_plumb = X_plumb * ( K0_plumb + K1_plumb * (0.01*N_plumb + 0.06*N_eng) * ((F/N_eng)^0.866) );

%%%% Fractional Model %%%%

%W_tank(2) = X_tank * f_tank * W_fuel; Removing for now due to lack of data for fractional model

% Total Fuel System Weight
W_FS = W_tank + W_plumb;
end