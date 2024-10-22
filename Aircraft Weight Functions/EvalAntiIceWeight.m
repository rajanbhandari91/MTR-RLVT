function [W_ai,W_ai_sys,W_ai_elect] = EvalAntiIceWeight(X)

% Author: Cole McCormick
% Last Update: 10/21/2024

%% Parameters %%

% A_blade = total blade area of rotors {ft^2}
% l_wing = wing length (less fuseladge width) {ft}
% k_elect = electrical system weight factor
% k_rotor = rotor de-ice system weight factor
% k_wing = wing de-ice system weight factor
% k_air = engine air intake de-ice system weight factor
% k_jet = jet air intake de-ice system weight factor
% W_eng = total engines weight {lb}
% W_jet = total jet weight {lb}
% X_elect = anti-ice electrical system weight factor (1 if present, else 0)
% X_sys anti-ice system weight factor (1 if present, else 0)

%% Unpack Inputs %%

A_blade = X(1);
l_wing = X(2);

k_elect = X(3);
k_rotor = X(4);
k_wing = X(5);
k_air = X(6);
k_jet = X(7);

W_eng = X(8);
W_jet = X(9);

X_elect = X(10);
X_sys = X(11);

%% AFDD Weight Model Hydraulic Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.240

W_ai_elect = X_elect * k_elect * A_blade;
W_ai_sys = X_sys * (k_rotor * A_blade + k_wing * l_wing + k_air * W_eng + k_jet * W_jet);

W_ai = W_ai_sys * W_ai_elect;
end