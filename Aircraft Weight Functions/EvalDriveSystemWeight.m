function [W_DS, W_gb, W_rs, W_ds, W_rb] = EvalDriveSystemWeight(X)

% Author: Cole McCormick
% Last Update: 10/21/2024

%% Parameters %%

% P_DSlimit   = drive system power limit (MCP) {hp}
% Q_DSlimit   = Drive system Torque limit {hp/rpm}
% N_rotor     = number of main rotors
% N_gb        = number of gear boxes
% N_ds        = number of intermediate drive shafts
% Omega_rotor = main-rotor rotation speed {rpm}
% Omega_eng   = engine output speed {rpm}
% x_hub       = length of drive shaft between motors
% V_tip       = main rotor tip speed {ft/s}
% f_Q         = second (main or tail) rotor torque limit (fraction of total drive system)
% f_P         = second (main or tail) rotor power limit (fraction of total drive system)
% f_rs        = rotor shaft weight (fraction gear box and rotorshaft) (= 0.13, range 0.06-0.20)
% X_gb        = gear box factor (1 if present, else 0)
% X_rs        = rotor shaft factor (1 if present, else 0)
% X_ds        = drive shaft factor (1 if present, else 0)
% X_rb        = rotor brake factor (1 if present, else 0)

%% Unpack Inputs %%

P_DSlimit = X(1);
N_rotor = X(2);
N_gb = X(3);
N_ds = X(4);
Omega_rotor = X(5);
Omega_eng = X(6);
x_hub = X(7);
V_tip = X(8);

f_Q = X(9);
f_P = X(10);
f_rs = X(11);

X_gb = X(12);
X_rs = X(13);
X_ds = X(14);
X_rb = X(15);

%% AFDD Weight Model Drive Sysyem %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.236
% Consists of gear boxes, rotorshafts, drive shafts, and rotor brakes.

% Preliminary Calculations
Q_DSlimit = P_DSlimit / Omega_rotor;

%%%% Gearbox and Rotorshaft weight %%%%

% AFDD82 Model
% gearbox and rotorshaft weight
w_gbrs(1) = 57.72 * (P_DSlimit^0.8195) * (f_Q)^0.0680 * (N_gb^0.0663) * ((Omega_eng/1000)^0.0369) / (Omega_rotor^0.6379);

% AFDF00 Model
% gearbox and rotorshaft weight
w_gbrs(2) = 95.7634 * (N_rotor^0.38553) * (P_DSlimit^0.78137) * (Omega_eng^0.09899) / (Omega_rotor^0.80686);


% gearbox weight
W_gb = X_gb * (1-f_rs)* mean(w_gbrs);

% rotorshaft weight
W_rs = X_rs * f_rs * mean(w_gbrs);

%%%% Drive Shaft and Rotor Brake Weight %%%%
% AFDD82 Model

W_ds = X_ds * 1.166 * (Q_DSlimit^0.3828) * (x_hub^1.0455) * (N_ds^0.3909) * ((0.01*f_P)^0.2693);

W_rb = X_rb * 0.000871 * (W_blade) * ((0.01*V_tip)^2);

%%%% Total Drive System Weight %%%%

W_DS = W_gb + W_rs + W_ds + W_rb;

end