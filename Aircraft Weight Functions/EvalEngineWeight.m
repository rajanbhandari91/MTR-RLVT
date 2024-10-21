function [W_ES,W_eng,W_exh,W_acc] = EvalEngineWeight(X)

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% N_eng  = number of main engines or jets
% P      = installed takeoff power/thrust (SLS static, specified rating) per engine {hp} {lb}
% K0_exh = engine exhaust weight vs. power/thrust constant 0
% K1_exh = engine exhaust weight vs. power/thrust constant 1
% X_eng  = engine or jet factor (1 if engine present, else 0)
% X_exh  = engine exhaust factor (1 if supports present, else 0)
% X_acc  = engine accesories factor (1 if supports present, else 0)
% f_lub  = lubrication system factor (1.4799 if lubrication included in accessories, else 1)

%% Unpack Inputs %%

N_eng = X(1); 
P = X(2); 
K0_exh = X(3); 
K1_exh = X(4);

f_lub = X(5);

X_eng = X(6); 
X_exh = X(7); 
X_acc = X(8); 

%% AFDD Weight Model Engine System Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.234

% Engine system consists of main engines, engine exhaust system, and engine
% accessories.

% Engine or Jet Weight
W_eng = X_eng * N_eng * W_N_eng;

% Exhaust System Weight
W_exh = X_exh * N_eng * (K0_exh + K1_exh*P);

% Engine Accessories Weight
W_acc = X_acc * 2.0088 * f_lub * ((W_eng/N_eng)^0.5919) * (N_eng^0.7858);

% Total Engine System Weight
W_ES = W_eng + W_exh + W_acc;

end