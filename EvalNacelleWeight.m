function [W_nacelle, W_airind] = EvalNacelleWeight(X)

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% W_MTO    = maximum takeoff weight {lb}
% W_eng    = weight all main enignes {lb}
% N_eng    = number of main engines
% S_nac    = wetted area of nacelles and pylons (less spinner) {ft^2}
% f_airind = air induction weight (fraction nacelle plus air induction) (range: 0.1-0.6)
% f_pylon  = pylon support structure weight (fraction W_MTO)
% X_supt   = engine support factor (1 if supports present, else 0)
% X_cowl   = engine cowl factor (1 of cowl present, else 0)
% X_pylon  = pylon support structure factor (1 if pylon present, else 0)
% X_airind = air induction factor (1 if air induction system present, else 0)

%% Unpack Inputs %%

W_MTO = X(1); 
W_eng = X(2); 
N_eng = X(3); 
S_nac = X(4); 

f_airind = X(5); 
f_pylon = X(6); 

X_supt = X(7); 
X_cowl = X(8); 
X_pylon = X(9); 
X_airind = X(10); 

%% AFDD Weight Model Nacelle Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.233

% nacelle group consists of: engine support structure, engine cowling, and
% pylong support structure

% Engine Support Structure Weight
W_supt = X_supt * 0.0412 * (1-f_airind) * (W_eng/N_eng)^1.1433 * N_eng^1.3762;

% Engine Cowling Weight
W_cowl = X_cowl * 0.2315 * (S_nac^1.3476);

% Pylon Support Structure Weight 
W_pylon = X_pylon * f_pylon * W_MTO;

% Total Nacelle Weight
W_nacelle = W_supt + W_cowl + W_pylon;

%% AFDD Weight Model Air Induction Group %%

W_airind = X_airind * 0.0412 * f_airind * ((W_eng/N_eng)^1.1433) * (N_eng^1.3762);

end