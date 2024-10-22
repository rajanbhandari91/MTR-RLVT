function [W_LG, W_LG_N] = EvalLDGWeight_lb(X)

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% W_MTO   = maximum takeoff weight {lb}
% WL      = wing loading (1.0 for helicopter) {lb/ft^2}
% N_LG    = number of landing gear assemblies
% f_LG    = landing gear weight (fraction maximum takeoff weight)
% f_LGret = retraction weight (fraction basic landing gear weight)
% f_LGcw  = crashworthiness weight (fraction basic landing gear weight)
% X_LG    = landing gear factor (0 if landing gear not present, else 1)
% X_LGret = retraction factor (0 if gear is fixed, else 1)
% X_LGcw  = crashworthiness factor (0 if gear is not rated for crash, else 1)
% W_LGbas = basic landing gear weight {lb}
% W_LGret = landing gear retraction weight {lb}
% W_LGcw  = landing gear crashworthiness weight {lb}
% W_LG    = total landing gear weight {lb}
% W_LG_N  = individual landing gear weight {lb}


%% Unpack Inputs %%

W_MTO = X(1);
WL = X(2);
N_LG = X(3);

%f_LG = X(6);
f_LGret = X(7);
f_LGcw = X(8);

X_LG = X(9);
X_LGret = X(10);
X_LGcw = X(11);


%% AFDD Weight Model  Alighting Gear Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.233

% Parametric Model
w_LG(1) = 0.4013 * (W_MTO^0.6662) * (N_LG^0.5360) * (WL^0.1525);

% Fractional Model
% w_LG(2) = f_LG * W_MTO; removing for now unless we want to use fractional
% model to check against parametric model.

W_LGbas = X_LG * w_LG;

% Retraction Weight
w_LGret = f_LGret * W_LGbas;

W_LGret = X_LGret * w_LGret;

% Crashworthiness Weight
w_LGcw = f_LGcw * (W_LGbas + W_LGret);

W_LGcw = X_LGcw * w_LGcw;

% Sum Weights
W_LG = W_LGbas+ W_LGret + W_LGcw;

% Individual Gear Weights

W_LG_N = W_LG/N_LG;
                          
end