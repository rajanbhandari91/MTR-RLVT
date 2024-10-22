function W_hyd = EvalHydWeight_lb(X)
    % Reviewed 11/19/2021 2129 hrs
    % 1. Raymer, General Aviation Weights, Pg. 576

    %% Nomenclature
    % W_dg = Design Flgiht Gross Weight (We usually assume WTO)
    % M = Design Mach Number


% if M < 0.4
%     K_h = 0.05;                                                            
% elseif M >= 0.4 && M < 0.8
%      K_h = 0.11;
% elseif M >= 0.8
%     K_h = 0.12;
% else
%     K_h = 0.013;
% end
% 
%     W_hyd_lb = K_h*(W_dg^0.8)*(M^0.5);



% Author: Cole McCormick
% Last Update: 10/21/2024

%% Parameters %%

% W_EQhyd    = hydraulic equipement weight (fixed) {lb}
% W_mb       = fight controls boost mechanism weight {lb}
% w_fc       = flight controls weight {lb}
% f_hyd      = hydraulics weight (fraction boost mechanism weight) (typical = 0.4, tiltwing)
% X_hyd      = hydraulics weight factor (1 if present, else 0)
% ConfigFlag = (Fixed Wing / Tilt Rotor = 0, Tiltwing = 1)

%% Unpack Inputs %%

W_EQhyd = X(1); % hydraulic equipement weight (fixed) {lb}
W_mb = X(2); % fight controls boost mechanism weight {lb}
w_fc = X(3); % flight controls weight {lb}
f_hyd = X(4); % hydraulics weight (fraction boost mechanism weight) (typical = 0.4, tiltwing)
X_hyd = X(5); % hydraulics weight factor (1 if present, else 0)
ConfigFlag = X(6); % (Fixed Wing / Tilt Rotor = 0, Tiltwing = 1)

%% AFDD Weight Model Hydraulic Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.239
% Consists of hydrualics for flight controls and equipement.

% AFDD82 Parametric Model

switch ConfigFlag
    case 0 % Fixed Wing or Tilt Rotor
        W_hyd = X_hyd * f_hyd * W_mb;
    case 1 % Tilt Wing
        W_hyd = X_hyd * f_hyd * w_fc;
end

W_hyd = W_hyd + W_EQhyd;

end