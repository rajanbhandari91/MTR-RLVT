function [W_FC,W_nb,W_mb,W_b] = EvalFlightControlsWeight(X)

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% W_cc       = cockpit controls weight {lb}
% W_afcs     = automatic flight control system weight {lb}
% S_ht       = horizontal tail planform area {ft^2}
% W_MTO      = maximum takeoff weight {lb}
% N_rotor    = number of main rotors
% N_blade    = number of blades per rotor
% c          = rotor mean blade chord
% V_tip      = rotor hover tip velocity
% f_hyd      = rotary wing hydraulics weight (fraction hydraulics plus boost mechanism weight) (typical value = 0.4)
% f_red      = fcs redundacny factor (range 1.0-3.0)
% f_nbsv     = non-boosted controls survival factor (1.8984 for ballistically survivable, else 1)
% f_mbsv     = boost mechanism survival factor (1.3029 for ballistically survivable, else 1)
% f_bsv      = boosted controls survival factor (1.1171 for ballistically survivable, else 1)
% f_nb       = non-boosted weight (fraction total flight controls weight) (typical value = 0.6, range 0.3-1.8, for RW)
% f_CVnb     = tilt rotor non-boosted weight (fraction boost mechanism weight)
% f_CVmb     = tilt rotor boost mechanism weight (fraction maximum takeoff weight)
% X_nb       = non-boosted controls factor (1 if present, else 0)
% X_mb       = boost mechanism factor (1 if present, else 0)
% X_b        = boosted controls factor (1 if present, else 0)
% X_CVnb     = non-boosted conversion controls factor (1 if present, else 0)
% X_CVmb     = boost mechanism conversion controls factor (1 if present, else 0)
% ConfigFlag = (Fixed Wing = 0, Helicopter = 1, Tiltwing = 2, Tiltrotor = 3)

%% Unpack Inputs %%

W_cc = X(1); % cockpit controls weight {lb}
W_afcs = X(2); % automatic flight control system weight {lb}

S_ht = X(3); % horizontal tail planform area {ft^2}
W_MTO = X(4); % maximum takeoff weight {lb}
N_rotor = X(5); % number of main rotors
N_blade = X(6); % number of blades per rotor
c = X(7); % rotor mean blade chord
V_tip = X(8); % rotor hover tip velocity

f_hyd = X(9); % rotary wing hydraulics weight (fraction hydraulics plus boost mechanism weight) (typical value = 0.4)
f_red = X(10); % fcs redundacny factor (range 1.0-3.0)
f_nbsv = X(11); % non-boosted controls survival factor (1.8984 for ballistically survivable, else 1)
f_mbsv = X(12); % boost mechanism survival factor (1.3029 for ballistically survivable, else 1)
f_bsv = X(13); % boosted controls survival factor (1.1171 for ballistically survivable, else 1)
f_nb = X(14); % non-boosted weight (fraction total flight controls weight) (typical value = 0.6, range 0.3-1.8, for RW)
f_CVnb = X(15); % tilt rotor non-boosted weight (fraction boost mechanism weight)
f_CVmb = X(16); % tilt rotor boost mechanism weight (fraction maximum takeoff weight)

X_nb = X(17); % non-boosted controls factor (1 if present, else 0)
X_mb = X(18); % boost mechanism factor (1 if present, else 0)
X_b = X(19); % boosted controls factor (1 if present, else 0)
X_CVnb = X(20); % non-boosted conversion controls factor (1 if present, else 0)
X_CVmb = X(21); % boost mechanism conversion controls factor (1 if present, else 0)

ConfigFlag = X(22); % (Fixed Wing = 0, Helicopter = 1, Tiltwing = 2, Tiltrotor = 3)

%% AFDD Weight Model Flight Controls Group %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.237
% Consists of cockpit controls, automatic flight control system, and system
% controls. 
% cockpit controls, and automatic fcs are fixed weights.

switch ConfigFlag
    case 0 % Fixed Wing
        w = 0.91*W_MTO^0.6;

        W_nb = X_nb*f_nb*w;
        W_mb = X_mb*(1-f_nb)*w;
        W_b = 0;

    case 1 % Helicopter
        w = 0.01735 * (W_MTO^0.64345) * (S_ht^0.40952);

        W_nb = X_nb*f_nb*w;
        W_mb = X_mb*(1-f_nb)*w;
        W_b = 0;

    case 2 % Rotary Wing
        % Parametric Method
        w_fc = 0.2873 * f_mbsv * ((N_rotor*N_blade)^0.6257) * (c^1.3286) * ((0.01*V_tip)^2.1129) * (f_red^0.8942);

        W_nb = X_nb * 2.1785 * f_nbsv * (W_MTO^0.3999) * (N_rotor^1.3855);
        W_mb = X_mb * (1-f_hyd) * w_fc;
        W_b = X_b * 0.02324 * f_bsv * ((N_rotor*N_blade)^1.0042) * (N_rotor)^0.1155 * (c^2.2296) * ((0.01*V_tip)^3.1877);

        % Fraction Method
        % W_nb = X_nb * f_nb * (1-f_hyd)*w_fc; commenting out for now due to lack of data for f_nb

    case 3 % Tilting Rotor
        w = 0.91*W_MTO^0.6;

        W_nb = X_nb*f_nb*w;
        W_mb = X_mb*(1-f_nb)*w;
        W_b = 0;

        W_CVmb = X_CVnb * f_CVnb * W_MTO;
        W_CVnb = X_CVmb * f_CVmb * W_CVmb;

        W_nb = W_nb + W_CVnb;
        W_mb = W_mb + W_CVmb;
end

% Total Flight Controls Weight
W_FC = W_nb + W_mb + W_b + W_cc + W_afcs;

end