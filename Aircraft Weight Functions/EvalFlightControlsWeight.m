function [W_FC] = EvalFlightControlsWeight(X)

% Author: Cole McCormick
% Last Update: 10/20/2024

%% Parameters %%

% S_ht = horizontal tail planform area {ft^2}
% W_MTO = maximum takeoff weight {lb}
% N_rotor = number of main rotors
% N_blade = number of blades per rotor
% c = rotor mean blade chord
% V_tip = rotor hover tip velocity
% f_hyd = rotary wing hydraulics weight (fraction hydraulics plus boost mechanism weight) (typical value = 0.4)
% f_red = fcs redundacny factor (range 1.0-3.0)
% f_mbsv = boost mechanism survival factor (1.3029 for ballistically survivable, else 1)
% f_bsv = boosted controls survival factor (1.1171 for ballistically survivable, else 1)
% f_nb = non-boosted weight (fraction total flight controls weight) (typical value = 0.6, range 0.3-1.8, for RW)
% f_CVnb = tilt rotor non-boosted weight (fraction boost mechanism weight)
% f_CVmb = tilt rotor boost mechanism weight (fraction maximum takeoff weight)
% X_nb = non-boosted controls factor (1 if present, else 0)
% X_mb = boost mechanism factor (1 if present, else 0)
% X_b = boosted controls factor (1 if present, else 0)
% X_CVnb = non-boosted conversion controls factor (1 if present, else 0)
% X_CVmb = boost mechanism conversion controls factor (1 if present, else 0)

%% Unpack Inputs %%

S_ht = X(1);
W_MTO = X(2);
N_rotor = X(3);
N_blade = X(4);
c = X(5);
V_tip = X(6);

f_hyd = X(7);
f_red = X(8);
f_mbsv = X(9);
f_bsv = X(10);
f_nb = X(11);
f_CVnb = X(12);
f_CVmb = X(13);

X_nb = X(14);
X_mb = X(15);
X_b = X(16);
X_CVnb = X(17);
X_CVmb = X(18);

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