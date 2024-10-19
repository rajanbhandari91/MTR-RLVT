function [M_wing] = EvalWingWEIGHT(X)

% Author: Cole McCormick
% Last Update: 10/19/2024

%% Parameters %%

% W_box = wing torque box weight {kg}
% W_spar = wing spar weight {kg}
% W_SD = structural design gross weight {N}
% T_max_jump = rotor maximum thrust capability (Jump Takeoff) (C_T / sigma)
% n_jump = load factor at W_SD (jump takeoff)
% N_rotor = number of rotors
% b_fold = fraction of wing span that folds (0 to 1)
% b_w = wing span (length of torque box) {m}
% l_w = wing length (minus fuselage width) {m}
% c_w = wing chord {m}
% w_tb = torque box chord to wing chord ratio
% c_tb = torque box chord {m}
% T_w = wing airfoil thickness-to-chord ratio
% t_w = wing thickness {m}
% r_pylon = pylon radius of gyration {m}
% Sigma = rotor speed for wing weight design condition {rad/sec}
% w_T = wing torsion mode frequency (fraction rotor speed) {per rev}
% w_B = wing beam bending mode frequency (fraction rotor speed) {per rev}
% w_C = wing chord bending mode frequency (fraction rotor speed) {per rev}
% rho_tb = density of torque box material {kg/m^3}
% rho_sp = density of spar material {kg/m^3}
% G_tb = torque box shear modulus {N/m^2}
% E_tb = torque box modulus {N/m^2}
% E_sp = spar modulus {N/m^2}
% e_U = ultimate strain allowable (min of spar and torque box)
% C_t = weight correction for spar taper (equivalent siffness)
% C_j = weight correction for spar taper (equivelant strength)
% C_m = strength correction for spar taper (equivelant stiffness)
% e_tb = structural efficiency factor (torque box)
% e_Sp = structural efficiency factor (spars)
% U_fair = unit weight of leading and trailing edge fairings {kg/m^2}
% U_flap = unit weight of control surfaces {kg/m^2}
% U_ext = unit weight of wing extenstion {kg/m^2}
% S_fair = area of leading and trailing edge fairings {m^2}
% S_flap = area of control surfaces {m^2}
% S_ext = area of wing extensions {m^2}
% f_fit = wing fittings and brackets (fraction maximum thrust of one rotor)
% f_fold = wing fold/tilt (fraction total weight excluding fold, including weight on tips)
% f_efold = wing extension fold/tilt (fraction extenstion weight)
% w_attach = width of wing structural attachements to body {m}
% lamda_w = wing taper ratio
% A_w = wing aspect ratio
% Delta_w = wing sweep angle (? MAC) {deg}
% n_z = design ultimate flight load factor at W_SD {g}
% U_w = unit weight of wing planform {kg/m^2}
% S_w = wing planform area {m^2}
% T_cap = maximum thrust of one rotor {N}
% M_tip = tip mass at end of wing {kg}

%% Unpack Inputs %%

w_attach = X(1); % width of wing structural attachements to body {m}
r_pylon = X(2); % pylon radius of gyration {m}
T_cap = X(3); % maximum thrust of one rotor {N}
Sigma = X(4); % rotor speed for wing weight design condition {rad/sec}

W_SD = X(5); % structural design gross weight {N}
M_tip = X(6); % tip mass at end of wing {kg}

w_T = X(7); % wing torsion mode frequency (fraction rotor speed) {per rev}
w_B = X(8); % wing beam bending mode frequency (fraction rotor speed) {per rev}
w_C = X(9); % wing chord bending mode frequency (fraction rotor speed) {per rev}

U_fair = X(10); % unit weight of leading and trailing edge fairings {kg/m^2}
U_flap = X(11); % unit weight of control surfaces {kg/m^2}
U_ext = X(12); % unit weight of wing extenstion {kg/m^2}

S_flap = X(13); % area of control surfaces {m^2}
S_ext = X(14); % area of wing extensions {m^2}
S_w = X(15); % wing planform area {m^2}
A_wing = X(16); % wing aspect ratio
Delta_w = X(17); % wing sweep angle {deg}
lambda_w = X(18); %wing taper ratio
b_w = X(19); % wing span (length of torque box) {m}
c_w = X(20); % wing chord {m}
b_fold = X(21); % fraction of wing span that folds (0 to 1)
l_w = X(22); % wing length (minus fuselage width) {m}
T_w = X(23); % wing airfoil thickness-to-chord ratio
w_tb = X(24); % torque box chord to wing chord ratio
n_z = X(25); % design ultimate flight load factor at W_SD {g}

E_tb = X(26); % torque box modulus {N/m^2}
E_sp = X(27); % spar modulus {N/m^2}

e_U = X(28); % ultimate strain allowable (min of spar and torque box)
G_tb = X(29); % torque box shear modulus {N/m^2}

e_tb = X(30); % structural efficiency factor (torque box)
e_sp = X(31); % structural efficiency factor (spars)

rho_tb = X(31); % density of torque box material {kg/m^3}
rho_sp = X(32); % density of spar material {kg/m^3}

C_m = X(33); % strength correction for spar taper (equivelant stiffness)
C_j = X(34); % weight correction for spar taper (equivelant strength)
C_t = X(35); % weight correction for spar taper (equivalent siffness)

f_fit = X(36); % wing fittings and brackets (fraction maximum thrust of one rotor)
f_fold = X(37);  % wing fold/tilt (fraction total weight excluding fold, including weight on tips)
f_efold = X(38); % wing extension fold/tilt (fraction extenstion weight)

X_prim = X(39);
X_flap = X(40);
X_fair = X(41);
X_fit = X(42);
X_fold = X(43);
X_efold = X(44);
X_ext = X(45);

tiltRotorFlag = X(46); % 1 for tilt rotor or tiltwing 0 for fixed
jumpTakeoffFlag = X(47); % 1 for jump takeoff reqiurement 0 for else
LG_Pos_Flag = X(48); % 0 for landing gear on wing 1 for otherwise


%% AFDD Weight Model %%
% NDARC NASA Design and Analysis of Rotorcraft, Wayne Johnson,
% pg.223

switch tiltRotorFlag
    case 0
        %% TILTROTOR/TILTWING EQUATIONS %%
        % Wing primarily sized to meet torsional stiffness reqiurements.

        %%%% Wing Form Factor Equations %%%%
        % Relate typical airfoil and torque box geometry to ideal shapes
        % for stiffness reiqurement calculations.

        F_B = 0.073 * sind(2*pi * (T_w - 0.151) / 0.1365 ) + (0.14569 * T_w)...
            + (0.610 * sind(2*pi * (w_tb + 0.080) / 2.1560 )) - ( (0.4126 - 1.6309 * T_w)...
            * (w_tb - 0.131) ) + 0.0081;

        F_C = (0.640424 * w_tb^2) - (0.89717 * w_tb) + (0.4615 * T_w) + 0.655317;

        F_T = ((0.27 - T_w) / 0.12) * 0.12739 * ( -0.96 + sqrt( 3.32 + (94.6788 * w_tb) - (w_tb/0.08344)^2 ) )...
            - (2.7545*w_tb^2) + (5.1799*w_tb) - 0.2683;

        F_VH = (0.25 * sind(5.236*w_tb)) + 0.325;

        %%%% Torque Box Cross Sectional Area %%%%
        % Calculates torque box cross sectional area based off wing torsion
        % frequency.

        t_w = T_w * c_w;

        GJ = (w_T * Sigma)^2 * 0.5 *(b_w - w_attach) * (0.5 * M_tip * r_pylon^2);

        A_tb = (4 * GJ) / (G_tb * F_T * t_w^2);

        %%%% Spar Cap Cross Sectional Area %%%%
        
        W_tip = M_tip * 9.81;
        f_tip = W_tip / W_SD;
        f_mode = 1 - f_tip;

        EI_C = (w_C * Sigma)^2 * (1/24) * b_w^3 * (0.5 * M_tip * f_mode);

        EI_B = (w_B * Sigma)^2 * (1/24) * b_w^3 * (0.5 * M_tip * f_mode);
        
        c_tb = w_tb * c_w;
        EI_Ctb = (E_tb*F_C*A_tb*c_tb^2)/4;

        if EI_C > EI_Ctb
            EI_Csp = EI_C - EI_Ctb;
        else
            EI_Csp = 0;
        end

        A_Csp = EI_Csp / (E_sp*c_tb^2 / 4);

        EI_Btb = E_tb * F_B * A_tb * (t_w^2) / 4;

        EI_VH = E_Sp * F_VH * A_Csp * (t_w^2) / 4;

        if EI_B > EI_Btb + EI_VH
            EI_Bsp = EI_B - EI_Btb - EI_VH;
        else
            EI_Bsp = 0;
        end

        A_Bsp = EI_Bsp / (E_sp * t_w^2 / 4);

        EI_sp = EI_VH + EI_Bsp;

        A_sp = A_Csp + A_Bsp;


        
        %%%% Primary Structure %%%%
        % Composed of torque box + spar weights

        W_box = A_tb * rho_tb * b_w / e_tb;
        W_spar = C_t * A_sp * rho_sp * b_w /e_sp;

        % Setup while loop
        % This while loop iterates wing weight as delta spar weight reqiures wing
        % weight to calculate.

        delta_W_spar = 1;
        iter = 0;
        
        while delta_W_spar > 0 && iter < 10

        w_prim = (W_box + W_spar) / 1; % if using english units must replace 1 with g.
        W_prim = X_prim * w_prim;

        %%%% Fairing Weight %%%%

        S_fair = (b_w - w_attach) * c_w * (1 - w_tb) - S_flap;

        w_fair = S_fair * U_fair;
        W_fair = X_fair * w_fair;

        %%%% Control Surface Weight %%%%

        w_flap = S_flap * U_flap;
        W_flap = X_flap * w_flap;

        %%%% Fittings Weight %%%%

        w_fit = f_fit * T_cap;
        W_fit = X_fit * w_fit;

        %%%% Fold/Tilt Weight %%%%
        
        w_fold = f_fold * (W_prim + W_fair + W_flap + W_fit + W_tip);
        W_fold = X_fold * w_fold;

        %%%% Wing Extension Weight %%%%

        w_ext = S_ext * U_ext;
        W_ext = X_ext * w_ext;

        w_efold = f_efold * W_ext;
        W_efold = X_efold * w_efold;

        W_fold = W_fold + W_efold;
        W_prim = W_prim + W_ext;

        %%%% Total Wing Weight %%%%

        W_wing = W_prim + W_fair + W_flap + W_fit + W_fold;

        %%%% Spar Cap weight for Jump Takeoff Condition %%%%

        if jumpTakeoffFlag == 1
            M_U = T_cap * l_w * ( (0.75 * (1 - f_tip)) - (0.375 * (l_w/b_w) * (W_wing/W_SD)) );
        else
            M_U = 0;
        end

        M_tb = 2* EI_Btb * e_U / t_w;

        M_sp = 2* C_m * EI_sp * e_U / t_w;
        
        if M_U > M_tb + M_sp
        delta_M = M_U - (M_tb + M_sp);
        else 
        delta_M = 0;
        end

        delta_A_sp = 2*delta_M / (e_U * E_sp * t_w);

        delta_W_spar = C_j * delta_A_sp * rho_sp * b_w / e_sp;
        
        W_spar = W_spar + delta_W_spar;
        
        iter = iter + 1;    
        end

    case 1
        %% Aircraft Wing %%

        %%%% Area Method Model %%%%

        %W_wing = S_w * U_w; Ignoring this for now since guessing U_w is
        %most likely not accurate for us.
        

        %%%% Parametric Method %%%%
        % AFDD93
        
        switch LG_Pos_Flag
            case 1
                f_LGloc = 1;
            case 0
                f_LGloc = 1.7247;
        end
        W_wing = 5.66411 * f_LGloc * ((W_SD * 0.2248089431 / (1000 * cosd(Delta_w)))^0.847) * (n_z^0.39579) * ((S_w*10.7639)^0.21754)...
            * (A_wing^0.50016) * (((1+lambda_w)/T_w)^0.09359) * ((1-b_fold)^-0.14356);
        W_wing = W_wing * 4.4482216153; % Convert from lb_f to N
        
end

    M_wing = W_wing/9.81; % Convert from N to kg

end

    