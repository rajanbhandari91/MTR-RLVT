function [FltCon] = SetDefaultFltCon()
% Initialize the FltCon (flight condition) struct to some default values

FltCon.Vel = {200,'keas'};      	 % velocity value, velocity type (can be KEAS, KTAS, or MACH)
FltCon.FPA = [];                     % flightpath angle, deg
FltCon.ALT = {0,'ft'};               % MSL altitude value, MSL altitude unit (can be ft or m)
FltCon.RV = inf;                     % flight path radius of curvature in vertical plane, m
FltCon.Turn = {'bank',0};            % turn specification variable (bank, loadfactor, turnradius, turnrate), value of this variable
FltCon.TRK = 090;                    % track, deg
FltCon.WindSpd = 00;                 % windspeed, kt
FltCon.WindDir = 090;                % wind direction, deg
FltCon.Updraft = 0;                  % updraft, kt
FltCon.HDG = [];                     % heading, deg
FltCon.BETA = [];                    % sideslip, deg

FltCon.SpecControls = [];
FltCon.SpecAddlStates = [];
end