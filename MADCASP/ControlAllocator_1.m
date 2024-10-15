function [U] = ControlAllocator(UTrimVec,FltCon,Vehicle)

%% Wash-in/Wash-out Function Standardization
% x1 = start speed, x2 = end speed
% m = 1/(x2-x1);

% wash-out = max(0,min(m(x2 - x),1));
% wash-in = max(0,min(m(x - x2) + 1,1));


% LPC-03 Phoenix (02/21/2022)

%% CONTROL EFFECTOR DEFINITIONS 
% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Left rudder deflection, deg, positive TER
% 8. Right rudder deflection, deg, positive TER

% 9. Cruise prop RPM

% 10. Lift prop 1 RPM
% 11. Lift prop 2 RPM
% 12. Lift prop 3 RPM
% 13. Lift prop 4 RPM
% 14. Lift prop 5 RPM
% 15. Lift prop 6 RPM
% 16. Lift prop 7 RPM
% 17. Lift prop 8 RPM


%% CONTROL VARIABLE DEFINITIONS
% UNames.CTRL = {'ULAT','ULONG','UDIR','CPRPM','LPRPM'};
ulat = UTrimVec(1);             % positive for rolling right
ulong = UTrimVec(2);            % positive for pitching up
udir = UTrimVec(3);             % positive for yawing right
CPRPM0 = UTrimVec(4);           % cruise motor RPM
LPRPM0 = UTrimVec(5);           % collective lift prop RPMs
dtcp = UTrimVec(6);             % cruise prop throttle


%% Control Mapping

% Control Limits 
ControlLimits = 30; % deg


% Flaperon movements
dF1 = ControlLimits * ulat;
dF2 = ControlLimits * ulat;
dF3 = - ControlLimits * ulat;
dF4 = - ControlLimits * ulat;

% Elevator movements
dE1 = - ControlLimits * ulong;
dE2 = - ControlLimits * ulong;

% Rudder movements
dR1 = ControlLimits * udir;
dR2 = ControlLimits * udir;

% cruise propeller RPM (direct)
CPRPM = CPRPM0;

% wash-in/wash-out defintions
% x1 = start speed, x2 = end speed
% m = 1/(x2-x1);
V1 = 20;    % kts
V2 = 25;    % kts
m = 1/(V2 - V1);

% wash-out = max(0,min(m(x2 - x),1));
% wash-in = max(0,min(m(x - x2) + 1,1));

% lift rotor control washout logic
KRPM_pitch = max(0,min(m*(V2-FltCon.KEAS),1));  % wash out lift propulsors
KRPM_roll = KRPM_pitch;
KRPM_yaw = KRPM_roll;

% lift propulsor RPM deltas
delta_RPM = 4500;   % tune this according to trim results
Bth = delta_RPM * ulong * KRPM_pitch;
Bph = delta_RPM * ulat  * KRPM_roll;
Bps = delta_RPM * udir  * KRPM_yaw;

% RPM mixing matrix (1)
dBmat = Vehicle.Controls.dBmat;
% RPM mixing matrix (2)
% dBmat = [...
%     +1.0, +1.0, -1.0;
%     +1.0, +1.0, +1.0;
%     -1.0, +1.0, -0.0;
%     -1.0, +1.0, +0.0;
%     +1.0, -1.0, -0.0;
%     +1.0, -1.0, +0.0;
%     -1.0, -1.0, -1.0;
%     -1.0, -1.0, +1.0];
% Form delta RPMs
dRPM = dBmat * [Bph;Bth;Bps];

% add dRPM to form varied RPM commands
LPRPM = LPRPM0*ones(8,1) + dRPM;

%% Propagate the values of the control effectors
U = [...
    dF1
    dF2
    dF3
    dF4
    dE1
    dE2
    dR1
    dR2
    CPRPM
    LPRPM
    dtcp];