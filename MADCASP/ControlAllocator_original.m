function [U] = ControlAllocator(UTrimVec,FltCon,Vehicle)



% LPC-03 Phoenix (02/21/2022)

%%%% CONTROL EFFECTOR DEFINITIONS 
% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Left rudder deflection, deg, positive TER
% 8. Right rudder deflection, deg, positive TER

% 9. Cruise prop throttle, [0-1]
% 10. Cruise prop pitch, deg
% 11. Cruise prop RPM

% 12. Lift prop 1 RPM
% 13. Lift prop 2 RPM
% 14. Lift prop 3 RPM
% 15. Lift prop 4 RPM
% 16. Lift prop 5 RPM
% 17. Lift prop 6 RPM
% 18. Lift prop 7 RPM
% 19. Lift prop 8 RPM

% 20. Lift prop 1 pitch
% 21. Lift prop 2 pitch
% 22. Lift prop 3 pitch
% 23. Lift prop 4 pitch
% 24. Lift prop 5 pitch
% 25. Lift prop 6 pitch
% 26. Lift prop 7 pitch
% 27. Lift prop 8 pitch


%%%% CONTROL VARIABLE DEFINITIONS
% UNames.CTRL = {'ULAT','ULONG','UDIR','THR','BPcp','CPRPM','B0','LPRPM'};
ulat = UTrimVec(1);             % positive for rolling right
ulong = UTrimVec(2);            % positive for pitching up
udir = UTrimVec(3);             % positive for yawing right
dt_cp = UTrimVec(4);            % throttle
bp_cp = UTrimVec(5);            % cruise prop pitch
rpm_cp = UTrimVec(6);           % cruise motor RPM
B0 = UTrimVec(7);               % lift rotors collective pitch



Mtip = 0.3;

% Tip Mach with lift rotor failures
if sum(Vehicle.Propulsion.LPSetup.HealthStatus)<8
%     Vehicle.Propulsion.LPSetup.HealthStatus
    Mtip= 0.35;
end

rpm_lp = (Mtip * FltCon.SpeedOfSound - FltCon.KTAS * 0.51444) * (2/Vehicle.Propulsion.LiftPropDiam_m) * (30/pi);
rpm_lp(FltCon.KEAS > Vehicle.Aero.VThreshold/0.51444) = 0;





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

% cruise propeller throttle (direct)
dt = dt_cp;

% cruise propeller pitch (direct)
CPBladePitch = bp_cp;

% cruise propeller RPM (direct)
CPRPM = rpm_cp;

% lift propeller RPM (direct)
LPRPM = rpm_lp*ones(8,1);

% lift rotor control washout logic
KB_pitch = min(1,max(0,1 - (FltCon.KEAS - 100)/(140-100)));  % updated
KB_roll = KB_pitch;
KB_yaw = KB_roll;

% lift propulsor blade pitch
Bth = + 10 * ulong * KB_pitch;
Bph = + 10 * ulat  * KB_roll;
Bps = + 10 * udir  * KB_yaw;

dBmat= Vehicle.Controls.dBmat;
dB= dBmat * [Bph;Bth;Bps];

LPBladePitch = B0 + dB;


% propagate the values of the control effectors
U = [...
    dF1
    dF2
    dF3
    dF4
    dE1
    dE2
    dR1
    dR2
    dt
    CPBladePitch
    CPRPM
    LPRPM
    LPBladePitch];