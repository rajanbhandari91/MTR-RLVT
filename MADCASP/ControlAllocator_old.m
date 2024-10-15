function [U] = ControlAllocator(UTrimVec,FltCon,Vehicle)

%% Wash-in/Wash-out Function Standardization
% x1 = start speed, x2 = end speed
% m = 1/(x2-x1);

% wash-out = max(0,min(m(x2 - x),1));
% wash-in = max(0,min(m(x - x2) + 1,1));


% RAVEN

%% CAPTURE Control vector
% RAVEN

% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left MBD flaperon deflection, deg, positive TED
% 3. Left IBD flaperon deflection, deg, positive TED
% 4. Right IBD flaperon deflection, deg, positive TED
% 5. Right MBD flaperon deflection, deg, positive TED
% 6. Right OBD flaperon deflection, deg, positive TED
% 7. Stabilator deflection, deg, positive TED
% 8. Rudder deflection, deg, positive TER

% 9.  dt1, Prop deflection angle, deg, [0 - 90]
% 10. dt2, Prop deflection angle, deg, [0 - 90]
% 11. dt3, Prop deflection angle, deg, [0 - 90]
% 12. dt4, Prop deflection angle, deg, [0 - 90]

% 13. n1, Prop 1 RPM
% 14. n2, Prop 2 RPM
% 15. n3, Prop 3 RPM
% 16. n4, Prop 4 RPM
% 17. n5, Prop 5 RPM
% 18. n6, Prop 6 RPM


% 19. c1,  Prop 1 Pitch
% 20. c2,  Prop 2 Pitch
% 21. c3,  Prop 3 Pitch
% 22. c4,  Prop 4 Pitch
% 23. c5,  Prop 5 Pitch
% 24. c6,  Prop 6 Pitch



%% CONTROL VARIABLE DEFINITIONS
% UNames.CTRL = {'ULAT','ULONG','UDIR','PROPANG','B0','CPRPM','LPRPM'};
ulat = UTrimVec(1);         % normalized lateral control [-1,+1], positive right wing-down
ulong = UTrimVec(2);        % normalized longitudinal control [-1,+1], positive nose-up
udir = UTrimVec(3);         % normalized directional control [-1,+1], positive nose-right
% PropAng = UTrimVec(4);      % wing angle (deg)
% B0 = UTrimVec(5);           % propeller collective pitch (deg)
CPRPM = UTrimVec(4);           % cruise prop RPM
LPRPM = UTrimVec(5);           % collective lift prop RPMs


% Set LiftFan RPM Rotor 5 and 6 based on airspeed
MtipFans = 0.85;
LPRadius = Vehicle.Propulsion.LPSetup.RotorRadius;
LPRPM(FltCon.KEAS<30) = (60/(2*pi)) * (1/LPRadius) * sqrt((FltCon.SpeedOfSound * MtipFans)^2 - (FltCon.KTAS*0.5144)^2);
LPRPM(FltCon.KEAS>=30) = 0;



%% Control Mapping
% settings
FlaperonLim = 30;                       % deg, deflection limits
StabilatorLim = 30;                     % deg, deflection limit
FlaperonLimVFM = 90;                    % deg, deflection limits in vertical flight
RudderLim = 30;                         % deg, deflection limits

% Flaperon movements
dF1 = FlaperonLim * ulat;
dF2 = FlaperonLim * ulat;
dF3 = FlaperonLim * ulat;
dF4 = -FlaperonLim * ulat;
dF5 = -FlaperonLim * ulat;
dF6 = -FlaperonLim * ulat;

% Stabilator movements
dS = - StabilatorLim * ulong;

% Rudder movements
dR = RudderLim * udir;




%% Propagate the values of the control effectors
% U = [...
%     dF1
%     dF2
%     dF3
%     dF4
%     dF5
%     dF6
%     dS
%     dR
%     dt1
%     dt2
%     dt3
%     dt4
%     CPRPM
%     LPRPM
%     cCP
%     cLP
%     ];

U = [...
    dF1
    dF2
    dF3
    dF4
    dF5
    dF6
    dS
    dR
    dt1
    dt2
    dt3
    dt4
    CPRPM
    LPRPM
    ];