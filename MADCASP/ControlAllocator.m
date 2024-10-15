function [U] = ControlAllocator(UTrimVec,FltCon,Veh)

% RLVT MTR
%%% CONTROL SURFACES
% (1)	df1     Left outboard flaperon      (deg, +TED)
% (2)	df2     Left midboard flaperon      (deg, +TED)
% (3)	df3     Left inboard flaperon       (deg, +TED)
% (4)	df4     Right inboard flaperon      (deg, +TED)
% (5)	df5     Right midboard flaperon     (deg, +TED)
% (6)	df6     Right outboard flaperon     (deg, +TED)
% (7)	de1     Right Elevator              (deg, +TED)
% (8)	de2     Left Elevator               (deg, +TED)
% (9)	dr      Rudder                      (deg, +TER)
%%% NACELLE ANGLES
% (10)	dt1     Left wingtip nacelle angle
% (11)	dt2     Left forward boom prop tilt angle
% (12)	dt3     Right forward boom prop tilt angle
% (13)	dt4     Right wingtip nacelle angle
%%% PROPELLER RPM
% (14)	n1      Prop 1 RPM
% (15)	n2      Prop 2 RPM
% (16)	n3      Prop 3 RPM
% (17)	n4      Prop 4 RPM
% (18)	n5      Prop 5 RPM
% (19)	n6      Prop 6 RPM
%%% PROPELLER BLADE PITCH
% (20)	dc1     Prop 1 collective
% (21)	dc2     Prop 2 collective
% (22)	dc3     Prop 3 collective
% (23)	dc4     Prop 4 collective
% (24)	dc5     Prop 5 collective
% (25)	dc6     Prop 6 collective


ulat = UTrimVec(1);         % normalized lateral control [-1,+1], positive right wing-down
ulon = UTrimVec(2);         % normalized longitudinal control [-1,+1], positive nose-up
udir = UTrimVec(3);         % normalized directional control [-1,+1], positive nose-right

dt = UTrimVec(4);           % common nacelle tilt angle [0-horizontal, 90-vertical]

dc14 = UTrimVec(5);         % common collective for props 1-4
dc56 = UTrimVec(6);         % common collective for props 5-6


n56 = UTrimVec(7);
n14 = UTrimVec(8);
df = 0;


if FltCon.KEAS >= Veh.Aero.VThreshold/0.5144
    n56 = 0;
    df = 00;
end
% 
% 
%     n14 = UTrimVec(8);          % common RPM for props 1-4
%     n56 = UTrimVec(7);          % common RPM for props 5-6
%     df = UTrimVec(9);           % flap angle for flaperons
% else
%     n14 = 3000;
%     n56 = n14 * sqrt(3/2);
%     df = 20;
% 
%     if FltCon.KEAS >= 50
%         n56 = 0;
%         df = 00;
%     end
% end

% Bounds
OBDFlaperon_Bounds = [-30, 120];
MBDFlaperon_Bounds = [-30, 30];
IBDFlaperon_Bounds = [-30, 30];
Elevator_Bounds = [-30, 30];
Rudder_Bounds = [-30, 30];

FlaperonRollAuthority = 30;
FlaperonYawAuthority = 30;
ElevatorAuthority = 30;
RudderAuthority = 30;
NacelleYawAuthority = 15;


% wash-in/wash-out variables
Prop14PitchStopAngle = 30;



zeta_OBDF_yaw = 1;


zeta_OBDF_roll = 1;

zeta_NAC_yaw = 1;



zeta_PROPS14_pitch = max(0, (dt - Prop14PitchStopAngle)/(90-Prop14PitchStopAngle));



zeta_PROPS56_pitch = 1;

zeta_PROPS14_roll = 1;
zeta_PROPS56_roll = 1;


Props14RollAuthority = 20;
Props56RollAuthority = 20;

Props14PitchAuthority = 20;
Props56PitchAuthority = 20;


% OUTBOARD FLAPERON ACTION
OBDF_df = max(dt, df);

df1 =   zeta_OBDF_roll * FlaperonRollAuthority * ulat - zeta_OBDF_yaw * FlaperonYawAuthority * udir + OBDF_df;
df1 = max(OBDFlaperon_Bounds(1), min(df1, OBDFlaperon_Bounds(2)));

df6 =   -zeta_OBDF_roll * FlaperonRollAuthority * ulat + zeta_OBDF_yaw * FlaperonYawAuthority * udir + OBDF_df;
df6 = max(OBDFlaperon_Bounds(1), min(df6, OBDFlaperon_Bounds(2)));

% MIDBOARD AND INBOARD FLAPERON ACTION
df2 = df + FlaperonRollAuthority * ulat; df2 = max(MBDFlaperon_Bounds(1), min(df2, MBDFlaperon_Bounds(2)));
df5 = df - FlaperonRollAuthority * ulat; df5 = max(MBDFlaperon_Bounds(1), min(df5, MBDFlaperon_Bounds(2)));

df3 = df + FlaperonRollAuthority * ulat; df3 = max(IBDFlaperon_Bounds(1), min(df3, IBDFlaperon_Bounds(2)));
df4 = df - FlaperonRollAuthority * ulat; df4 = max(IBDFlaperon_Bounds(1), min(df4, IBDFlaperon_Bounds(2)));


% ELEVATOR ACTION
de = - ElevatorAuthority * ulon;
de = max(Elevator_Bounds(1), min(de, Elevator_Bounds(2)));


% RUDDER ACTION
dr = RudderAuthority * udir;
dr = max(Rudder_Bounds(1), min(dr, Rudder_Bounds(2)));


% PROPELLER TILT ANGLES
dt1 = dt - zeta_NAC_yaw * NacelleYawAuthority * udir;
dt2 = dt;
dt3 = dt;
dt4 = dt + zeta_NAC_yaw * NacelleYawAuthority * udir;


% PROPELLER RPM
n1 = n14;
n2 = n14;
n3 = n14;
n4 = n14;
n5 = n56;
n6 = n56;


% PROPELLER BLADE PITCH
dc1 = dc14                                                            + zeta_PROPS14_roll * Props14RollAuthority * ulat;
dc2 = dc14 + zeta_PROPS14_pitch * Props14PitchAuthority * ulon        + zeta_PROPS14_roll * Props14RollAuthority * ulat;
dc3 = dc14 + zeta_PROPS14_pitch * Props14PitchAuthority * ulon        - zeta_PROPS14_roll * Props14RollAuthority * ulat;
dc4 = dc14                                                            - zeta_PROPS14_roll * Props14RollAuthority * ulat;
dc5 = dc56 - zeta_PROPS56_pitch * Props56PitchAuthority * ulon        + zeta_PROPS56_roll * Props56RollAuthority * ulat;
dc6 = dc56 - zeta_PROPS56_pitch * Props56PitchAuthority * ulon        - zeta_PROPS56_roll * Props56RollAuthority * ulat;


U = [...
    df1
    df2
    df3
    df4
    df5
    df6
    de
    de
    dr
    dt1
    dt2
    dt3
    dt4
    n1
    n2
    n3
    n4
    n5
    n6
    dc1
    dc2
    dc3
    dc4
    dc5
    dc6];

end