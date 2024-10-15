function [dNdt,Hr,F,M,PkW,TR] = CruisePropDynamics(dt,Vehicle,CPRPM,CPBP,TAS,AOADeg,rho)

% grab cruise prop diameter
PropDiam = Vehicle.Propulsion.CruisePropDiam_m;

% calculate rev/s
n = CPRPM/60;
% calculate advance ratio
J = TAS.*cosd(AOADeg)./(n*PropDiam);
% calculate thrust coefficient
% CT = Vehicle.Propulsion.CPSetup.PropCurves.GI_CT_J_beta(J,CPBladePitch);
CT = Vehicle.Propulsion.CPSetup.PropCurves.GI_CT_rpm(CPRPM);

% calculate torque coefficient
% CQ = Vehicle.Propulsion.CPSetup.PropCurves.GI_CQ_J_beta(J,CPBladePitch);
CQ = Vehicle.Propulsion.CPSetup.PropCurves.GI_CQ_rpm(CPRPM);

% calculate propeller efficiency
% eta = Vehicle.Propulsion.CruiseProp.GI_eta_J_beta(J,CPBladePitch);

% calculate dimensional thrust (N)
T_N =  CT .* (rho .* n.^2 * PropDiam^4);

% calculate dimensional torque (Nm)
Q_Nm =  CQ .* (rho .* n.^2 * PropDiam^5);


CruisePropTorque_Nm = Q_Nm;

% net cruise prop force and moment
F = [T_N;0;0];


% propeller location
r_prop = Vehicle.Propulsion.CPSetup.HubLocation;


M = ...
    [0,          -r_prop(3),       r_prop(2);
    r_prop(3),        0,          -r_prop(1);
    -r_prop(2),   r_prop(1),  0]...
    * F + [-Q_Nm;0;0];       



% cruise prop angular momentum
PropSpeed_rads = n*2*pi;
Hr = [Vehicle.Propulsion.I_cruiseprop * PropSpeed_rads; 0; 0];

% cruise propeller angular acceleration
CruisePropDrivePower_W = Vehicle.Propulsion.CruisePropPower_kW * 1000 * dt/0.7;  %Props were picked in the green zone. The green zone means 0.5-0.8 throttle.
CruisePropDriveTorque_Nm = CruisePropDrivePower_W / PropSpeed_rads;
PkW = CruisePropDrivePower_W/1000;

PropAcc_radss = (CruisePropDriveTorque_Nm - CruisePropTorque_Nm)/Vehicle.Propulsion.I_cruiseprop;

TR = CruisePropDriveTorque_Nm./CruisePropTorque_Nm;

dNdt = PropAcc_radss;

end