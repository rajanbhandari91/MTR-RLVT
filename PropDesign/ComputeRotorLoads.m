function [F_RP_bf, M_RP_bf, Hr_bf, Pwr_kW, XInflow_out, XInflow_dot, InactiveRotors, ConvFlag, Summary, ElemWise] = ComputeRotorLoads(Setup,FlowCon, xloc, yloc, zloc, th_RA, ph_RA, Vb, Om, PropRPMs, BladePitch,  XInflow, EvalMode, CutoffRPM)



% F_RP_bf = zeros(3,1);
% M_RP_bf = zeros(3,1);
% Hr_bf = [0;0;0];

% nRotor = Setup.nRotor;

HealthStatus = Setup.HealthStatus;

% sines and cosines of rotor axis inclinations
cosd_th_RA = cosd(th_RA);
sind_th_RA = sind(th_RA);
 
cosd_ph_RA = cosd(ph_RA);
sind_ph_RA = sind(ph_RA);


InactiveRotors = HealthStatus==0;
InactiveRotors(PropRPMs<CutoffRPM)=1;


% Find the indices of individual rotors that fall below threshold RPM
% (if any). Set the RPM of these to the threshold RPM for running the
% BEMT code. Then, set the computed forces and moments for these rotors
% to zero after the fact
% InactiveRotors = PropRPMs<=PropCutoffRPM;
PropRPMs(InactiveRotors==1) = CutoffRPM;




uw = Vb(1);
vw = Vb(2);
ww = Vb(3);
pw = Om(1);
qw = Om(2);
rw = Om(3);

% compute velocities seen at the rotor hubs, resolved in body-fixed axes
uhub_bf = uw  +  0.*xloc   - rw.*yloc   +  qw.*zloc;
vhub_bf = vw  + rw.*xloc   +  0.*yloc   -  pw.*zloc;
whub_bf = ww  - qw.*xloc   + pw.*yloc   +   0.*zloc;

% convert these velocities from body-fixed axes to rotor aligned axes
u_RA =             cosd_th_RA.*uhub_bf                                        - sind_th_RA.*whub_bf;
v_RA = sind_th_RA.*sind_ph_RA.*uhub_bf  + cosd_ph_RA.*vhub_bf     + cosd_th_RA.*sind_ph_RA.*whub_bf;
w_RA = sind_th_RA.*cosd_ph_RA.*uhub_bf  - sind_ph_RA.*vhub_bf     + cosd_th_RA.*cosd_ph_RA.*whub_bf;
Vhub_RA = reshape([u_RA;v_RA;w_RA],[3,1,Setup.nRotor]);

% convert vehicle angular velocity from body-fixed axes to rotor aligned axes
p_RA =             cosd_th_RA.*pw                                - sind_th_RA.*rw;
q_RA = sind_th_RA.*sind_ph_RA.*pw   + cosd_ph_RA.*qw + cosd_th_RA.*sind_ph_RA.*rw;
r_RA = sind_th_RA.*cosd_ph_RA.*pw   - sind_ph_RA.*qw + cosd_th_RA.*cosd_ph_RA.*rw;
Om_RA = [p_RA;q_RA;r_RA];
Om_RA = repmat(Om_RA,[1,1,Setup.nRotor]);

% 
% % inflow guess
% Setup.InflowGuess = 8;
% 


% Get forces and moments at the rotor hubs, resolved in rotor axes (RA)
[Fhub_RA,Mhub_RA,XInflow_dot,Summary,ElemWise,XInflow] = EvalRotorFM(Setup,FlowCon,PropRPMs,BladePitch,XInflow,Vhub_RA,Om_RA,HealthStatus,EvalMode);

XInflow_dot(InactiveRotors,:) = -XInflow(InactiveRotors,:);


ConvFlag = -1;
if~isempty(Summary)
    ConvFlag = Summary.Conv;
end

% Xrotor_dot_Props(PropRPMs<=PropCutoffRPM) = -XInflow_Props;


% % Inflow velocity at rotors
w = max(0,XInflow(1,:));
w(InactiveRotors==1)  = 0 ;
%vi = mean(w);

Fxhub_RA = Fhub_RA(1,:); Fyhub_RA = Fhub_RA(2,:); Fzhub_RA = Fhub_RA(3,:);
Mxhub_RA = Mhub_RA(1,:); Myhub_RA = Mhub_RA(2,:); Mzhub_RA = Mhub_RA(3,:);

% zero out forces and moments of inactive rotors
Fxhub_RA(InactiveRotors) = 0;
Fyhub_RA(InactiveRotors) = 0;
Fzhub_RA(InactiveRotors) = 0;
Mxhub_RA(InactiveRotors) = 0;
Myhub_RA(InactiveRotors) = 0;
Mzhub_RA(InactiveRotors) = 0;

% aerodynamic drag torque on rotors
PropTorques = abs(Mzhub_RA);

% Convert hub forces and moments to body-fixed axes
Fxhub_bf =  cosd_th_RA.*Fxhub_RA  +  sind_th_RA.*sind_ph_RA.*Fyhub_RA  + sind_th_RA.*cosd_ph_RA.*Fzhub_RA;
Fyhub_bf =                                       + cosd_ph_RA.*Fyhub_RA               - sind_ph_RA.*Fzhub_RA;
Fzhub_bf = -sind_th_RA.*Fxhub_RA  +  cosd_th_RA.*sind_ph_RA.*Fyhub_RA  + cosd_th_RA.*cosd_ph_RA.*Fzhub_RA;

Mxhub_bf =  cosd_th_RA.*Mxhub_RA  +  sind_th_RA.*sind_ph_RA.*Myhub_RA  + sind_th_RA.*cosd_ph_RA.*Mzhub_RA;
Myhub_bf =                                       + cosd_ph_RA.*Myhub_RA               - sind_ph_RA.*Mzhub_RA;
Mzhub_bf = -sind_th_RA.*Mxhub_RA  +  cosd_th_RA.*sind_ph_RA.*Myhub_RA  + cosd_th_RA.*cosd_ph_RA.*Mzhub_RA;

% compute the rotor forces and moments with respect to vehicle reference
% point (RP)
FxRP_bf = Fxhub_bf; FyRP_bf = Fyhub_bf; FzRP_bf = Fzhub_bf;

MxRP_bf = Mxhub_bf                    + yloc.*Fzhub_bf   - zloc.*Fyhub_bf;
MyRP_bf = Myhub_bf  - xloc.*Fzhub_bf                     + zloc.*Fxhub_bf;
MzRP_bf = Mzhub_bf  + xloc.*Fyhub_bf  - yloc.*Fxhub_bf;

% output forces and moments about reference point, resolved in body axes
F_RP_bf = [FxRP_bf; FyRP_bf; FzRP_bf];
M_RP_bf = [MxRP_bf; MyRP_bf; MzRP_bf];

% Angular momentum of spinning rotors
N = Setup.SpinDir.*PropRPMs'*2*pi/60;
Hr_RA = Setup.Inertia_kgm2 * N;

Pwr_kW = (abs(N.*PropTorques))/1000;

% Convert angular momentum and its derivative to body-fixed axes
Hrx_bf =  + sind_th_RA.*cosd_ph_RA.*Hr_RA;
Hry_bf =  - sind_ph_RA.*Hr_RA;
Hrz_bf =  + cosd_th_RA.*cosd_ph_RA.*Hr_RA;
% sum these up
Hr_bf = [sum(Hrx_bf);sum(Hry_bf);sum(Hrz_bf)];

% % reshape XInflow_out if being evaluated directly from MATLAB
% if EvalMode~=1
%     XInflow_out = reshape(XInflow,[1,7*nRotor]);
% end

XInflow_out = XInflow;