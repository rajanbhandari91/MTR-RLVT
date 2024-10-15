function Setup = TestProp(Vehicle,Setup)
SetupTemp = Setup;
SetupTemp.SpinDir = 1 .* ones(1,Setup.nRotor);
ugrid = [0.01:5:1.2*(Vehicle.Operations.VC*0.5144)];
dRPM = -0;

SetupTemp.ThrustScaler = 1;
SetupTemp.TorqueScaler = 1;

NProps = SetupTemp.nRotor;


for j = 1:length(ugrid)

    uw = ugrid(j) ;
    vw = 0;
    ww = 0;
    pw = 0;
    qw = 0;
    rw = 0;

    % temporary hardcodes
    % XInflow = [];
    [~,a,~,rho] = atmosisa(6000 / 3.28);

    FlowCon.rho = rho;
    FlowCon.SpeedOfSound = a;
    FlowCon.DynVisc = 0.17000E-04;

    % Note 1: th_ra = 0 when rotor axis points "up"
    % Note 2: th_ra = -90 when rotor axis points "forward
    % Note 3: dW wing angles are = 0 in fwd position, = 90 in vertical posn
    % dth = dW;

    th_RA = -90 .* ones(1,SetupTemp.nRotor);
    ph_RA = SetupTemp.RotorAxisPhi;


    % propulsor hub locations
    xloc = SetupTemp.RotorLoc(1,:);

    yloc = SetupTemp.RotorLoc(2,:);

    zloc = SetupTemp.RotorLoc(3,:);

    HealthStatus = SetupTemp.HealthStatus;
    InactiveRotors = HealthStatus==0;

    % Find the indices of individual rotors that fall below threshold RPM
    % (if any). Set the RPM of these to the threshold RPM for running the
    % BEMT code. Then, set the computed forces and moments for these rotors
    % to zero after the fact
    % InactiveRotors = PropRPMs<=PropCutoffRPM;
    PropRPMs(InactiveRotors==1) = 300;

    % compute velocities seen at the rotor hubs, resolved in body-fixed axes
    uhub_bf = uw  +  0.*xloc   - rw.*yloc   +  qw.*zloc;
    vhub_bf = vw  + rw.*xloc   +  0.*yloc   -  pw.*zloc;
    whub_bf = ww  - qw.*xloc   + pw.*yloc   +   0.*zloc;

    % convert these velocities from body-fixed axes to rotor aligned axes
    u_RA =              cosd(th_RA).*uhub_bf                                          - sind(th_RA).*whub_bf;
    v_RA = sind(th_RA).*sind(ph_RA).*uhub_bf  + cosd(ph_RA).*vhub_bf     + cosd(th_RA).*sind(ph_RA).*whub_bf;
    w_RA = sind(th_RA).*cosd(ph_RA).*uhub_bf  - sind(ph_RA).*vhub_bf     + cosd(th_RA).*cosd(ph_RA).*whub_bf;
    Vhub_RA = reshape([u_RA;v_RA;w_RA],[3,1,SetupTemp.nRotor]);

    % convert vehicle angular velocity from body-fixed axes to rotor aligned axes
    p_RA =              cosd(th_RA).*pw                                  - sind(th_RA).*rw;
    q_RA = sind(th_RA).*sind(ph_RA).*pw   + cosd(ph_RA).*qw + cosd(th_RA).*sind(ph_RA).*rw;
    r_RA = sind(th_RA).*cosd(ph_RA).*pw   - sind(ph_RA).*qw + cosd(th_RA).*cosd(ph_RA).*rw;
    Om_RA = [p_RA;q_RA;r_RA];
    Om_RA = repmat(Om_RA,[1,1,SetupTemp.nRotor]);


    % Get forces and moments at the rotor hubs, resolved in rotor axes (RA)
    SetupTemp.InflowGuess = 5;
    % SetupTemp.InflowGuess(uw > 10) = 5;

    XInflow_Props = [];

    % range of blade-pitch
    BPgrid = -2:1:SetupTemp.DesignCases.BP75R(1)+5;

    for i = 1:length(BPgrid)
        % Blade Pitch
        BladePitch = BPgrid(i).*ones(NProps, 1) ;

        % Blade RPMs
        RPM_Max = max(Setup.DesignCases.RPM);    %(sqrt((0.51 * FlowCon.SpeedOfSound).^2 - 1*uw^2) / (Setup.RotorRadius)) * 60/(2*pi);

        PropRPMs = RPM_Max.*ones(NProps, 1); %(SetupTemp.DesignCases.RPM(1) + dRPM).*ones(NProps, 1);

        % PropRPMs(PropRPMs > RPM_Max) = RPM_Max;        

        EvalMode = 2;
        [Fhub_RA,Mhub_RA,Xrotor_dot_Props,Summary,ElemWise,XInflow_Props_out] = EvalRotorFM(SetupTemp,FlowCon,PropRPMs',BladePitch,XInflow_Props,Vhub_RA,Om_RA,HealthStatus,EvalMode);

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
        % PropTorques = Mzhub_RA; % abs(Mzhub_RA);

        % CT
        CT_beta(i,:) = -Fzhub_RA(1)/( FlowCon.rho * (PropRPMs(1)/60)^2 * SetupTemp.RotorDiam_m^4);
        Thrust(i,:) = -Fzhub_RA(1);

        % Adv Ratio
        J_beta(i,:) = uw/((PropRPMs(1)/60)*SetupTemp.RotorDiam_m);

        % Angular momentum of spinning rotors
        N = SetupTemp.SpinDir.*PropRPMs'*2*pi/60;
    end

    CT_u(:,j)  = CT_beta;
    Thrust_u(:,j) = Thrust;
    J_u(:,j)   = J_beta;  
end


BetaGrid = repmat(BPgrid',1,length(ugrid));
GI_CT_beta_J = griddedInterpolant(BetaGrid, J_u, CT_u, 'linear', 'nearest');
GI_T_beta_J = griddedInterpolant(BetaGrid, J_u, Thrust_u, 'linear', 'nearest');

% reverse query and find beta as a function of J and CQ 'GI_beta_J_CQ'
% Vehicle.Propulsion.PropCurves.GI_beta_J_CQ

nData = 20;

Jgrid  = linspace(J_u(1), J_u(end), nData); 
CTgrid = linspace(0,max(max(CT_u)), nData); 
Tgrid = linspace(0,max(max(Thrust_u)), nData);

for ii = 1:nData

    CT_temp      = GI_CT_beta_J(BPgrid', Jgrid(ii).*ones(length(BPgrid),1));
    T_temp      = GI_T_beta_J(BPgrid', Jgrid(ii).*ones(length(BPgrid),1));
    Beta_temp    = interp1(CT_temp, BPgrid', CTgrid', 'linear');
    Beta_temp    = interp1(T_temp, BPgrid', Tgrid', 'linear');

    % find if any NaN
    not_nan = find(~isnan(Beta_temp));  

    % if first is not nan but aft segment has any nan replace the nan
    % segment with the last converged value
    if ~isempty(not_nan)
        if not_nan(1) == 1
            Beta_temp(not_nan(end)+1:end) = Beta_temp(not_nan(end));
        else
            % also replace first nan segment with first converged value
            Beta_temp(1:not_nan(1)-1)     = Beta_temp(not_nan(1));
            Beta_temp(not_nan(end)+1:end) = Beta_temp(not_nan(end));
        end
    else % if any case where, entire column is NaN, stop!
        ii = ii - 1;
        break;
    end

    Beta_MAT(:,ii) =  Beta_temp;
end


[CT_MAT,J_MAT] = ndgrid(CTgrid, Jgrid(1:ii));
Setup.PropCurves.GI_beta_CT_J = griddedInterpolant(CT_MAT, J_MAT, Beta_MAT, 'linear', 'linear');
[T_MAT,J_MAT] = ndgrid(Tgrid, Jgrid(1:ii));
Setup.PropCurves.GI_beta_T_J = griddedInterpolant(T_MAT, J_MAT, Beta_MAT, 'linear', 'linear');
% Setup.PropCurves.Thrust = Thrust_u;
% Setup.PropCurves.Pitch = BPgrid';
stopper = 1;