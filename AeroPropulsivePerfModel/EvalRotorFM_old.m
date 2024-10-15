function [F_RA,M_RA,Xrotor_dot,Summary,ElemWise,XInflow_out] = EvalRotorFM(Setup2,FlowCon,RPM,BladePitch,XInflow,Vhub_RA,Om_RA,HealthStatus,EvalMode)



if isempty(Setup2)
    global Setup;
else
    Setup = Setup2;
end


Summary = [];
ElemWise = [];

% capture the setup of the rotors
nRotor = Setup.nRotor;                                          % number of rotors
nBlades = Setup.nBlades;                                        % number of blades
SpinDir = reshape(Setup.SpinDir,[1,1,nRotor]);

% capture lift and drag characteristics of blades
% GI_Cl = Setup.GI_Cl;
% GI_Cd = Setup.GI_Cd;

% capture flight condition
rho = FlowCon.rho;                                              % kg/m3, air density
SpeedOfSound = FlowCon.SpeedOfSound;                            % m/s, speed of sound

% DISCRETIZATION SETTINGS
npsi = Setup.npsi;
nrad = Setup.nrad;
PSI = Setup.PSI;
dXBlades_RA = Setup.dXBlades_RA;
dYBlades_RA = Setup.dYBlades_RA;
dZBlades_RA = Setup.dZBlades_RA;

% CAPTURE RPM
RPM = reshape(RPM,[1,1,nRotor]);

% Convert from RPM to rad/s and account for spin direction
N = (2*pi/60)*RPM.*SpinDir;

% CAPTURE BLADE PITCH
if ~isempty(BladePitch)
    BETA = reshape(BladePitch*pi/180,[1,1,nRotor]);
else
    BETA = zeros(1,1,nRotor);
end


%% TRANSFORMATIONS BETWEEN REFERENCE FRAMES

% 1. Body Fixed (BF or bf)
% X - forward
% Y - right
% Z - down

% Rotate BF by RotorAxisTheta about 2-axis, then by RotorAxisPhi about
% 1-axis, yielding

% 2. Rotor aligned frame (RA)
% X - what results from two rotations as described above
% Y - what results from two rotations as described above
% Z - pointing downward along rotor shaft, i.e., away from rotor towards
% motor

% Rotate by azimuth angle PSI about 3-axis and by FLAPANGLE about 2-axis to
% yield (note: have not implemented FLAPANGLE yet)

% 3. Blade aligned frame (BA)
% X - radially outward along the blade
% Y - (roughly) chordwise (drag) direction
% Z - (roughly) opposite of normal (lift) direction

% 4. Inflow reference frame (for dynamic inflow model)
% Rotate RA frame about z by PSIW, the azimuthal angle of the relative wind
% X and Y - in rotor plane
% Z - coincident with Z axis of RA frame

% Matrix dimensions
% Rows: azimuthal positions
% Columns: radial positions
% Third dimension: one slice per rotor


% Separate out the u,v,w components of velocity seen at the hub of each rotor
uhub_RA = Vhub_RA(1,1,:);
vhub_RA = Vhub_RA(2,1,:);
whub_RA = Vhub_RA(3,1,:);


% Separate out the p,q,r components of velocity seen at the hub of each rotor
phub_RA = Om_RA(1,1,:);
qhub_RA = Om_RA(2,1,:);
rhub_RA = Om_RA(3,1,:);

% compute far upstream incidence and azimuth angles
% sign convention: ALPW positive when flow incident from above rotor

% normal to disk
Vinf_n = -whub_RA;
% tangential to disk
Vinf_t = sqrt(uhub_RA.^2+vhub_RA.^2);
% total velocity
%V0 = sqrt(uhub_RA.^2+vhub_RA.^2+whub_RA.^2);

%ALPW = atan2(Vinf_n,Vinf_t);
PSIW = atan2(vhub_RA,uhub_RA);

% sign convention for PSIW
% 0 - flow from straight ahead
% 90 deg - flow from right
% 180 deg - flow from behind
% 270 deg - flow from left

PSIW(isnan(PSIW))=0;
% will only occur in perfect hover

% convert hub velocities and angular velocities to inflow reference frame
Vhub_IN = [...
    uhub_RA.*cos(PSIW) + vhub_RA.*sin(PSIW);
   -uhub_RA.*sin(PSIW) + vhub_RA.*cos(PSIW);
                                          whub_RA];
 
Om_IN = [...
    phub_RA.*cos(PSIW) + qhub_RA.*sin(PSIW);
   -phub_RA.*sin(PSIW) + qhub_RA.*cos(PSIW);
                                          rhub_RA];

                                      

% PSI calculated in inflow reference frame
PSIWtemp = reshape(PSIW,[1,1,nRotor]);
PSIWtemp = repmat(PSIWtemp,[npsi,nrad,1]);


PSI_prep = pi - PSI*pi/180;
PSI_prep(PSI_prep < 0) = 2*pi + PSI_prep(PSI_prep < 0);


PSI_IN = PSI_prep + PSIWtemp;

PSI_IN(PSI_IN >= 2*pi) = PSI_IN(PSI_IN >= 2*pi) - 2*pi;
PSI_IN(PSI_IN < 0) = PSI_IN(PSI_IN < 0) + 2*pi;

% Vinf_n = V0.*sin(ALPW);    % positive when flow enters disk from top
% Vinf_t = V0.*cos(ALPW);    % always positive, by defn.

% note: the following are now "relative wind" at hub location in
% rotor-aligned axes
u_RA_rel = -Vinf_t.*cos(PSIW);
v_RA_rel = -Vinf_t.*sin(PSIW);
w_RA_rel = +Vinf_n;


% incremental velocities due to vehicle rates due to offsets of blade
% elements from rotor hubs
p_RA = Om_RA(1,1,:); q_RA = Om_RA(2,1,:); r_RA = Om_RA(3,1,:);

du_loc_RA =                             - r_RA .* dYBlades_RA      + q_RA .* dZBlades_RA;
dv_loc_RA =  + r_RA .* dXBlades_RA                                 - p_RA .* dZBlades_RA;
dw_loc_RA =  - q_RA .* dXBlades_RA      + p_RA .* dYBlades_RA;


% Compute velocities seen by each of the blade elements (x,y,z) on each of the
% rotors accounting for the rigid body motion, resolved in RA frame
u_loc_RA = u_RA_rel - du_loc_RA;
v_loc_RA = v_RA_rel - dv_loc_RA;
w_loc_RA = w_RA_rel - dw_loc_RA;

% Conversions between rotor-aligned (RA) and blade-aligned (BA) frames
% Note: rotation of PSI about axis-3 rotates RA to BA
% Suppose V_RA = {x_RA, y_RA, z_RA} and V_BA = {x_BA, y_BA, z_BA}. Then,
% x_RA = x_BA * cos(PSI) - y_BA * sin(PSI)
% y_RA = x_BA * sin(PSI) + y_BA * cos(PSI)
% z_RA = z_BA
% and
% x_BA = x_RA * cos(PSI) + y_RA * sin(PSI)
% y_BA = -x_RA * sin(PSI) + y_RA * cos(PSI)
% z_BA = z_RA

% note: for convenience, PSI is in degrees!
% u_loc_BA = u_loc_RA .* cosd(PSI) + v_loc_RA .* sind(PSI);
v_loc_BA = -u_loc_RA .* sind(PSI) + v_loc_RA .* cosd(PSI);
w_loc_BA = w_loc_RA;




%% If inflow velocity is not provided (XInflow is empty), then find it iteratively
if isempty(XInflow)
    [XInflow,~,iter,Conv] = ConvergeInflow(Vhub_IN,Om_IN,v_loc_BA,w_loc_BA,N,BETA,PSI_IN,PSIW,rho,SpeedOfSound,Setup);
    %VInflow = VInflow_ss;
else % if it is given, use it
    iter = 'NA';
    Conv = 'NA';
end



%% COMPUTE ROTOR LOADS WITH GIVEN OR CONVERGED INFLOW AND EVALUATE PITT-PETERS DYNAMIC INFLOW MODEL
[~,RotorThrust,~,dT,dL,dD,Cl,Cd,ALPHADEG,PHI,RE,F_RA,M_RA,Fx_RA,Fy_RA,Fz_RA,Mx_RA,My_RA,Mz_RA,Trq,dFx_BA,dFy_BA,dFz_BA,dMx_RA,dMy_RA,dMz_RA,Loads_PPDIM,VInflow]  = ComputeRotorLoads(v_loc_BA,w_loc_BA,N,BETA,XInflow,PSI_IN,PSIW,rho,SpeedOfSound,Setup);

% EVALUATE PITT-PETERS DYNAMNIC INFLOW MODEL
[Xrotor_dot,~] = EvalDynamicInflowModel(XInflow,Vhub_IN,Om_IN,Loads_PPDIM,N,Setup.RotorRadius);


% Prevent excessively large derivative values
Xrotor_dot(Xrotor_dot>1000) = 1000;
Xrotor_dot(Xrotor_dot<-1000) = -1000;




%% CREATE ROTOR PERFORMANCE SUMMARY (if EvalMode == 3)


if(strcmpi(EvalMode,'report'))
    N = reshape(N,[1,nRotor]);
    % Add computed quantities to summary structure
    Summary.N = (N.*HealthStatus);
    Summary.RPM = reshape(RPM,[1,nRotor]);
    Summary.MTip = abs(Summary.N)*Setup.RotorRadius/SpeedOfSound;
    
    Summary.Fx_RA = reshape(Fx_RA,[1,nRotor]);
    Summary.Fy_RA = reshape(Fy_RA,[1,nRotor]);
    Summary.Fz_RA = reshape(Fz_RA,[1,nRotor]);
    Summary.Mx_RA = reshape(Mx_RA,[1,nRotor]);
    Summary.My_RA = reshape(My_RA,[1,nRotor]);
    Summary.Mz_RA = reshape(Mz_RA,[1,nRotor]);
    
    
    Summary.RotThrust = reshape(RotorThrust,[1,nRotor]);
    Summary.TotThrust = Summary.RotThrust;
    Summary.Trq = reshape(Trq,[1,nRotor]);
    Summary.Pwr = (Summary.Trq.*Summary.N);
    Summary.VInflow = VInflow;
    Summary.XInflow = XInflow;
    Summary.Vi0 = XInflow(1,:);
    Summary.V1s = XInflow(2,:);
    Summary.V1c = XInflow(3,:);
    Summary.X = XInflow(4,:);
    Summary.S = XInflow(5,:);
    Summary.kc = XInflow(6,:);
    Summary.ks = XInflow(7,:);
    
    Summary.Vi0_dot = Xrotor_dot(1,:);
    Summary.V1s_dot = Xrotor_dot(2,:);
    Summary.V1c_dot = Xrotor_dot(3,:);
    Summary.X_dot = Xrotor_dot(4,:);
    Summary.S_dot = Xrotor_dot(5,:);
    Summary.kc_dot = Xrotor_dot(6,:);
    Summary.ks_dot = Xrotor_dot(7,:);
    
    
    Summary.lb_ft2 = (Summary.TotThrust * 0.224809)./(Setup.DiskArea * 3.28^2);
    Summary.lb_hp = (Summary.TotThrust * 0.224809)./(abs(Summary.Pwr)/745.7);
    Summary.hp_1000lb = (abs(Summary.Pwr)/745.7)./(Summary.TotThrust * 0.224809/1000);
    Summary.kW_kg = (abs(Summary.Pwr)/1000)./(Summary.TotThrust/9.81);
    
    Summary.iter = iter;
    Summary.Conv = Conv;
    
    ElemWise.dT = dT;
    ElemWise.dL = dL;
    ElemWise.dD = dD;
    ElemWise.Cl = Cl;
    ElemWise.Cd = Cd;
    ElemWise.CldCd = Cl./Cd;
    ElemWise.Cl32dCd = Cl.^(3/2)./Cd;
    ElemWise.ALPHADEG = ALPHADEG;
    ElemWise.PHI = PHI;
    ElemWise.RE = RE;
    
    ElemWise.dFx_BA = dFx_BA;
    ElemWise.dFy_BA = dFy_BA;
    ElemWise.dFz_BA = dFz_BA;
    ElemWise.dMx_RA = dMx_RA;
    ElemWise.dMy_RA = dMy_RA;
    ElemWise.dMz_RA = dMz_RA;

end


XInflow_out = XInflow;




%% LOCAL FUNCTIONS

% PITT PETERS DYNAMIC INFLOW MODEL WITH WAKE DISTORTION MODIFICATIONS
    function [Xrot_dot,Xrot_ss] = EvalDynamicInflowModel(X_inflow,Vhub_IN,Om_IN,Loads_rotor,N,radius)
        
        R = radius;        % rotor radius (m)
        Om = reshape(abs(N),[1,nRotor]);       % angular velocity (rad/s)             
        
        % rotor states
        Vi0 = X_inflow(1,:);        % uniform component of inflow velocity
        V1s = X_inflow(2,:);        % sine-varying component of inflow velocity
        V1c = X_inflow(3,:);        % cosine-varying component of inflow velocity
        X =   X_inflow(4,:);        % wake skew
        S =   X_inflow(5,:);        % wake spacing
        kc =  X_inflow(6,:);        % longitudinal wake curvature
        ks =  X_inflow(7,:);        % lateral wake curvature
       
        % operating state
        Vfwd = reshape(Vhub_IN(1,1,:),[1,nRotor]);             % forward velocity (m/s)
        VV = - reshape(Vhub_IN(3,1,:),[1,nRotor]);             % climb velocity (m/s)
        
        pndm = Om_IN(1,:)./(2*Om);     % non-dimensional roll rate --> VERIFY THIS
        qndm = Om_IN(2,:)./(2*Om);     % non-dimensional pitch rate --> VERIFY THIS
        
        CT = Loads_rotor(1,:);          % thrust coefficient
        Croll = Loads_rotor(2,:);       % rolling moment coefficient
        Cpitch = Loads_rotor(3,:);      % pitching moment coefficient
         
        % constant
        KRe = 1;            % wake curvature parameter (calibration)
        
        % calculate non-dimensional velocities
        lam0 = Vi0./(Om.*R);
        mu = Vfwd./(Om.*R);
        Vc = VV./(Om.*R);
        
        % mass flow parameter associated with mean inflow
        Vm = sqrt(mu.^2 + (lam0 + Vc).^2);
        % mass flow parameter associated with higher harmonics of inflow
        Vbar = (mu.^2 + (lam0 + Vc).*(2*lam0 + Vc))./Vm;
              
        % Time constants for rotor dynamic wake distortion model
        % Note: Verify whether these are correct.
        TX = 32./(15*pi.*Vbar);
        TS = 32./(15*pi.*Vm);
        TR = 32./(15*pi.*Vbar);
        
        %TD = diag([TX,TS,TR,TR]);
        
        % quasi-steady wake skew
        chi = atan(mu./(lam0 + Vc));
        X_qs = tan(chi/2);
        
        % quasi-steady wake spacing
        S_qs = 2*pi*Vm;
        
        % quasi-steady longitudinal wake curvature (no flapping)
        kc_qs = qndm./(lam0 + Vc);
        
        % quasi-steady lateral wake curvature (no flapping)
        ks_qs = pndm./(lam0 + Vc);
        
        % mass matrix in Pitt-Peters model
        M = diag([128/(75*pi), 16/(45*pi), 16/(45*pi)]);
        
        % Augmented L gain matrix, L = Ltilde + dL1 + dL2 + dL3
        l22 = -(5/2).*kc.*X - (3*mu/2).*ks.*(1+3.*(X.^2)/2);
        l32 = -(5/2).*ks.*X - (3*mu/2).*kc.*(1-3.*(X.^2)/2);
        
        % Column 1
        L11 = 0.5              + KRe*(0           + 0              + 0              );
        L21 =   0              + KRe*(ks/2        +0.75.*ks.*X.^2  + (5/4).*mu.*kc.*X);
        L31 = (15*pi/64).*X    + KRe*(kc/2        -0.75.*kc.*X.^2  + (5/4).*mu.*ks.*X);
        
        % Column 2
        L12 =   0              + KRe*(0           + 0              + 0              );
        L22 = 2*(1+X.^2)       + KRe*(0           + 0              + l22            );
        L32 =   0              + KRe*(0           + 0              + l32            );
        
        % Column 3
        L13 = -(15*pi/64).*X   + KRe*(0           + 0              + 0              );
        L23 =   0              + KRe*(0           + 0              - (5/2).*ks.*X   );
        L33 = 2*(1-X.^2)       + KRe*(0           + 0              - (3/10).*kc.*X  );
        
        
        % compute inverse of L matrix
        Det = L11.*L22.*L33 - L11.*L23.*L32 - L12.*L21.*L33 + L12.*L23.*L31 + L13.*L21.*L32 - L13.*L22.*L31;
        
        invL_11 = (L22.*L33 - L23.*L32)./Det;
        invL_21 = -(L21.*L33 - L23.*L31)./(Det);
        invL_31 = (L21.*L32 - L22.*L31)./(Det);
        
        invL_12 = -(L12.*L33 - L13.*L32)./(Det);
        invL_22 = (L11.*L33 - L13.*L31)./(Det);
        invL_32 = -(L11.*L32 - L12.*L31)./(Det);
        
        invL_13 = (L12.*L23 - L13.*L22)./(Det);
        invL_23 = -(L11.*L23 - L13.*L21)./(Det);
        invL_33 = (L11.*L22 - L12.*L21)./(Det);
        
        % Time derivatives of rotor inflow states, [Vio; V1s; V1c] - unit m/s2
        Vi0_dot = ((Om.^2.*R).*CT           - (  Vm.*Om).*(invL_11.*Vi0   +  invL_12.*V1s  + invL_13.*V1c))/M(1,1);
        V1s_dot = ((Om.^2.*R).*(-Croll)     - (Vbar.*Om).*(invL_21.*Vi0   +  invL_22.*V1s  + invL_23.*V1c))/M(2,2);
        V1c_dot = ((Om.^2.*R).*(-Cpitch)    - (Vbar.*Om).*(invL_31.*Vi0   +  invL_32.*V1s  + invL_33.*V1c))/M(3,3);
        
        % steady state values of rotor inflow states, [Vio_ss, V1s_ss, V1c_ss] - unit m/s2
        Vi0_ss = (L11.*(CT./Vm) + L12.*(-Croll./Vbar) + L13.*(-Cpitch./Vbar)).*Om.*R;
        V1s_ss = (L21.*(CT./Vm) + L22.*(-Croll./Vbar) + L23.*(-Cpitch./Vbar)).*Om.*R;
        V1c_ss = (L31.*(CT./Vm) + L32.*(-Croll./Vbar) + L33.*(-Cpitch./Vbar)).*Om.*R;
        
        
        
        % Time derivative of wake distortion parameters
        X_dot  = Om.*(X_qs - X)./TX;
        S_dot  = Om.*(S_qs - S)./TS;
        kc_dot = Om.*(kc_qs - kc)./TR;
        ks_dot = Om.*(ks_qs - ks)./TR;

        % time derivatives of states
        Xrot_dot = [Vi0_dot; V1s_dot; V1c_dot; X_dot; S_dot; kc_dot; ks_dot];
        
        % steady state values of states
        Xrot_ss = [Vi0_ss; V1s_ss; V1c_ss; X_qs; S_qs; kc_qs; ks_qs];
        
%         invL = [...
%             invL_11(1),invL_12(1),invL_13(1);
%             invL_21(1),invL_22(1),invL_23(1);
%             invL_31(1),invL_32(1),invL_33(1)];
%         
%         V_invL = diag([Vm(1),Vbar(1),Vbar(1)])*invL;
%         
%         Mmod = inv(V_invL)*M
        
    end

% FUNCTION TO CONVERGE INFLOW OVER THE ROTORS
    function [x_infl,dx_infl,iter,conv] = ConvergeInflow(Vhub_RA,Om_RA,v_loc_BA,w_loc_BA,N,BETA,PSI_IN,PSIW,rho,SpeedOfSound,Setup)

        % number of rotors
        nRotorLocal = length(N);
        
        % rotor radius
        radius = Setup.RotorRadius;
                
        x_infl = zeros(7,nRotorLocal);        
        x_infl(1,:) = 30;
        
        imax = 100;
        iter = 1;
        conv = 0;
        
        dv_tol = 0.1;
        
        while(conv==0&&iter<imax)
        
        [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,loads_PPDIM,~]  = ComputeRotorLoads(v_loc_BA,w_loc_BA,N,BETA,x_infl,PSI_IN,PSIW,rho,SpeedOfSound,Setup);
   
        [~,Xr_ss] = EvalDynamicInflowModel(x_infl,Vhub_RA,Om_RA,loads_PPDIM,N,radius);
        x_infl_old = x_infl;
        x_infl_new = Xr_ss;
        
        dx_infl = x_infl_new - x_infl_old;
        dv_infl = dx_infl(1:3,:);
        
        s = abs(dv_infl)<dv_tol;
        conv = all(s,[1,2]);
        
        x_infl = x_infl_old + .4*(x_infl_new - x_infl_old);
  
        iter = iter+1;
        end
        
    end

% FUNCTION TO COMPUTE ROTOR THRUST FOR A GIVEN INFLOW
    function [netThrust,rotorThrust,ductThrust,dT,dL,dD,Cl,Cd,ALPHADEG,phi,RE,v_in] = ComputeThrust(v_loc_BA,w_loc_BA,N,beta,x_inflow,PSI_IN,rho,SpeedOfSound,Setup)
        
        nRotorLocal = length(N);            % count the number of entries of N that have been passed down
        
        
        vi0 = reshape(x_inflow(1,:),[1,1,nRotorLocal]);
        v1s = reshape(x_inflow(2,:),[1,1,nRotorLocal]);
        v1c = reshape(x_inflow(3,:),[1,1,nRotorLocal]);
        
        %PSI_IN*180/pi
        
        v_in = vi0 + (Setup.R/Setup.RotorRadius).*(v1s.*sin(PSI_IN) + v1c.*cos(PSI_IN));
        
        % Compute the tangential velocity (U_T, along BA(2)) and the normal velocity
        % (U_P, along BA(3)) seen by the blade elements
        vt = zeros(size(Setup.R));
        
        U_T_ba = abs(N).*Setup.R(:,:,1:nRotorLocal) - (sign(N)).* v_loc_BA - vt;
        U_P_ba = w_loc_BA + v_in;
        
        % total sectional velocity
        U_tot = sqrt(U_T_ba.^2+U_P_ba.^2);
        
        % Mach numbers and Prandl-Glauert correction factor
        Mach = U_tot/SpeedOfSound;
        MachEff = min(Mach,0.9);
        
        B = 1./sqrt(1-MachEff.^2);
        if imag(B)~=0
            warning('imaginary B')
            B = 1;
        end
        
        % Prandtl tip loss function
        %lam = v_inflow./((abs(N))*Setup.RotorRadius)
        lam = (Setup.R/Setup.RotorRadius).*abs(U_P_ba./U_T_ba);
        %lam = v_inflow./((abs(N))*Setup.RotorRadius);
        f = (Setup.nBlades/2).*(1-Setup.R/Setup.RotorRadius)./lam;
        F = (2/pi)*acos(exp(-f));
        %F = ones(1,1,nRotorLocal);
        % calculate the angle PHI
        phi = atan(U_P_ba./U_T_ba);
        
        % calculate effective sectional angles of attack
        ALPHA = beta + Setup.Pitch(:,:,1:nRotorLocal) - phi;
        ALPHADEG = ALPHA*180/pi;
        
        % Query sectional lift and drag coefficients
        RE = FlowCon.rho.*U_tot.*Setup.Chord(:,:,1:nRotorLocal)./FlowCon.DynVisc;
        
        %         Cl = GI_Cl(ALPHADEG,RE); %B*
        %         Cd = GI_Cd(ALPHADEG,RE);
        
        
        CL0 = Setup.PropDef.CL0;
        CL_a = Setup.PropDef.CL_a;
        CL_min = Setup.PropDef.CL_min;
        CL_max = Setup.PropDef.CL_max;
        CD0 = Setup.PropDef.CD0;
        CLCD0 = Setup.PropDef.CLCD0;
        REexp = Setup.PropDef.REexp;
        REref = Setup.PropDef.REref;
        CD2u = Setup.PropDef.CD2u;
        CD2l = Setup.PropDef.CD2l;
        
        Cl = (CL0 + CL_a.*ALPHA).*B;
        Cl =  max(CL_min,min(CL_max,Cl));
        
        Clmaxclipped = Cl>=CL_max;
        Clminclipped = Cl<=CL_min;
        DragAddFlag = Clmaxclipped + Clminclipped;
        
        CD2 = CD2u* ones(size(Cl));
        CD2(Cl<CLCD0) = CD2l;
        
        aCD0 = (CLCD0-CL0)./CL_a;
        dCd_stall_mag = 2 * (sin(ALPHA - aCD0)).^2;
        dCd_stall = dCd_stall_mag.*DragAddFlag;
        
        Cd = (CD0 + CD2 .* (Cl-CLCD0).^2).*(RE./REref).^(REexp) + dCd_stall;
        
        % Compute element-wise lift and drag forces
        dL = (0.5*rho*U_tot.^2) .* Setup.Chord(:,:,1:nRotorLocal) .* Setup.dR .* Cl.*F(:,:,1:nRotorLocal);
        dD = (0.5*rho*U_tot.^2) .* Setup.Chord(:,:,1:nRotorLocal) .* Setup.dR .* Cd;
        
        % Compute the element-wise thrust
        Thrust = dL.*cos(phi) - dD.*sin(phi);
        dT =  Thrust;
        
        % Compute radially-summed and azimuthally-averaged thrust
        rotorThrust = Setup.nBlades * sum(sum(Thrust))/(Setup.npsi);
        
        % Compute duct thrust
        ductThrust = 0;
        
        % Compute net thrust
        netThrust = rotorThrust + ductThrust;
        
    end

% FUNCTION TO COMPUTE ROTOR LOADS FOR A GIVEN INFLOW
    function [NetThrust,RotorThrust,DuctThrust,dT,dL,dD,Cl,Cd,ALPHADEG,PHI,RE,F_RA,M_RA,Fx_RA,Fy_RA,Fz_RA,Mx_RA,My_RA,Mz_RA,Trq,dFx_BA,dFy_BA,dFz_BA,dMx_RA,dMy_RA,dMz_RA,Loads_PPDIM,v_in]  = ComputeRotorLoads(v_loc_BA,w_loc_BA,N,BETA,XInflow,PSI_IN,PSIW,rho,SpeedOfSound,Setup)
        
        DiscArea = Setup.DiskArea;
        RotorRadius = Setup.RotorRadius;
          
        % Either with the given inflow or the computed inflow, compute thrust:
        [NetThrust,RotorThrust,DuctThrust,dT,dL,dD,Cl,Cd,ALPHADEG,PHI,RE,v_in] = ComputeThrust(v_loc_BA,w_loc_BA,N,BETA,XInflow,PSI_IN,rho,SpeedOfSound,Setup);
        
        % ELEMENT WISE FORCES IN BLADE-ALIGNED (BA) FRAME
        % in blade radial directions (x)
        dFx_BA = zeros(npsi,nrad,nRotor);
        % in blade tangential directions (y)
        % note, account for the direction of rotation!
        % if cw-rotating rotor, positive y-axis is in direction of blade tangential
        % velocity. If ccw-rotating rotor, positive y-axis is opposite to direction
        % of blade tangential velocity.
        
        dFy_BA = -sign(N).*(dL.*sin(PHI) + dD.*cos(PHI));
        % in blade normal direction (z)
        dFz_BA = -(dL.*cos(PHI) - dD.*sin(PHI));
        
        % ELEMENT WISE FORCES in ROTOR-ALIGNED (RA) FRAME
        % x_RA = x_BA * cos(PSI) - y_BA * sin(PSI)
        % y_RA = x_BA * sin(PSI) + y_BA * cos(PSI)
        % z_RA = z_BA
        % note: for convenience, psi is in degrees!
        dFx_RA = dFx_BA.*cosd(PSI) - dFy_BA.*sind(PSI);
        dFy_RA = dFx_BA.*sind(PSI) + dFy_BA.*cosd(PSI);
        dFz_RA = dFz_BA;
        
        % moments generated on each element w.r.t. center of rotor and expressed in
        % RA frame
        dMx_RA = dYBlades_RA.*dFz_RA - dZBlades_RA.*dFy_RA;
        dMy_RA = dZBlades_RA.*dFx_RA - dXBlades_RA.*dFz_RA;
        dMz_RA = dXBlades_RA.*dFy_RA - dYBlades_RA.*dFx_RA;
        
        % Vane and duct forces and moments
        Fxvane_RA = 0;
        Fyvane_RA = 0;
        Fzvane_RA = 0;
        Mxvane_RA = 0;
        Myvane_RA = 0;
        Mzvane_RA = 0;
        
        Fx_RA_Duct = 0;
        Fy_RA_Duct = 0;
        Fz_RA_Duct = 0;
        Mx_RA_Duct = 0;
        My_RA_Duct = 0;
        Mz_RA_Duct = 0;
        
        
        % Note: the above matrices contain elemental forces and moments for ONE
        % blade in MULTIPLE azimuthal positions. To get the net forces and moments,
        % sum ALL elements of these matrices, MULTIPLY by number of blades, and
        % DIVIDE by number of azimuthal positions
        
        % note: add corresponding forces and moments generated due to duct
        Fx_RA = nBlades * sum(sum(dFx_RA))/npsi  + Fx_RA_Duct + Fxvane_RA;
        Fy_RA = nBlades * sum(sum(dFy_RA))/npsi  + Fy_RA_Duct + Fyvane_RA;
        Fz_RA = nBlades * sum(sum(dFz_RA))/npsi  + Fz_RA_Duct + Fzvane_RA;
        Mx_RA = nBlades * sum(sum(dMx_RA))/npsi  + Mx_RA_Duct + Mxvane_RA;
        My_RA = nBlades * sum(sum(dMy_RA))/npsi  + My_RA_Duct + Myvane_RA;
        Mz_RA = nBlades * sum(sum(dMz_RA))/npsi  + Mz_RA_Duct + Mzvane_RA;
        
        F_RA = reshape([Fx_RA;Fy_RA;Fz_RA],[3,nRotor,1]);
        M_RA = reshape([Mx_RA;My_RA;Mz_RA],[3,nRotor,1]);
        
        % compute shaft torque
        Trq = Mz_RA;
        
        
        
        % compute non-dimensional loads for Pitt Peters dynamic inflow
        % model
        % 1. CT (thrust coefficient)
        % 2. Croll (rolling moment coefficient, inflow frame)
        % 3. Cpitch (pitching moment coefficient, inflow frame)

        
        % CHECK THIS AREA
        
        Mx_IN = Mx_RA.*cos(PSIW) + My_RA.*sin(PSIW);
        My_IN = -Mx_RA.*sin(PSIW) + My_RA.*cos(PSIW);
        
        
        
%         Roll = Mx_IN(1)
%         Pitch = My_IN(1)
        
        
        
        Loads_PPDIM(1,:) = -reshape(Fz_RA./(rho.*DiscArea.*(N.*RotorRadius).^2),[1,nRotor]);  % thrust coefficient
        Loads_PPDIM(2,:) = reshape(Mx_IN./(rho.*DiscArea.*RotorRadius.*(N.*RotorRadius).^2),[1,nRotor]); % rolling moment coefficient
        Loads_PPDIM(3,:) = reshape(My_IN./(rho.*DiscArea.*RotorRadius.*(N.*RotorRadius).^2),[1,nRotor]); % rolling moment coefficient
        
        
    end




end




% LOADS GENERATED ON VANES

%
%
% if isempty(VaneAngles)
%     VaneAngles = zeros(1,nRotor);
% end
%
%
% % Setup.Vane.GI_Cl = NACA0012.GI_Cl;
% % Setup.Vane.GI_Cd = NACA0012.GI_Cd;
% % Setup.Vane.Span = 40/100;
% % Setup.Vane.Chord = 7/100;
% % Setup.Vane.Area = Setup.Vane.Span * Setp.Vane.Chord;
%
% dvane = VaneAngles*pi/180;
% dvane = reshape(dvane,[1,1,nRotor]);
%
% % Vane loads generation
% uvane_RA = V0.*cos(ALP_I).*cos(PSIW);
% vvane_RA = V0.*cos(ALP_I).*sin(PSIW);
% wvane_RA = V0.*sin(ALP_I) + VInflow;
%
% Vtot_vane = sqrt(uvane_RA.^2+vvane_RA.^2+wvane_RA.^2);
% qbar_vane = 0.5.*rho.*Vtot_vane.^2;
%
% % longitudinally aligned vanes
% uvane_VA = uvane_RA;
% vvane_VA = vvane_RA.*cos(dvane) + wvane_RA.*sin(dvane);
% wvane_VA = -vvane_RA.*sin(dvane) + wvane_RA.*cos(dvane);
%
% AOAvane = atan(vvane_VA./wvane_VA);
%
% Clvane = Setup.Vane.GI_Cl(AOAvane*180/pi).*Setup.Vane.XVanesPresent;
% Cdvane = Setup.Vane.GI_Cd(AOAvane*180/pi).*Setup.Vane.XVanesPresent;
%
% Lvane = qbar_vane.*Clvane.*Setup.Vane.Area;
% Dvane = qbar_vane.*Cdvane.*Setup.Vane.Area;
%
% Fxvane_VA = 0;
% Fyvane_VA = Lvane.*cos(AOAvane) + Dvane.*sin(AOAvane);
% Fzvane_VA = - Lvane.*sin(AOAvane) + Dvane.*cos(AOAvane);
%
% Fxvane_RA = Fxvane_VA;
% Fyvane_RA = Fyvane_VA.*cos(dvane) - Fzvane_VA.*sin(dvane);
% Fzvane_RA = Fyvane_VA.*sin(dvane) + Fzvane_VA.*cos(dvane);
%
% Mxvane_RA = -Setup.Vane.ZOffset.*Fyvane_RA;
% Myvane_RA = +Setup.Vane.ZOffset.*Fxvane_RA;
