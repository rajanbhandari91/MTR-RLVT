function [LinModel] = LinearizeModel(Init,Veh,longcon,latcon,Ref,varargin)

% default case, evaluate both A and B matrices
Eval_AMatrix = 1;
Eval_BMatrix = 1;

if ~isempty(varargin)
    Eval_AMatrix = varargin{1}==1;
    Eval_BMatrix = varargin{1}==2;
end



% long variables: 1, 3, 5, 7, 9, 11
% lat variables: 2, 4, 6, 8, 10, 12
longvar = [1, 3, 5, 7, 9, 11];
latvar = [2, 4, 6, 8, 10, 12];



dx = 0.01;
du = 0.01;


% PRIMARY OPTION
% 1. u
% 2. v
% 3. w
% 4. p
% 5. q
% 6. r
% 7. x
% 8. y
% 9. z
% 10. ph
% 11. th
% 12. ps



% for non-dimensionalizing A-matrices
V = Ref.Velocity;
c = Ref.RefLongLength;

% longitudinal
% u, w, q, x, z, th
NumLong = [V,V,2*V/c,c/2,c/2,1];
Tlong(1,:) = NumLong/(2*V^2/c);         % u-dot
Tlong(2,:) = NumLong/(2*V^2/c);         % w-dot
Tlong(3,:) = NumLong/(4*V^2/c^2);       % q-dot
Tlong(4,:) = NumLong/(V);               % x-dot
Tlong(5,:) = NumLong/(V);               % z-dot
Tlong(6,:) = NumLong/(2*V/c);           % th-dot

% lateral
c = Ref.RefLatLength;
% v, p, r, y, ph, ps
NumLat = [V,2*V/c,2*V/c,c/2,1,1];
Tlat(1,:) = NumLat/(2*V^2/c);         % v-dot
Tlat(2,:) = NumLat/(4*V^2/c^2);       % p-dot
Tlat(3,:) = NumLat/(4*V^2/c^2);       % r-dot
Tlat(4,:) = NumLat/(V);               % y-dot
Tlat(5,:) = NumLat/(2*V/c);           % ph-dot
Tlat(6,:) = NumLat/(2*V/c);           % ps-dot



X0 = Init.X;
U0 = Init.U;

% AOA = atan2(X0(3),X0(1));
% % BETA = asin(X0(2)/sqrt(X0(1)^2+X0(2)^2+X0(3)^2));
% % L_BS = [cos(AOA), 0, -sin(AOA); 0, 1, 0; sin(AOA), 0, cos(AOA)] * [cos(BETA), -sin(BETA), 0; sin(BETA), cos(BETA), 0; 0, 0, 1];

% % Conversion of states from Body to Stability Axes
% u = X0(1);
% v = X0(2);
% w = X0(3);
% p = X0(4);
% q = X0(5);
% r = X0(6);
% x = X0(7);
% y = X0(8);
% z = X0(9);
% ph = X0(10);
% th = X0(11);
% ps = X0(12);

AOA = atan2(X0(3),X0(1));

% [u v w]'
X0(1:3) = ConvertBAtoSAorSAtoBA(X0(1:3), [], AOA);

% [p q r]'
X0(4:6) = ConvertBAtoSAorSAtoBA(X0(4:6), [], AOA);


if Eval_AMatrix

    for i = 1:1:length(X0)
        X = X0;
        U = U0;

        % perturb i-th element "high"
        X(i) = X0(i) + dx;
        [Xdot_high] = CalcXdotRes(X,U,Veh,AOA);

        % perturb i-th element "low"
        X(i) = X0(i) - dx;
        [Xdot_low] = CalcXdotRes(X,U,Veh,AOA);

        A(:,i) = (Xdot_high - Xdot_low)/(2*dx);

    end

    % A matrix - append to struct
    Along = A(longvar,longvar);
    Alat = A(latvar,latvar);

    LinModel.Along = Along;
    LinModel.Alongnd = Along.*Tlong;

    LinModel.Alat = Alat;
    LinModel.Alatnd = Alat.*Tlat;

end


if Eval_BMatrix

    % perturb each control EFFECTOR; populate B-matrix elements
    for i = 1:1:length(U0)

        X = X0;
        U = U0;

        U(i) = U0(i)+du;
        [Xdot_high] = CalcXdotRes(X,U,Veh,AOA);

        U(i) = U0(i)-du;
        [Xdot_low] = CalcXdotRes(X,U,Veh,AOA);

        B(:,i) = (Xdot_high-Xdot_low)/(2*du);

    end


    % append B-matrix
    LinModel.B = B;
    LinModel.Blong = B(longvar,:);
    LinModel.Blat = B(latvar,:);

    % if controls are defined as being longitudinal or lateral, partition out
    % the B-matrix accordingly
    if ~isempty(longcon)
        Blong = B(longvar,longcon);
        LinModel.Blong = Blong;
    end
    if ~isempty(latcon)
        Blat = B(latvar,latcon);
        LinModel.Blat = Blat;
    end



    if exist('ControlAllocator.m','file')

        Ucv0 = Init.Ucv;
        ncv = length(Ucv0);


        for i = 1:ncv
            Ucv = Ucv0;

            Ucv(i) = Ucv0(i)+du;
            [U_high] = ControlAllocator(Ucv,Init.FltCon,Veh);

            Ucv(i) = Ucv0(i)-du;
            [U_low] = ControlAllocator(Ucv,Init.FltCon,Veh);

            G(:,i) = (U_high - U_low)/(2*du);
        end

        LinModel.G = G;

    end

end

LinModel.Ref = Ref;



    function [Xdotres] = CalcXdotRes(X,U,Veh,AOA)

        Controls = U;


        Vb = X(1:3);
        Om = X(4:6);
        xyz = X(7:9);
        PTS = X(10:12);


        % Convert everything to body axis from stability axis for FMComp
        Vb = ConvertBAtoSAorSAtoBA([], Vb, AOA);
        Om = ConvertBAtoSAorSAtoBA([], Om, AOA);


        States.AirData.AOA = atan2(Vb(3),Vb(1));
        States.AirData.AOADeg = States.AirData.AOA*180/pi;
        States.AirData.AOAdot = 0;

        States.AirData.BETA = asin(Vb(2)/norm(Vb));

        States.AirData.uw = Vb(1);
        States.AirData.vw = Vb(2);
        States.AirData.ww = Vb(3);

        States.AirData.pw = Om(1);
        States.AirData.qw = Om(2);
        States.AirData.rw = Om(3);


        [States.AirData.OAT,States.AirData.a,States.AirData.P,States.AirData.rho] = atmosisa(-X(9));

        States.AirData.qbar = 0.5 * States.AirData.rho * (norm(Vb))^2;
        States.AirData.TAS = norm(Vb);
        States.AirData.EAS = States.AirData.TAS * sqrt(States.AirData.rho/1.225);
        States.AirData.MACH = States.AirData.TAS/States.AirData.a;
        States.Attitude.ph = PTS(1);
        States.Attitude.th = PTS(2);


        States.Posn.AltMSL = -xyz(3);
        %States.Posn.AltAGL = States.Posn.AltMSL;

        m = Veh.MassProp.Mass;
        r_cg = Veh.MassProp.r_cg;
        Icb = Veh.MassProp.MOI;

        EvalMode = 1;

        LLR = [0;0;0];
        EngStates_in = [];
        %[Fbf,Mbf,Hr,Hrdot] = FMComp(States,Controls,Veh,m,r_cg,EvalMode);
        [Fbf,Mbf,Hr,Hrdot,VehStatesDot,AddlRes,AddlOutputs] = FMComp(States,Controls,EngStates_in,Veh,m,r_cg,EvalMode);
        %[Fbf,Mbf,Hr,Hrdot,Xrotor_dot,AddlRes,AddlOutputs] = FMComp(States,Controls,XInflow,Veh,m,r_cg,EvalMode);

        phi = PTS(1); theta = PTS(2); psi = PTS(3);
        [~,~,~,~,Quats] = ConvEulerToQuats(phi,theta,psi);


        % Convert everything coming out of FMComp to stability axes
        Fbf = ConvertBAtoSAorSAtoBA(Fbf, [], AOA);
        Mbf = ConvertBAtoSAorSAtoBA(Mbf, [], AOA);
        Icb = ConvertBAtoSAorSAtoBA(Icb, [], AOA);
        Om = ConvertBAtoSAorSAtoBA(Om, [], AOA);
        Vb = ConvertBAtoSAorSAtoBA(Vb, [], AOA);
        Hr = ConvertBAtoSAorSAtoBA(Hr, [], AOA);
        Hrdot = ConvertBAtoSAorSAtoBA(Hrdot, [], AOA);


        % [Vbdot,Omdot,QuatsDot,LLRdot] =External_AircraftEOM_OffsetFromCG(Fb,Mb,mass,Ib,Ibdot,r_cg,v_cg,a_cg,Om,Quats,Vb,LLR,hb,hbdot)
        [Vbdot,Omdot,QuatsDot,LLRdot] = External_AircraftEOM_OffsetFromCG(Fbf,Mbf,m,Icb,zeros(3,3),r_cg,zeros(3,1),zeros(3,1),Om,Quats,Vb,LLR,Hr,Hrdot);




        LBV = [...
            cos(theta)*cos(psi),                                cos(theta)*sin(psi),                                -sin(theta);
            sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     sin(phi)*cos(theta);
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),     cos(phi)*cos(theta)];
        LVB = LBV';

        Xedot = LVB*(Vb);

        P = Om(1); Q = Om(2); R = Om(3);
        phi_dot     = P + Q*sin(phi)*tan(theta) + R*cos(phi)*tan(theta);
        theta_dot   = Q*cos(phi) - R*sin(phi);
        psi_dot     = Q*sin(phi)*sec(theta) + R*cos(phi)*sec(theta);
        PTSdot = [phi_dot,theta_dot,psi_dot]';

        Xdotres = [Vbdot;Omdot;Xedot;PTSdot];

    end




end