function [Ps,FPA,V_out,vdot,dt_out,AOA,CTRL,dmdt,dEdt,LDE,ddtStates,fval,exitflag,soln] = SolveFltCon(mass,dt,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle,SolverType,varargin)

% capture any suggested initial conditions from prior solutions
X0prev = [];
if ~isempty(varargin)
    X0prev = varargin{1};
end
X0 = X0prev;



% SolverType = 2;     % use fsolve
% SolverType = 3;     % use fmincon
% SolverType = 2;     % use newton solver

% inputs:
% mass of the vehicle, kg
% dt, in 'perf' mode: vector of normalized throttle settings;
% dt, in 'power' mode: will be scaled up in the ratio of dt elements to
% meet requested performance
% rho, density, kg/m3
% V, airspeed, m/s

% zero out the following residuals:
% 1. x-wind axis (axial force)
% 2. z-wind axis (normal force)
% 3. y-wind axis (pitching moment)

% constants
g = 9.81;
% EvalType

options = optimoptions('fsolve','Display','off','FunctionTolerance',1e-6,'StepTolerance',1e-6);
options.Algorithm = 'levenberg-marquardt';

if SolverType == 3
    options = optimoptions('fmincon','Display','off','Algorithm','sqp');
end

% options.Algorithm = 'trust-region-dogleg';
% options.Algorithm = 'trust-region';


% If achievable flightpath performance is to be evaluated
if strcmpi(EvalType,'perf')
    Mode = 1;
    PrescribedAttitude = [];

    if isempty(X0prev)
        X0 = [0,8*pi/180,0.0];
    end

    if V<= Vehicle.Aero.VThreshold
        Mode = 2;
        % initial guesses, cruise throttle, lift rotor RPM,
        % longitudinal control
        if isempty(X0prev)
            X0 = [300/(60*3.28), 0.2, 0];
        end

        PrescribedAttitude = Vehicle.Aero.PitchSchedule_V(V)*pi/180;
    end

    dVdt = 0;
    dFPAdt = 0;

    f = @(X)CalcResPerfMode(X,dt,rho,V,dVdt,dFPAdt,mass,g,Vehicle,Mode,PrescribedAttitude);

    % zero out the residual equations
    if SolverType == 1
        [soln,fval,exitflag,output] = fsolve(f,X0,options);
    end

    if SolverType == 2
        [soln,fval,exitflag,output] = newtonsolver(f,X0,options);
    end

    if Mode == 1
        % capture flight path angle, angle of attack, and control setting
        FPA = soln(1);
        %         fprintf('dhdt = %0.2f\n',V*sin(FPA)*3.28*60)
        AOA = soln(2);
        CTRL = soln(3);
        dt_out(1) = dt;
        dt_out(2) = 0;
    end

    if Mode == 2
        dhdt = soln(1);
        V = max(V,abs(dhdt));
        FPA = asin(dhdt/(0.0000001+V));
        AOA = PrescribedAttitude - FPA;
        dt_out(1) = soln(2);
        dt_out(2) = dt;
        CTRL = soln(3);
    end

    % with trim solution known, evaluate Aero-Propulsive performance
    [Fxap,Fzap,myap,dmdt,dEdt,LDE,ddtStates] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,FPA,Vehicle);

    % calculate specific excess power
    Ps = Fxap*V/(mass*g);
    vdot = g*Ps/V;

    V_out = V;

end



% If achievable acceleration performance is to be evaluated
if strcmpi(EvalType,'accel')
    Mode = 1;
    PrescribedAttitude = [];

    if isempty(X0prev)
        X0 = [1,8*pi/180,0.0];
    end

    if V<= Vehicle.Aero.VThreshold
        Mode = 2;
        % initial guesses, cruise throttle, lift rotor RPM,
        % longitudinal control
        if isempty(X0prev)
            X0 = [0.00, 1, 0];
        end

        PrescribedAttitude = Vehicle.Aero.PitchSchedule_V(V)*pi/180;
    end

    dFPAdt = 0;

    V = max(V,abs(dhdt));
    FPA = asin(dhdt/(0.0000001+V));

    f = @(X)CalcResAccelMode(X,dt,rho,V,FPA,dFPAdt,mass,g,Vehicle,Mode,PrescribedAttitude);

    % zero out the residual equations
    if SolverType == 1
        [soln,fval,exitflag,output] = fsolve(f,X0,options);
    end

    if SolverType == 2
        [soln,fval,exitflag,output] = newtonsolver(f,X0,options);
    end

    if Mode == 1
        % capture acceleration, angle of attack, and control setting
        vdot = soln(1);
        AOA = soln(2);
        CTRL = soln(3);
        dt_out(1) = dt;
        dt_out(2) = 0;
    end

    if Mode == 2
        vdot = soln(1);
        AOA = PrescribedAttitude - FPA;
        if dt < 0.05
            AOA = 4*pi/180;
        end
        dt_out(1) = dt;
        dt_out(2) = soln(2);
        CTRL = soln(3);
    end

    % with trim solution known, evaluate Aero-Propulsive performance
    [Fxap,~,~,dmdt,dEdt,LDE,ddtStates] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,FPA,Vehicle);

    % calculate specific excess power
    Ps = Fxap*V/(mass*g);


    V_out = V;
end

% if required power setting is to be evaluated
if strcmpi(EvalType,'power')
    
    % display('Velocity and dhdt')
    V = max(V,abs(dhdt));
    %         dhdt

    FPA = asin(dhdt/(0.0000001+V));

    Mode = 1;
    PrescribedAttitude = [];
    
    % initial guesses, cruise throttle, AOA, longitudinal control
    if isempty(X0prev)
        X0 = [0.01,3*pi/180,0];
    end
    lb = [0.01 -5 -1];
    ub = [1 10 1];

    if V<= Vehicle.Aero.VThreshold
        Mode = 2;
        % initial guesses, cruise throttle, lift rotor RPM,
        % longitudinal control
        if isempty(X0prev)
            % X0 = [0.000001000000000   0.863796748787442   0.019044392699943];
            % X0 = [0.01 0.2 0.1];
            X0(1) = interp1([0 Vehicle.Aero.VThreshold],[0 0.12],V,'linear');
            X0(2) = interp1([0 Vehicle.Aero.VThreshold],[1 0],V,'linear');
            X0(3) = 0.1;
        end  

        lb = [0 0 -1];
        ub = [1.4 1.4 1];
        PrescribedAttitude = Vehicle.Aero.PitchSchedule_V(V)*pi/180;
    end

    %         Mode

    f = @(X)CalcResPowerMode(X,FPA,fpadot,vdot,rho,V,mass,g,Vehicle,Mode,PrescribedAttitude,SolverType);

    % zero out the residual equations
    %options.Algorithm = 'levenberg-marquardt';

    if SolverType == 1
        [soln,fval,exitflag,output] = fsolve(f,X0,options);
    end

    if SolverType == 2
        [soln,fval,exitflag,output] = newtonsolver(f,X0,options);
    end

    if SolverType == 3
        [soln,fval,exitflag,~] = fmincon(f,X0,[],[],[],[],lb,ub,[],options);
    end


    if Mode == 1
        dt_out(1) = soln(1);
        dt_out(2) = 0;
        AOA = soln(2);
        CTRL = soln(3);
    end

    if Mode == 2
        dt_out(1) = soln(1);
        dt_out(2) = soln(2);
        AOA = PrescribedAttitude - FPA;
        CTRL = soln(3);
    end

    % with trim solution known, evaluate Aero-Propulsive performance
    [Fxap,Fzap,Myap,dmdt,dEdt,LDE,ddtStates] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,FPA,Vehicle);

    % calculate specific excess power
    Ps = Fxap*V/(mass*g);

    V_out = V;

    if exitflag < 0
        res(1,1)= Fxap/mass - g*sin(FPA) - vdot;
        res(2,1)= Fzap/mass + g*cos(FPA) + V*fpadot;
        res(3,1)= Myap(1)/mass;
    end


end



% if required power setting is to be evaluated
if strcmpi(EvalType,'trim')

    V = max(V,abs(dhdt));

    FPA = asin(dhdt/(0.0000001+V));

    Mode = 1;
    PrescribedAttitude = [];

    % initial guesses, cruise throttle, AOA, longitudinal control
    if isempty(X0prev)
        X0 = [0.9,3*pi/180,0.0, 0.0, 0.0];
    end

    if V<= Vehicle.Aero.VThreshold
        Mode = 2;
        % initial guesses, cruise throttle, lift rotor RPM,
        % longitudinal control
        if isempty(X0prev)
            X0 = [0.01, 1, 0.1, 0.1, 0.1];
        end
        lb = [0.0001 0 -1 -1 -1];
        ub = [1.7 1.7 1 1 1];

        PrescribedAttitude = Vehicle.Aero.PitchSchedule_V(V)*pi/180;
    end


    f = @(X)CalcResTrimMode(X,FPA,fpadot,vdot,rho,V,mass,g,Vehicle,Mode,PrescribedAttitude,SolverType);

    % zero out the residual equations
    if SolverType == 1
        [soln,fval,exitflag,output] = fsolve(f,X0,options);
    end

    if SolverType == 2
        [soln,fval,exitflag,output] = newtonsolver(f,X0,options);
    end

     if SolverType == 3
        [soln,fval,exitflag,~] = fmincon(f,X0,[],[],[],[],lb,ub,[],options);
    end

    if Mode == 1
        dt_out(1) = soln(1);
        dt_out(2) = 0;
        AOA = soln(2);
        CTRL = [soln(3),soln(4),soln(5)];
    end

    if Mode == 2
        dt_out(1) = soln(1);
        dt_out(2) = soln(2);
        AOA = PrescribedAttitude - FPA;
        CTRL = [soln(3),soln(4),soln(5)];
    end

    % with trim solution known, evaluate Aero-Propulsive performance
    [Fxap,~,~,dmdt,dEdt,LDE,ddtStates] = AeroPropPerf(AOA,CTRL,dt_out,rho,V,FPA,Vehicle);

    % calculate specific excess power
    Ps = Fxap*V/(mass*g);

    V_out = V;


end





if(length(CTRL)<3)
    CTRL = [CTRL,0,0];
end

% LDE = [LDE,exitflag];





% Residual calculation function for "performance" mode
    function [res] = CalcResPerfMode(X,dt,rho,V,Vdot,FPAdot,m,g,Vehicle,Mode,PresVar)

        if Mode == 1
            fpa = X(1);
            aoa = X(2);
            de = X(3);
            dthr(1) = dt;
            dthr(2) = 0;
        end


        if Mode == 2
            dh_dt = X(1);
            V = max(V,abs(dh_dt));
            fpa = asin(dh_dt/(0.0000001+V));
            aoa = PresVar - fpa;
            de = X(3);
            dthr(1) = X(2);
            dthr(2) = dt;
        end


        [fxap,fzap,myap,~,~,~,~] = AeroPropPerf(aoa,de,dthr,rho,V,fpa,Vehicle);

        res(1,1)= fxap/m - g*sin(fpa) - Vdot;
        res(2,1)= fzap/m + g*cos(fpa) + V*FPAdot;
        res(3,1)= myap(1)/m;

    end

% Residual calculation function for "accel" mode
    function [res] = CalcResAccelMode(X,dt,rho,V,fpa,FPAdot,m,g,Vehicle,Mode,PresVar)

        if Mode == 1
            Vdot = X(1);
            aoa = X(2);
            de = X(3);
            dthr(1) = dt;
            dthr(2) = 0;
        end


        if Mode == 2
            Vdot = X(1);
            %V = max(V,abs(dh_dt));
            %fpa = asin(dh_dt/(0.0000001+V));
            aoa = PresVar - fpa;
            if dt <0.05
                aoa = 4*pi/180;
            end
            de = X(3);
            dthr(1) = dt;
            dthr(2) = X(2);
        end


        [fxap,fzap,myap,~,~,~,~] = AeroPropPerf(aoa,de,dthr,rho,V,fpa,Vehicle);

        res(1,1)= fxap/m - g*sin(fpa) - Vdot;
        res(2,1)= fzap/m + g*cos(fpa) + V*FPAdot;
        res(3,1)= myap(1)/m;

    end



% Residual calculation function for "power" mode
    function [res] = CalcResPowerMode(X,fpa,fpadot,vdot,rho,V,m,g,Vehicle,Mode,PresVar,SolverType)

        if Mode == 1
            dthr(1) = X(1);
            aoa = X(2);
            dcon = X(3);
            dthr(2) = 0;
        end

        if Mode == 2
            dthr(1) = X(1);
            dthr(2) = X(2);
            dcon = X(3);
            aoa = PresVar - fpa;
        end

        [fxap,fzap,myap,~,~,~,~] = AeroPropPerf(aoa,dcon,dthr,rho,V,fpa,Vehicle);

        res(1,1)= fxap/m - g*sin(fpa) - vdot;
        res(2,1)= fzap/m + g*cos(fpa) + V*fpadot;
        res(3,1)= myap(1)/m;

        if SolverType == 3
            res = norm(res);
        end

    end



% Residual calculation function for "trim" mode
    function [res] = CalcResTrimMode(X,fpa,fpadot,vdot,rho,V,m,g,Vehicle,Mode,PresVar,SolverType)

        if Mode == 1
            dthr(1) = X(1);
            aoa = X(2);
            dcon = [X(3),X(4),X(5)];
            dthr(2) = 0;
        end

        if Mode == 2
            dthr(1) = X(1);
            dthr(2) = X(2);
            dcon = [X(3),X(4),X(5)];
            aoa = PresVar - fpa;
        end

        [fxap,fzap,myap,~,~,~,~] = AeroPropPerf(aoa,dcon,dthr,rho,V,fpa,Vehicle);

        res(1,1)= fxap/m - g*sin(fpa) - vdot;
        res(2,1)= fzap/m + g*cos(fpa) + V*fpadot;
        res(3,1)= myap(1)/m;
        res(4,1) = myap(2)/m;
        res(5,1) = myap(3)/m;

        if SolverType == 3
            res = norm(res);
        end

    end




% newton solver (for testing)
    function [soln,fval,exitflag,output] = newtonsolver(f,X0,options)
        exitflag = 0;
        n = length(X0);


        X = X0';

        maxiter = 5;
        etol = 0.01;
        conv = 0;
        iter = 0;

        dx = .01;

        while(conv <= 0 && iter <=maxiter)
            iter = iter + 1;
            J0 = zeros(n,n);
            IllDefined = 0;

J  = zeros(n,n);
            for i = 1:n
                XH = X;
                XH(i) = XH(i) + dx;
                fH = f(XH);

                XL = X;
                XL(i) = XL(i) - dx;
                fL = f(XL);

                J(:,i) = (fH-fL)/(2*dx);

                if(all(J(:,i)==0))
                    IllDefined = IllDefined + 1;
                end
            end
            Xold = X;
            e = f(X);
            conv = all(abs(e)<etol,'all');

            conv(IllDefined>0) = -1;
            
            if(IllDefined==0)
            J0 = J;
            end

            X = Xold - J0\e;

%             % e
%             inv(J)
            

            %             Mode

        end

        soln = X;
        fval = e;
        exitflag(conv==1) = norm(e);
        output = [];

    end




end