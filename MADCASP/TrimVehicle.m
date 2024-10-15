function [Soln] = TrimVehicle(FltCon,Vehicle,Settings)

% EvalMode. = 1 if calling force&moment buildup functions from MATLAB. = 2
% if these are being called from within Simulink
EvalMode = 1;

% Add information to the Flight Condition (FltCon) structure
% e.g., if speed is given in terms of one speed type (MACH, KEAS, KTAS),
% compute the others based on that information
[FltCon] = CompleteFltCon(FltCon);


% Capture vehicle mass and CG location for case 
m = Vehicle.MassProp.Mass;
r_cg = Vehicle.MassProp.r_cg;

% Set the lower and upper bounds (X_LB and X_UB) and initial value (X0) of the
% state vector (motion states)
[X_LB,X_UB,X0] = InitializeX(FltCon);
Settings.NumStates = length(X0);        % this captures the number of states included in the trim vector

% Set the lower and upper bounds (U_LB and U_UB) and initial value (U0) of
% the control vector
UTrimVec = zeros(100,1);
SpecControls = FltCon.SpecControls;
SpecAddlStates = FltCon.SpecAddlStates;

[~,~,U_LB,U_UB,~,~,U0] = ControlMapper(UTrimVec,SpecControls,SpecAddlStates,FltCon,Settings.MappingNum,X0);

% concatenate state and control upper and lower bounds and initial values
% to get upper and lower bounds and initial value of the trim vector
LB = [X_LB;U_LB];
UB = [X_UB;U_UB];
TrimVector0 = [X0;U0];

% option parameters for fmincon algorithm
options.Display = 'off'; %'iter-detailed';
% options.Algorithm = 'sqp';
% options.Algorithm = 'active-set';
% options.Algorithm = 'interior-point';

options.ConstraintTolerance = 1e-2;

% handles for constraint and objective functions
% fcon: equality constraints that have to be satisfied
fcon = @(TrimVector)NONLCON(TrimVector,FltCon,Vehicle,m,r_cg,EvalMode,Settings);
% fobj: objective function that has to be minimized during trimming
fobj = @(TrimVector)PseudoObjFcn(TrimVector,FltCon,Vehicle,m,r_cg,EvalMode,Settings);

AlgoOrder = {'sqp','active-set','interior-point'};
Trimmed = 0;
ctr = 1;

while(Trimmed==0&&ctr <=3)
    
    options.Algorithm = AlgoOrder(ctr);
    fprintf('... %s',AlgoOrder{ctr});
    % fmincon (constrained minimization) algorithm used for trimming
    % Syntax I/O: [X,FVAL,EXITFLAG,OUTPUT] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)
%     fobj(X)
%     fcon(X)
    [TrimVector,FVAL,EXITFLAG,~] = fmincon(fobj,TrimVector0,[],[],[],[],LB,UB,fcon,options);
    
    % Run the residual calculation function (CalcResiduals.m) in "report" mode
    % with the converged TrimVector to record all
    [Res,Soln2] = CalcResiduals(TrimVector,FltCon,Vehicle,m,r_cg,'report',Settings);
    
    ctr = ctr + 1;
    
    if(norm(Res)<0.0001)
        Trimmed = 1;
    end
    
end



% Record the results (exit condition and residual norm) and the flight
% condition at which the vehicle was trimmed
Soln1.exitflag = EXITFLAG;
Soln1.Res = norm(Res);
Soln1.FVAL = FVAL;
Soln1.ALT =  FltCon.ALT{1};
Soln1.MACH = FltCon.Mach;
Soln1.KEAS = FltCon.KEAS;
Soln1.KTAS = FltCon.KTAS;

% Concatenate to get the final trim solution
Soln = [struct2table(Soln1),Soln2];
Soln.Vehicle = Vehicle;
Soln.FltCon = FltCon;

% screen display message
fprintf('--- exitflag = %0.2f, Res %0.6f\n',EXITFLAG,Soln1.Res);







%% InitializeX embedded function
    function [X_LB,X_UB,X0] = InitializeX(FltCon)
        
        %State bounds
%         
%         if FltCon.EAS < 100*0.51444
%         AOABounds = [-5,0];           % deg
%         else
%             AOABounds = [-5,14];
%         end
        
        AOABounds = [-10,25];


if FltCon.KEAS <= 80
    AOABounds = [0,0];
end


        BETABounds = [-20,20];          % deg
        BankBounds = [-60,60];          % deg
        TurnRateBounds = [-200,200];    % deg/s
        HeadingBounds = [0,360];        % deg
        FPABounds = [-30,30];           % deg
        
        
        AOA0 = 0.0;
        BETA0 = 0.0;
        BANK0 = 0;
        FPA0 = 0;
        
        % conversion factors
        kt_to_ms = 0.51444;
        
        % constants
        g = 9.81;
        
        % if sideslip is specified, set that as the initial guess
        if~isempty(FltCon.BETA)
            BETA0 = FltCon.BETA;
        end
        
        % if bank angle is specified:
        if strcmpi(FltCon.Turn{1},'bank')
            BANK0 = FltCon.Turn{2};
            TURNRATE0 = g * tand(BANK0)/FltCon.TAS;
        end
        
        % if turnrate is specified:
        if strcmpi(FltCon.Turn{1},'turnrate')
            TURNRATE0 = FltCon.Turn{2};
            BANK0 = atand(FltCon.TAS * TURNRATE0 / g);
        end
        
        % if turn radius is specified:
        if strcmpi(FltCon.Turn{1},'turnradius')
            R = FltCon.Turn{2};
            BANK0 = atand(FltCon.TAS^2 / R * g);
            TURNRATE0 = FltCon.TAS/R;
        end
        
        % if turn load factor is specified:
        if strcmpi(FltCon.Turn{1},'loadfactor')
            n = FltCon.Turn{2};
            BANK0 = acosd(1/n);
            TURNRATE0 = g * tand(BANK0)/FltCon.TAS;
        end
        
        % if heading is specified, set that as the initial guess, else compute an
        % initial heading guess based on specified track and wind
        if ~isempty(FltCon.HDG)
            HEADING0 = FltCon.HDG;
        else
            Vcw = FltCon.WindSpd * kt_to_ms * sind(FltCon.WindDir - FltCon.TRK);
            WCA = asind(Vcw/FltCon.TAS);
            if(isnan(WCA))
                WCA = 0;
            end
            HEADING0 = FltCon.TRK + WCA;
        end
        
        
        % if flightpath angle is not specified
        if isempty(FltCon.FPA)
            XBounds = [AOABounds;BETABounds;BankBounds;TurnRateBounds;HeadingBounds;FPABounds];
            X_LB = XBounds(:,1);
            X_UB = XBounds(:,2);
            X0 = [AOA0;BETA0;BANK0;TURNRATE0;HEADING0;FPA0];
        end
        
        % if flightpath angle is specified
        if ~isempty(FltCon.FPA)
            XBounds = [AOABounds;BETABounds;BankBounds;TurnRateBounds;HeadingBounds];
            X_LB = XBounds(:,1);
            X_UB = XBounds(:,2);
            X0 = [AOA0;BETA0;BANK0;TURNRATE0;HEADING0];
        end
        
    end


%% PseudoObjFcn embedded function
    function [OBJFCN] = PseudoObjFcn(X,FltCon,Vehicle,m,r_cg,EvalMode,Settings)
        
        specControls = FltCon.SpecControls;
        specAddlStates = [];
        MappingNum = Settings.MappingNum;
        uTrimVec = X(Settings.NumStates+1:end);
        StateVec = X(1:Settings.NumStates);
 
        % CONTROL PENALTIES
        [~,~,~,~,~,OBJFCN1] = ControlMapper(uTrimVec,specControls,specAddlStates,FltCon,MappingNum,X);
        
        % STATE PENALTIES
        [OBJFCN2] = StatePenalty(StateVec,FltCon);
        
        % PERFORMANCE PENALTIES
        EvalModeSpecial = 'report';
        [~,PerfSol] = CalcResiduals(X,FltCon,Vehicle,m,r_cg,EvalModeSpecial,Settings);
        
        [OBJFCN3] = PerfPenalty(PerfSol,FltCon);
        
        OBJFCN = OBJFCN1 + OBJFCN2 + OBJFCN3;
        
       
    end


%% NONLCON embedded function
    function [c, ceq] = NONLCON(x,FltCon,Vehicle,m,r_cg,EvalMode,Settings)
        c=[];
        [ceq] = CalcResiduals(x,FltCon,Vehicle,m,r_cg,EvalMode,Settings);
        
    end






end