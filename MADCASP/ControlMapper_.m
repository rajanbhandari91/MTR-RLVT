function [U,AddlStates,LB,UB,UNames,OBJFCN,UTrimVec0] = ControlMapper(UTrimVec,SpecControls,SpecAddlStates,FltCon,MappingNum,X)
global Vehicle
% This script is vehicle-specific

% LPC-03 Subscale Model (02/08/2023)

% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Left rudder deflection, deg, positive TER
% 8. Right rudder deflection, deg, positive TER

% 9. Cruise prop RPM

% 10. Lift prop 1 RPM
% 11. Lift prop 2 RPM
% 12. Lift prop 3 RPM
% 13. Lift prop 4 RPM
% 14. Lift prop 5 RPM
% 15. Lift prop 6 RPM
% 16. Lift prop 7 RPM
% 17. Lift prop 8 RPM


% short names for each of the controls
CtrlNames = {...
    'F1';
    'F2';
    'F3';
    'F4';
    'E1';
    'E2';
    'R1';
    'R2';
    'CPRPM';
    'L1RPM';
    'L2RPM';
    'L3RPM';
    'L4RPM';
    'L5RPM';
    'L6RPM';
    'L7RPM';
    'L8RPM';
    'dt'
    };


AddlStates = [];
UNames = [];
OBJFCN = [];

% Set state bounds
ULATBounds = [-1,1];       % deg
ULONBounds = [-1,1];      % deg
UDIRBounds = [-1,1];        % deg
CPRPMBounds = [200,15000];
LPRPMBounds = [0,15000];
dtBounds = [0,1.5];


if~isempty(SpecControls)
    UNames.SPECCTRL = SpecControls(:,1)';
end



%% MAPPING 1
if MappingNum == 1
    
    % if above TXN speed, limit LPRPM to idle
    if FltCon.KEAS >= 50
        LPRPMBounds = [0,0];
    end

    
    % set default guesses
    ULATGuess = 0;
    ULONGuess = 0;
    UDIRGuess = 0;
    RPMGuess = 5000;
    LPRPMGuess = 7500;
    dtGuess = 0;

    % if above TXN speed, guess higher RPM for CP
    if FltCon.KEAS >= 25
        RPMGuess = 10000;
    end
        
    UNames.CTRL = {'ULAT','ULON','UDIR','CPRPM','LPRPM','dtcp'};
    
    UTrimVec0 = [ULATGuess,ULONGuess,UDIRGuess,RPMGuess,LPRPMGuess,dtGuess]';
    
    Bounds = [ULATBounds;ULONBounds;UDIRBounds;CPRPMBounds;LPRPMBounds;dtBounds];

    [U] = ControlAllocator(UTrimVec,FltCon,Vehicle);

    % Grab controls for penalizing (if desired)
    ulat = UTrimVec(1);     % positive for rolling right
    ulon = UTrimVec(2);     % positive for pitching up
    udir = UTrimVec(3);     % positive for yawing right
    rpm_cp = UTrimVec(4);   % cruise motor RPM
    rpm_lp = UTrimVec(5);   % lift rotor RPM
    dt = UTrimVec(6);       % cruise propeller throttle setting


    
    AddlStates = [];

    % compute value of objective function
    % minimize directional input
    fobj(1) = dt;
    % minimize lateral input
%     fobj(2) = ulat;
    % minimize longitudinal input
    fobj(3) = 0.1*abs(ulon);
    % sum to obtain objective function
    OBJFCN = sum(fobj);
    
end



%% overrides

if~isempty(SpecControls)
    ind = find(contains(CtrlNames,SpecControls(:,1)));
    for i=1:1:length(ind)
        U(ind(i)) = SpecControls{i,2};
    end
end


%% bounds

% Lower bounds
LB = Bounds(:,1);
% Upper bounds
UB = Bounds(:,2);






