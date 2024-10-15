function [U,AddlStates,LB,UB,UNames,OBJFCN,UTrimVec0] = ControlMapper(UTrimVec,SpecControls,SpecAddlStates,FltCon,MappingNum,X)
global Vehicle
% This script is vehicle-specific

% LPC-03 Phoenix (01/18/2022)

% 1. Left OBD flaperon deflection, deg, positive TED
% 2. Left IBD flaperon deflection, deg, positive TED
% 3. Right IBD flaperon deflection, deg, positive TED
% 4. Right OBD flaperon deflection, deg, positive TED
% 5. Left elevator deflection, deg, positive TED
% 6. Right elevator deflection, deg, positive TED
% 7. Left rudder deflection, deg, positive TER
% 8. Right rudder deflection, deg, positive TER

% 9. Cruise prop throttle, [0-1]
% 10. Cruise prop pitch, deg
% 11. Cruise prop RPM

% 12. Lift prop 1 RPM
% 13. Lift prop 2 RPM
% 14. Lift prop 3 RPM
% 15. Lift prop 4 RPM
% 16. Lift prop 5 RPM
% 17. Lift prop 6 RPM
% 18. Lift prop 7 RPM
% 19. Lift prop 8 RPM

% 20. Lift prop 1 pitch
% 21. Lift prop 2 pitch
% 22. Lift prop 3 pitch
% 23. Lift prop 4 pitch
% 24. Lift prop 5 pitch
% 25. Lift prop 6 pitch
% 26. Lift prop 7 pitch
% 27. Lift prop 8 pitch


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
    'dt';
    'BCP';
    'CPRPM';
    'L1RPM';
    'L2RPM';
    'L3RPM';
    'L4RPM';
    'L5RPM';
    'L6RPM';
    'L7RPM';
    'L8RPM';
    'L1BETA';
    'L2BETA';
    'L3BETA';
    'L4BETA';
    'L5BETA';
    'L6BETA';
    'L7BETA';
    'L8BETA'};


AddlStates = [];
UNames = [];
OBJFCN = [];

% Set state bounds
ULATBounds = [-1,1];       % deg
ULONGBounds = [-1,1];      % deg
UDIRBounds = [-1,1];        % deg
ThrottleBounds = [0,1];         % [0-1]
RPMBounds = [200,2700];
BPBounds = [0, 40];
B0Bounds = [0, 18];
LPRPMBounds = [0, 5000];


if~isempty(SpecControls)
    UNames.SPECCTRL = SpecControls(:,1)';
end



%% MAPPING 1
if MappingNum == 1
    
    % desired cruise prop RPM
    CPRPMDes = max(1200,min(2200,1200 + (2200-1200)*(FltCon.KEAS)/(100)));

    
     Mtip = 0.8;
%     LPRPM0 = 340 * Mtip * (2/Vehicle.Propulsion.LiftPropDiam_m) * (30/pi);
% 
%     LPRPMDes = LPRPM0;

    
    

    if  FltCon.KEAS <=Vehicle.Aero.VThreshold/0.51444

%     LPRPMDes = LPRPM0 - (LPRPM0-1200)*(FltCon.KEAS*0.51444-20)/(Vehicle.Aero.VThreshold-20);

    LPRPMDes = (Mtip * 340 - FltCon.KTAS * 0.51444) * (2/Vehicle.Propulsion.LiftPropDiam_m) * (30/pi);

    end

    B0Des = 12;
    
    BPGuess = 32; 
    if FltCon.KEAS<40
        BPGuess = 1;
        BPBounds = [1,1];
    end
    
    B0Guess = 3;

    if FltCon.KEAS > Vehicle.Aero.VThreshold/0.51444
        B0Bounds = [0,0];
        LPRPMBounds = [0,0];
        LPRPMDes = 0;
        LPRPMDes = 0;
        B0Guess = 0;
    end

    
    % set default guesses
    ULATGuess = 0; ULONGGuess = 0; UDIRGuess = 0; ThrGuess = 0.1;  RPMGuess = CPRPMDes;  LPRPMGuess = LPRPMDes;
        
    UNames.CTRL = {'ULAT','ULONG','UDIR','THR','BP','RPM','B0'};
    
    UTrimVec0 = [ULATGuess, ULONGGuess, UDIRGuess, ThrGuess, BPGuess,  RPMGuess , B0Guess]';
    
    Bounds = [ULATBounds;ULONGBounds;UDIRBounds;ThrottleBounds;BPBounds;RPMBounds; B0Bounds];
    
%     UTrimVec(8) = LPRPMDes;

    [U] = ControlAllocator(UTrimVec,FltCon,Vehicle);
    
    
% ulat = UTrimVec(1);             % positive for rolling right
% ulong = UTrimVec(2);            % positive for pitching up
% udir = UTrimVec(3);             % positive for yawing right
dt_cp = UTrimVec(4);            % throttle
% bp_cp = UTrimVec(5);            % cruise prop pitch
rpm_cp = UTrimVec(6);           % cruise motor RPM
B0 = UTrimVec(7);               % lift rotors collective pitch
% rpm_lp = UTrimVec(8);           % lift rotor RPM

      
ulat = UTrimVec(1);             % positive for rolling right
ulong = UTrimVec(2);            % positive for pitching up
udir = UTrimVec(3);             % positive for yawing right
dt_cp = UTrimVec(4);            % throttle
bp_cp = UTrimVec(5);            % cruise prop pitch
rpm_cp = UTrimVec(6);           % cruise motor RPM
B0 = UTrimVec(7);               % lift rotors collective pitch
% rpm_lp = UTrimVec(8);           % lift rotor RPM


    
    AddlStates = [];
    

    

    
    % compute value of objective function
    % minimize throttle
    fobj(1) = .1*dt_cp;
    % try to match desired RPM
    fobj(2) = .1*(rpm_cp - CPRPMDes)^2;
    % 3
    fobj(3) = 0;
    % 4
    fobj(4) = 0;
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






