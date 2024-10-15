function [Vehicle] = TailSizingCalculations(Vehicle,Fus_HTail,Fus_VTail)


fprintf(' Stab sizing:')

% conversion factors
conv_kt_to_ms = 0.51444;
conv_ft_to_m = 1/3.28;

% create a local copy of the Vehicle structure
Veh = Vehicle;

% pick a test point from the power sizer list
TP = Vehicle.PointPerf.Constraints(1,:); % high altitude cruise

% condition specific
h = TP.PA_ft * conv_ft_to_m;
rho = TP.rho;
SIG = TP.SIG;
mass = Vehicle.MassProp.MTOM_kg;
V = TP.KTAS * conv_kt_to_ms;
% 1.1 * Vehicle.Aero.VThreshold/sqrt(SIG);         %

% set the CG location to target CG location
Veh.MassProp.r_cg = Veh.MassProp.TargetCG;


% vector of volume ratios to test for the horizontal stabililzer
HTVRVector = [0.8:0.05:1.2];

% vector of volume ratios to test for the vertical stabililzer
% VTVRVector = [0.05:0.015:0.2];





%%%%%%%%%% HORIZONTAL STABILIZER %%%%%%%%%%%%%%%%%%%%%%%%%

VH_found = 0;
i = 1;
imax = length(HTVRVector);
HTVRSelected = 0;

while(VH_found == 0 && i < imax)

    Veh.DesignPoint.HTVolumeRatio = HTVRVector(i);
    % Vehicle.DesignPoint.VTVolumeRatio = VTVRVector(i);

    [Veh] = StabilizerGeometryCalculations(Veh,Fus_HTail,Fus_VTail);
    [Veh] = UpdateAeroModel(Veh);

    [Sol] = AssessDynamicStability(V,h,rho,mass,Veh);

    % weighted constraint vector for longitudinal constraints
    c_long_weighted(i,:) = Sol.c_long_weighted;
    c_lat_weighted(i,:) = Sol.c_lat_weighted;

    c_weighted(i,:) = [c_long_weighted(i,:),c_lat_weighted(i,:)];

    if all(c_weighted(i,:) == 0)
        VH_found = 1;
        HTVRSelected = Veh.DesignPoint.HTVolumeRatio;
        % VTVRSelected = Veh.DesignPoint.VTVolumeRatio;

    else
        i = i + 1;
    end


end
% c_long_weighted

% case 2: no case satisfied all constraints (HTVR selected is still 0)
if HTVRSelected == 0

    % sum across each row
    c_sum = sum(c_weighted,2);

    % find the case that minimizes constraint violations
    [~,indminviolation] = min(c_sum);

    HTVRSelected = HTVRVector(indminviolation);
    % VTVRSelected = VTVRVector(indminviolation);

end











stopper = 1;




% prepare Vehicle structure for output
Vehicle.DesignPoint.HTVolumeRatio = HTVRSelected;
% Vehicle.DesignPoint.VTVolumeRatio = VTVRSelected;
[Vehicle] = StabilizerGeometryCalculations(Vehicle,Fus_HTail,Fus_VTail);
% Vehicle.DesignPoint.HTVolumeRatio = Vehicle.Recalcs.HTVolumeRatio;



[Vehicle] = UpdateAeroModel(Vehicle);


% set the CG location to target CG location
Vehicle.MassProp.r_cg = Veh.MassProp.TargetCG;
[Vehicle.Stability] = AssessDynamicStability(V,h,rho,mass,Vehicle);



fprintf(' VH %0.2f (%0.0f), VV %0.3f', Vehicle.DesignPoint.HTVolumeRatio, VH_found, Vehicle.DesignPoint.VTVolumeRatio)



Stopper = 1;







function [Sol] = AssessDynamicStability(V,h,rho,mass,Vehicle)




% defaults
dt0 = 1;
dhdt = 0;
fpadot = 0;
vdot = 0;
EvalType = 'power';
tolerance = 0.00;

SolverType = 1;

% Trim the aircraft at the given flight condition
[~,~,~,~,dt_out,AOA,CTRL,~,~,ADDL,~,FLAG,~] = SolveFltCon(mass,dt0,rho,V,dhdt,fpadot,vdot,EvalType,Vehicle,SolverType);

conv_ms_to_kt = 1/0.51444;

% Init.X: state vector at reference condition
X0 = [...
    V * cos(AOA);                                       % u (m/s)
    0;                                                  % v (m/s)
    V * sin(AOA);                                       % w (m/s)
    0;                                                  % p (rad/s)
    0;                                                  % q (rad/s)
    0;                                                  % r (rad/s)
    0;                                                  % x (m) - irrelevant
    0;                                                  % y (m) - irrelevant
    -h;                                                 % z (m)
    0;                                                  % phi (rad)
    AOA;                                                % theta (rad)
    0;                                                  % psi (rad) - irrelevant
    ];

Init.X = X0;

PropAng = max(0,90 - (85/(Vehicle.Aero.VThreshold/0.51444))*(V/0.51444)*cos(AOA));


ulat = CTRL(2);
ulong = CTRL(1);
udir = CTRL(3);
BCP = ADDL(10);
BLP = ADDL(9);
LPRPM = ADDL(8);
CPRPM = ADDL(14);

UTrimVec = [...
    ulat
    ulong
    udir
    PropAng
    BCP
    BLP
    LPRPM
    CPRPM ];




FltCon.KTAS = V * conv_ms_to_kt;
FltCon.KEAS = FltCon.KTAS * sqrt(rho/1.225);
FltCon.SpeedOfSound = 340.2941 * sqrt(rho/1.225);
Init.FltCon = FltCon;

[U0] = ControlAllocator(UTrimVec,FltCon, Vehicle);
Init.U = U0;
Init.Ucv = UTrimVec;

longcon = [];
latcon = [];



Ref.Velocity = V;
Ref.RefLongLength = Vehicle.Geom.RWing.MAC;
Ref.RefLatLength = Vehicle.Geom.RWing.Span*2;


Sol.AOA = AOA * 180/pi;
Sol.X0 = X0;
Sol.UTrimVec = UTrimVec;
Sol.U0 = U0;
Sol.ADDL = ADDL;
Sol.FLAG = FLAG;


% linearize the model about this trim point
Mode = 1; % evaluates only A matrix
[LinModel] = LinearizeModel(Init,Vehicle,longcon,latcon,Ref, Mode);
Sol.LinModel = LinModel;


%%%%%% ANALYZE LONGITUDINAL MODES
LongVarNames = {'u','w','q','x','z','th'};
LongVarRef = {'th'};

EVFactor = 2*Ref.Velocity/Ref.RefLongLength;
[LongModes] = AnalyzeModes(LinModel.Alongnd,EVFactor,LongVarNames,LongVarRef);
Sol.LongModes = LongModes;

% modes are arranged in increasing order of UNF

% pick out the short period (SP) as being the last two rows
EigSP = LongModes(end-1:end,:);

% pick out the damping ratio of the SP. Pick the minimum value from these
% two rows. If SP is stable, they will be the same. If unstable, the
% minimum value will be <0, and will get disqualified by constraint
% checking
Sol.ShortPeriodDR = min(EigSP.Zeta);

% pick out the undamped natural frequency (UNF) of the short period. If
% short period is stable, both rows will have the same UNF (complex
% conjugage pair). If short period is unstable, then this selection is not
% important, because the damping ratio will be negative
Sol.ShortPeriodUNF = min(EigSP.UNF);

% compute n/AOA and Control Anticipation Parameter (CAP)
% 1. find Zw = partial(w-dot)/partial(w) from Along
Zw = LinModel.Along(2,2);
% 2. find Zalpha = V x Zw = partial(w-dot)/partial(AOA) - unit: m/s2
Zalpha = V * Zw;
% 3. n_AOA --> approximated as n_AOA = - Zalpha/g - unit: g/rad or
Sol.n_AOA  = - Zalpha / 9.81;
% 4. CAP = (SP UNF)^2/(n_AOA)
Sol.CAP = Sol.ShortPeriodUNF^2 / Sol.n_AOA ;

% pick out the phugoid poles as being the 3rd and 4th rows
EigPHU = LongModes(3:4,:);

% following the same logic as above, pick out phugoid damping ratio and UNF
Sol.PhugoidDR = min(EigPHU.Zeta);
Sol.PhugoidUNF = min(EigPHU.UNF);


% CHECK AGAINST LONGITUDINAL HQ CONSTRAINTS
[c_long, long_conclusions, longweights] = LongStabilityConstraints(Sol);
Sol.long_conclusions = long_conclusions;
Sol.c_long = c_long;
Sol.c_long_weighted = max(0,c_long-tolerance).*longweights;


%%%%%% ANALYZE LATERAL MODES
LatVarNames = {'v','p','r','y','ph','ps'};
LatVarRef = {'ph'};

EVFactor = 2*Ref.Velocity/Ref.RefLatLength;
[LatModes] = AnalyzeModes(LinModel.Alatnd,EVFactor,LatVarNames,LatVarRef);
Sol.LatModes = LatModes;


% modes are arranged in increasing order of UNF

% find oscillatory modes
OscLatModes = LatModes(strcmpi(LatModes.Type,'oscillatory'),:);

% find non-oscillatory modes
NonOscLatModes = LatModes(strcmpi(LatModes.Type,'non-oscillatory'),:);


% if there are ***NO*** oscillatory modes
% typically, this will happen if the Dutch roll has become unstable
% can also mean that there will be directional divergence
if isempty(OscLatModes)
    Sol.c_lat = zeros(1,17); % keep size consistent
    Sol.c_lat(1:6) = - LatModes.Sigma(1:end)';
    Sol.lat_conclusions = [];
end


% if there are oscillatory modes
if ~isempty(OscLatModes)

    % identify the dutch roll characteristics
    Sol.DutchRollUNF = min(OscLatModes.UNF);
    Sol.DutchRollDR = min(OscLatModes.Zeta);
    Sol.DutchRollSigma = min(OscLatModes.Sigma);


    % of the non-oscillatory modes, the one with the largest magnitude is
    % the roll mode (end row of the table)
    Sol.RollTau = 1/NonOscLatModes.Sigma(end);

    % the penultimate row has the spiral modSol.SpiralPole e
    Sol.SpiralPole = - NonOscLatModes.Sigma(end-1);
    Sol.SpiralTTD = NonOscLatModes.TTHD(end-1);

    [c_lat, lat_conclusions] = LatStabilityConstraints(Sol);
    Sol.lat_conclusions = lat_conclusions;
    Sol.c_lat = c_lat;
    

end

Sol.c_lat_weighted = max(0,Sol.c_lat);
  


% % The smallest magnitude non-zero pole is the spiral
% Sol.SpiralPole = - LatModes.Sigma(3);
% 
% % count how many of the remaining 3 modes are non-oscillatory
% % possibilities are either all 3 or just 1
% RemNonOscModes = sum(strcmpi(LatModes.Type(4:end),'non-oscillatory'));
% 
% LatModes
% 
% if RemNonOscModes == 1
%     RemLatModes = LatModes(4:end,:);
%     % isolate rows with non-oscillatory modes (contains spiral and roll mode)
%     NonOscLatModes = RemLatModes(strcmpi(RemLatModes.Type,'non-oscillatory'),:);
% 
%     % the one with larger magnitude (last row of table) is the roll mode
%     Sol.RollTau = 1/NonOscLatModes.Sigma(end);
% 
%     % isolate rows with oscillatory modes (contains Dutch roll mode)
%     OscLatModes = RemLatModes(strcmpi(LatModes.Type(4:end),'oscillatory'),:);
% 
%     Sol.DutchRollUNF = min(OscLatModes.UNF);
%     Sol.DutchRollDR = min(OscLatModes.Zeta);
%     Sol.DutchRollSigma = min(OscLatModes.Sigma);
% 
%     % CHECK AGAINST LATERAL HQ CONSTRAINTS
%     [c_lat, lat_conclusions] = LatStabilityConstraints(Sol);
%     Sol.lat_conclusions = lat_conclusions;
%     Sol.c_lat = c_lat;
%     Sol.c_lat_weighted = max(0,c_lat);
% 
% end
% 
% if RemNonOscModes == 3
%     Sol.c_lat = zeros(1,17); % keep size consistent
%     Sol.c_lat(1:3) = - LatModes.Sigma(4:end)';
%     Sol.c_lat_weighted = max(0,Sol.c_lat);
%     Sol.lat_conclusions = [];
% end



end





function [c_long, conclusions, wvec] = LongStabilityConstraints(Sol)


ctr = 1;
% frame each one as an inequality type constraint

%%%% SHORT PERIOD, CATEGORY B, LEVEL 1
% 1. short period DR: Cat B, L1, min bound, DR > 0.30?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.ShortPeriodDR + 0.30;
wvec(ctr) = 5;
ctr = ctr + 1;

% 2. short period DR: Cat B, L1, max bound, DR < 2.00?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = Sol.ShortPeriodDR - 2.00;
wvec(ctr) = 5;
ctr = ctr + 1;

% 3. short period CAP: Cat B, L1, min bound, CAP > 0.085?
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.CAP + 0.085;
wvec(ctr) = 5;
ctr = ctr + 1;

% 4. short period CAP: Cat B, L1, max bound, CAP < 3.6?
c_long(ctr) = Sol.CAP - 3.6;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Cat B, L1 achieved?
conclusions.SP_CatB_L1 = all(c_long(1:4)<0);


%%%% SHORT PERIOD, CATEGORY B, LEVEL 2
% 5. short period DR: Cat B, L2, min bound, DR > 0.20?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.ShortPeriodDR + 0.20;
wvec(ctr) = 5;
ctr = ctr + 1;

% 6. short period DR: Cat B, L2, max bound, DR < 2.00?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = Sol.ShortPeriodDR - 2.00;
wvec(ctr) = 5;
ctr = ctr + 1;

% 7. short period CAP: Cat B, L1, min bound, CAP > 0.038?
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.CAP + 0.038;
wvec(ctr) = 5;
ctr = ctr + 1;

% 8. short period CAP: Cat B, L1, max bound, CAP < 10?
c_long(ctr) = Sol.CAP - 10;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Cat B, L2 achieved?
conclusions.SP_CatB_L2 = all(c_long(5:8)<0);






%%%% SHORT PERIOD, CATEGORY C, LEVEL 1
% 9. short period DR: Cat C, L1, min bound, DR > 0.35?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.ShortPeriodDR + 0.35;
wvec(ctr) = 5;
ctr = ctr + 1;

% 10. short period DR: Cat C, L1, max bound, DR < 1.30?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = Sol.ShortPeriodDR - 1.30;
wvec(ctr) = 5;
ctr = ctr + 1;

% 11. short period CAP: Cat C, L1, min bound, CAP > 0.16?
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.CAP + 0.16;
wvec(ctr) = 5;
ctr = ctr + 1;

% 12. short period CAP: Cat C, L1, max bound, CAP < 3.6?
c_long(ctr) = Sol.CAP - 3.6;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Cat C, L1 achieved?
conclusions.SP_CatC_L1 = all(c_long(9:12)<0);




%%%% SHORT PERIOD, CATEGORY C, LEVEL 2
% 13. short period DR: Cat C, L2, min bound, DR > 0.25?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.ShortPeriodDR + 0.25;
wvec(ctr) = 5;
ctr = ctr + 1;

% 14. short period DR: Cat C, L2, max bound, DR < 2.00?
% source: MIL-STD-8785C, Pamadi, Tbl 7.6
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = Sol.ShortPeriodDR - 2.00;
wvec(ctr) = 5;
ctr = ctr + 1;

% 15. short period CAP: Cat C, L2, min bound, CAP > 0.05?
% source: MIL-STD-1797A, Pamadi, Fig. 7.23
c_long(ctr) = - Sol.CAP + 0.05;
wvec(ctr) = 5;
ctr = ctr + 1;

% 16. short period CAP: Cat C, L2, max bound, CAP < 10?
c_long(ctr) = Sol.CAP - 10;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Cat C, L2 achieved?
conclusions.SP_CatC_L2 = all(c_long(13:16)<0);



%%%%% PHUGOID, LEVEL 1
% 17. phugoid DR > 0.04?
% source: MIL-F-8785C, Pamadi, Tbl 7.8
c_long(ctr) = - Sol.PhugoidDR + 0.04;
wvec(ctr) = 1;
conclusions.PH_L1 = c_long(17)<0;
ctr = ctr + 1;

%%%%% PHUGOID, LEVEL 2
% 18. phugoid DR > 0?
% source: MIL-F-8785C, Pamadi, Tbl 7.8
c_long(ctr) = - Sol.PhugoidDR;
wvec(ctr) = 1;
conclusions.PH_L2 = c_long(18)<0;

end



function [c_lat, conclusions, wvec] = LatStabilityConstraints(Sol)

ctr = 1;


%%%%%%%%%%% DUTCH ROLL MODE
% 1. Dutch Roll damping, Cat B, L1, DR > 0.08?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollDR + 0.08;
wvec(ctr) = 5;
ctr = ctr + 1;

% 2. Dutch Roll sigma, Cat B, L1, sigma > 0.15?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollSigma + 0.15;
wvec(ctr) = 5;
ctr = ctr + 1;

% 3. Dutch Roll UNF, Cat B, L1, UNF > 0.4?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollUNF + 0.4;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Dutch roll, Cat B, L1 achieved?
conclusions.DutchRoll_CatB_L1 = all(c_lat(1:3)<0);

% 4. Dutch Roll damping, Cat C, L1, DR > 0.08?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollDR + 0.08;
wvec(ctr) = 5;
ctr = ctr + 1;

% 5. Dutch Roll sigma, Cat C, L1, sigma > 0.15?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollSigma + 0.15;
wvec(ctr) = 5;
ctr = ctr + 1;

% 6. Dutch Roll UNF, Cat C, L1, UNF > 1?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollUNF + 1;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Dutch roll, Cat B, L1 achieved?
conclusions.DutchRoll_CatC_L1 = all(c_lat(4:6)<0);

% 7. Dutch Roll damping, Cat ALL, L2, DR > 0.02?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollDR + 0.02;
wvec(ctr) = 5;
ctr = ctr + 1;

% 8. Dutch Roll sigma, Cat ALL, L2, sigma > 0.05?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollSigma + 0.05;
wvec(ctr) = 5;
ctr = ctr + 1;

% 9. Dutch Roll UNF, Cat ALL, L2, UNF > 0.4?
% source: MIL-F-8785C, Pamadi Tbl 7.11
c_lat(ctr) = - Sol.DutchRollUNF + 0.4;
wvec(ctr) = 5;
ctr = ctr + 1;

% conclusion: Dutch roll, L2 achieved?
conclusions.DutchRoll_L2 = all(c_lat(7:9)<0);



%%%%%%%%%%%% ROLL MODE
% 10. Roll mode, Cat B, ALL, L1, time constant < 1.4?
% source: MIL-F-8785C, Pamadi, Tbl 7.9
c_lat(ctr) = Sol.RollTau - 1.4;
wvec(ctr) = 5;
conclusions.Roll_CatB_L1 = c_lat(ctr)<0;
ctr = ctr + 1;

% 11. Roll mode, Cat C, L1, time constant < 1.0?
% source: MIL-F-8785C, Pamadi, Tbl 7.9
c_lat(ctr) = Sol.RollTau - 1.0;
wvec(ctr) = 5;
conclusions.Roll_CatC_L1 = c_lat(ctr)<0;
ctr = ctr + 1;

% 12. Roll mode, Cat B, ALL, L2, time constant < 3.0?
% source: MIL-F-8785C, Pamadi, Tbl 7.9
c_lat(ctr) = Sol.RollTau - 3.0;
wvec(ctr) = 5;
conclusions.Roll_CatB_L2 = c_lat(ctr)<0;
ctr = ctr + 1;

% 13. Roll mode, Cat C, L2, time constant < 1.4?
% source: MIL-F-8785C, Pamadi, Tbl 7.9
c_lat(ctr) = Sol.RollTau - 1.4;
wvec(ctr) = 5;
conclusions.Roll_CatC_L2 = c_lat(ctr)<0;
ctr = ctr + 1;



%%%%%%%%%%%%%%%%% SPIRAL MODE
% 14. Spiral, CAT B, L1, TTD > 20 sec
% source: MIL-F-8785C, Pamadi, Table 7.10
c_lat(ctr) = Sol.SpiralPole - 0.693/20;
wvec(ctr) = 1;
conclusions.Spiral_CatB_L1 = c_lat(ctr)<0;
ctr = ctr + 1;

% 15. Spiral, CAT C, L1, TTD > 12 sec
% source: MIL-F-8785C, Pamadi, Table 7.10
c_lat(ctr) = Sol.SpiralPole - 0.693/12;
wvec(ctr) = 1;
conclusions.Spiral_CatC_L1 = c_lat(ctr)<0;
ctr = ctr + 1;

% 16. Spiral, CAT B, L2, TTD > 8 sec
% source: MIL-F-8785C, Pamadi, Table 7.10
c_lat(ctr) = Sol.SpiralPole - 0.693/8;
wvec(ctr) = 1;
conclusions.Spiral_CatB_L2 = c_lat(ctr)<0;
ctr = ctr + 1;

% 17. Spiral, CAT C, L2, TTD > 8 sec
% source: MIL-F-8785C, Pamadi, Table 7.10
c_lat(ctr) = Sol.SpiralPole - 0.693/8;
wvec(ctr) = 1;
conclusions.Spiral_CatC_L2 = c_lat(ctr)<0;


end














end