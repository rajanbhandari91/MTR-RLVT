function [Mission, Vehicle] = MissionAnalyzer(Mission,Vehicle)

global Settings

PropTypes = Mission.PropTypes;                                              % capture number of propulsor types

if ~isfield(Mission,'History')
    Mission = Initializer(Mission);
end


[npts,~] = size(Mission.History);
[~,nseg] = size(Mission.Segments);

H = Mission.History;
S = Mission.Segments;

H.THROT = zeros(npts,PropTypes);
H.dmdt = zeros(npts,PropTypes);
H.dEdt = zeros(npts,PropTypes);

DistTable = zeros(nseg+1,nseg+1);
TimeTable = zeros(nseg+1,nseg+1);

% Step FORWARD through the mission
History = [];


DummySegment.Points = 1;
DummySegment.Name = 'blank';
DummySegment.Type = 'blank';
DummySegNum = 0;

[History] = SegmentInitializer(DummySegment,DummySegNum,PropTypes);

History.Time(1) = 0;
History.Alt(1) = S(1).StartAlt;
History.TAS(1) = 0;
History.Dist(1) = 0;

% set starting mass
Mission.TOGM_kg = Vehicle.MassProp.TOGM_kg;
H.Mass(1) = Mission.TOGM_kg;
History.Mass(1) = Mission.TOGM_kg;

% set starting energy
Mission.StartEnergy_MJ  = Mission.StartEnergyMass_kg.*Vehicle.SOTA.SpecificEnergy_MJkg .* (Mission.StartEnergyPerc/100);
History.Energy(1,:) = Mission.StartEnergy_MJ;
History.Emass(1,:) = Mission.StartEnergyMass_kg;



Vehicle.States = Vehicle.DefaultStartStates;
History.States = Vehicle.States;


fprintf('\n Running %s\n', Mission.Name)

for i = 1:1:nseg


    SegInd = find(H.SegNum==i);

    % climb segment
    switch S(i).Type

        case 'climb'
            [SegmentHistory, Vehicle] = SegEval_Climb(Mission,i,History,Vehicle);

            if i>1
                History = [History;SegmentHistory];
            else
                History = SegmentHistory;
            end

        case 'cruise'
            [SegmentHistory, Vehicle] = SegEval_Cruise(Mission,i,History,Vehicle);
            History = [History;SegmentHistory];

        case 'descent'
            [SegmentHistory, Vehicle] = SegEval_Descent(Mission,i,History,Vehicle);
            History = [History;SegmentHistory];

        case 'loiter'
            [SegmentHistory, Vehicle] = SegEval_Loiter(Mission,i,History,Vehicle);
            History = [History;SegmentHistory];

        case 'verticalflight'
            [SegmentHistory, Vehicle] = SegEval_VertFlight(Mission,i,History,Vehicle);

            if i>1
                History = [History;SegmentHistory];
            else
                History = SegmentHistory;
            end

        case 'transition'
            [SegmentHistory, Vehicle] = SegEval_Transition(Mission,i,History,Vehicle);
            History = [History;SegmentHistory];


    end

    History.dhdt = History.TAS.*sin(History.FPA);

    [DistTable,TimeTable] = UpdateDistTable(History,DistTable,TimeTable,i);

    Mission.DistTable = DistTable;




end


% put names for additional parameters
AddlNames = Settings.AddlNames;
AddlTbl = array2table(History.ADDL);
AddlTbl.Properties.VariableNames = AddlNames;
History = [History,AddlTbl];
History = removevars(History,{'ADDL'});

% put names for states
StateNames = Settings.StateNames;
StatesTbl = array2table(History.States);
StatesTbl.Properties.VariableNames = StateNames;
History = [History,StatesTbl];
History = removevars(History,{'States'});



diffEMass = History.EMass(1,:) -  History.EMass(end,:);
SAR = History.Dist(end)./diffEMass;
SAR(isinf(SAR)) = 0;

History.MassFrac = History.Mass./History.Mass(1);
History.EFrac = History.Energy./(History.Energy(1,:) + 1e-20);

History.Dist = History.Dist/1000;
History.dDist = History.dDist/1000;


Mission.DistTable_km = DistTable/1000;
Mission.DistTable_NM = Mission.DistTable_km*0.539957;
Mission.TimeTable_min = TimeTable/60;



Mission.History = History;
Mission.HistoryIU = History;

Mission.HistoryIU.Time = Mission.History.Time/60;                           % time in min
Mission.HistoryIU.dTime = Mission.History.dTime/60;                         % time in min
Mission.HistoryIU.Alt = Mission.History.Alt*3.28;                           % alt in ft
Mission.HistoryIU.rho = Mission.History.rho/1.225;                          % rho as ratio
Mission.HistoryIU.EAS = Mission.History.EAS/ 0.51444;                       % speed in kt
Mission.HistoryIU.TAS = Mission.History.TAS/ 0.51444;                       % speed in kt
Mission.HistoryIU.GS = Mission.History.GS/ 0.51444;                         % speed in kt
Mission.HistoryIU.Ps = Mission.History.Ps*3.28*60;                          % in ft/min
Mission.HistoryIU.EnHt = Mission.History.EnHt*3.28;                         % in feet
Mission.HistoryIU.Dist = Mission.History.Dist*0.539957;                     % in NM
Mission.HistoryIU.dDist = Mission.History.dDist*0.539957;                   % in NM
Mission.HistoryIU.dVdt = Mission.History.dVdt/9.81;                         % in g
Mission.HistoryIU.dhdt = Mission.History.dhdt*3.28*60;                      % in ft/min
Mission.HistoryIU.AOA = Mission.History.AOA*180/pi;                         % in deg
Mission.HistoryIU.FPA = Mission.History.FPA*180/pi;                         % in deg
Mission.HistoryIU.dFPAdt = Mission.History.dFPAdt*180/pi;                   % in deg/s
Mission.HistoryIU.Mass = Mission.History.Mass*2.20462;                      % in lb
Mission.HistoryIU.EMass = Mission.History.EMass*2.20462;                    % in lb
Mission.HistoryIU.dMass = Mission.History.dMass*2.20462;                    % in lb
Mission.HistoryIU.dMdt = Mission.History.dMdt*2.20462;                      % in lb/sec

Mission.SAR = SAR;



Mission = MissionPostProcess(Mission);


Vehicle.Mission = Mission;



