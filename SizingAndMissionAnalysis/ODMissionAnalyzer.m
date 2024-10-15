function [Mission] = ODMissionAnalyzer(Mission,Vehicle)


EnergyMass_kg = Mission.EnergyMass_kg;
Payload_kg = Mission.Payload_kg;
TOGM_kg = Vehicle.MassProp.OEMLessNCE_kg + Payload_kg + sum(EnergyMass_kg);



Mission.TOGM_kg = TOGM_kg;




PropTypes = Mission.PropTypes;




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

% Mission.Dists(1,4) = 100000;


SAR = Vehicle.Mission.SAR;

History = [];


DummySegment.Points = 1;
DummySegment.Name = 'blank';
DummySegment.Type = 'blank';
DummySegNum = 0;




ConvFlag = 0;
Iter = 0;
IterMax = 10;
DistThresh = 10;

[nt,~] = size(Mission.DistTargetIndex);

for i = 1:nt
Mission.Dists(Mission.DistTargetIndex(i,1),Mission.DistTargetIndex(i,2))  = sum(EnergyMass_kg .* SAR)/nt;
end

DistCorr = 0;

while ConvFlag==0 && Iter < IterMax
    
    
    [History] = SegmentInitializer(DummySegment,DummySegNum,PropTypes);
    
    
    
    History.Time(1) = 0;
    History.Alt(1) = 0;
    History.TAS(1) = 40;
    History.Dist(1) = 0;
    
    % set starting mass
    H.Mass(1) = TOGM_kg;
    History.Mass(1) = TOGM_kg;
    Mission.Segments(1).StartMass = TOGM_kg;
    
    
    % set starting energy
    History.Energy(1,:) = EnergyMass_kg.*Vehicle.SOTA.EnergyDensity_MJkg;
    History.EMass(1,:) = EnergyMass_kg;
    
    
    
    
    
    
    
    Iter = Iter + 1;
    
    for i = 1:nt
    Mission.Dists(Mission.DistTargetIndex(i,1),Mission.DistTargetIndex(i,2)) = Mission.Dists(Mission.DistTargetIndex(i,1),Mission.DistTargetIndex(i,2)) + DistCorr/nt;
    end
%     DistCorr
    d =  Mission.Dists(Mission.DistTargetIndex(1,1),Mission.DistTargetIndex(1,2));
    
    
    t = 1;
    
    fprintf('Trial dist = %0.2f km \n',d/1000);
    
    for i = 1:1:nseg
        
        
        %SegInd = find(H.SegNum==i);
        
        % climb segment
        switch S(i).Type
            
            case 'climb'
                SegmentHistory = ClimbSegmentEval(Mission,i,History,Vehicle);
                
                if i>1
                    History = [History;SegmentHistory];
                else
                    History = SegmentHistory;
                end
                
            case 'cruise'
                SegmentHistory = CruiseSegmentEval(Mission,i,History,Vehicle);
                History = [History;SegmentHistory];
                
            case 'descent'
                SegmentHistory = DescentSegmentEval(Mission,i,History,Vehicle,[]);
                History = [History;SegmentHistory];
                
            case 'loiter'
                SegmentHistory = LoiterSegmentEval(Mission,i,History,Vehicle);
                History = [History;SegmentHistory];
                
            case 'verticalflight'
                SegmentHistory = VerticalFlightEval(Mission,i,History,Vehicle);
                
                if i>1
                    History = [History;SegmentHistory];
                else
                    History = SegmentHistory;
                end
                
            case 'transition'
                SegmentHistory = TransitionEval(Mission,i,History,Vehicle);
                History = [History;SegmentHistory];           
                
        end
        
        History.dhdt = History.TAS.*sind(History.FPA)*60;
        
        [DistTable,TimeTable] = UpdateDistTable(History,DistTable,TimeTable,i);
        
        Mission.DistTable = DistTable;
        Mission.TimeTable = TimeTable;
        
        
    end
    
    
    % establish SAR
    diffEMass = History.EMass(1,:) -  History.EMass(end,:);
    SAR = History.Dist(end)./diffEMass;
    
    SAR(isinf(SAR)) = 0;
    
    ResEnergy = History.EMass(end,:) - (Mission.ResEnergyPerc/100).*History.EMass(1,:);
    DistCorr = sum(SAR .* ResEnergy);
    
    if abs(DistCorr)<DistThresh
        ConvFlag=1;
    end
    
    
end




History.MassFrac = History.Mass./History.Mass(1);
History.EFrac = History.Energy./(History.Energy(1,:) + 0.0000000001);

History.Dist = History.Dist/1000;
History.dDist = History.dDist/1000;


Mission.DistTable_km = DistTable/1000;
Mission.DistTable_NM = Mission.DistTable_km*0.539957;
Mission.TimeTable_min = TimeTable/60;

Mission.History = History;
AddlNames = {'PCP','PLR','N0','Nth','de','RPM','BP','GPH','etap','LD','CL12CD','DLR','FLAG'};
AddlTbl = array2table(History.ADDL);
AddlTbl.Properties.VariableNames = AddlNames;

History = [History,AddlTbl];



Mission.HistoryIU = History;


Mission.HistoryIU.Time = Mission.History.Time/60;                           % time in min
Mission.HistoryIU.dTime = Mission.History.dTime/60;                         % time in min
Mission.HistoryIU.Alt = Mission.History.Alt*3.28;                           % alt in ft
Mission.HistoryIU.rho = Mission.History.rho/1.225;                          % rho as ratio
Mission.HistoryIU.EAS = Mission.History.EAS* 1.94384;                       % speed in kt
Mission.HistoryIU.TAS = Mission.History.TAS* 1.94384;                       % speed in kt
Mission.HistoryIU.GS = Mission.History.GS* 1.94384;                         % speed in kt
Mission.HistoryIU.Ps = Mission.History.Ps*3.28*60;                          % in ft/min
Mission.HistoryIU.EnHt = Mission.History.EnHt*3.28;                         % in feet
Mission.HistoryIU.Dist = Mission.History.Dist*0.539957;                  % in NM
Mission.HistoryIU.dDist = Mission.History.dDist*0.539957;                % in NM
Mission.HistoryIU.dVdt = Mission.History.dVdt/9.81;                         % in g
Mission.HistoryIU.dhdt = Mission.History.dhdt*3.28*60;                      % in ft/min
Mission.HistoryIU.AOA = Mission.History.AOA*180/pi;                         % in deg
Mission.HistoryIU.FPA = Mission.History.FPA*180/pi;                         % in deg
Mission.HistoryIU.dFPAdt = Mission.History.dFPAdt*180/pi;                   % in deg/s
Mission.HistoryIU.Mass = Mission.History.Mass*2.20462;                      % in lb
Mission.HistoryIU.EMass = Mission.History.EMass*2.20462;                     % in lb
Mission.HistoryIU.dMass = Mission.History.dMass*2.20462;                    % in lb
Mission.HistoryIU.dMdt = Mission.History.dMdt*2.20462;                      % in lb/sec



Mission.Range_km = History.Dist(end);
Mission.Range_NM = Mission.HistoryIU.Dist(end);
Mission.Time_min = Mission.HistoryIU.Time(end);
Mission.ConvFlag = ConvFlag;





