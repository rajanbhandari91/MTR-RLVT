function [Mission] = Initializer(Mission)
% Count the number of defined mission segments and then initialize each
% segment with the necessary data fields. These fields will be filled in
% durign the mission analysis


% count number of mission segments
[~,n] = size(Mission.Segments);


npts = 0;

Name = '';
Type = '';
SegNum = [];

for i = 1:1:n
    
    if(isempty(Mission.Segments(i).Points))
        Mission.Segments(i).Points = 10;
    end
    AddName = repmat({Mission.Segments(i).Name},Mission.Segments(i).Points,1);
    Name = [Name;AddName];
    
    AddType = repmat({Mission.Segments(i).Type},Mission.Segments(i).Points,1);
    Type = [Type;AddType];
    
    SegNum = [SegNum;repmat(i,Mission.Segments(i).Points,1)];

    npts = npts + Mission.Segments(i).Points;
        
end

PropTypes = Mission.PropTypes;


Mission.History.Name = Name;
Mission.History.SegNum = SegNum;
Mission.History.Type = Type;
Mission.History.Time = zeros(npts,1);
Mission.History.dTime = zeros(npts,1);
Mission.History.Alt = zeros(npts,1);
Mission.History.rho = zeros(npts,1);
Mission.History.EAS = zeros(npts,1);
Mission.History.TAS = zeros(npts,1);
Mission.History.GS = zeros(npts,1);
Mission.History.Mach = zeros(npts,1);
Mission.History.Ps = zeros(npts,1);
Mission.History.EnHt = zeros(npts,1);
Mission.History.Dist = zeros(npts,1);
Mission.History.dDist = zeros(npts,1);
Mission.History.dVdt = zeros(npts,1);
Mission.History.dhdt = zeros(npts,1);
Mission.History.AOA = zeros(npts,1);
Mission.History.FPA = zeros(npts,1);
Mission.History.dFPAdt = zeros(npts,1);
Mission.History.THROT = zeros(npts,PropTypes);
Mission.History.CTRL = zeros(npts,3);
Mission.History.Mass = zeros(npts,1);
Mission.History.dMass = zeros(npts,1);
Mission.History.dMdt = zeros(npts,PropTypes);
Mission.History.Energy = zeros(npts,PropTypes);
Mission.History.dEnergy = zeros(npts,PropTypes);
Mission.History.dEdt = zeros(npts,PropTypes);










% stand-in for optimal climb results
[GW,He] = ndgrid([2000,2300,2700,3500]*0.4536,[0,2000,4000,6000]);
Vopt = fliplr([42,42,42,42;41,41,41,41;40,40,40,40;39,39,39,39]);


% load GI_TAS_fn_GW_He

Mission.Optim.Climb.TAS_fn_GW_He = griddedInterpolant(GW,He,Vopt,'linear','nearest');

Mission.Optim.Descent.TAS_fn_GW_He = griddedInterpolant(GW,He,Vopt,'linear','nearest');


% stand-in for optimal cruise results
GWc = [2000,2300,2700,3500]*0.4536;
Vcopt = [140,137,135,130]*0.51444;
hcopt = [9000,8000,7000,6000]/3.28;
Mission.Optim.Cruise.TAS_fn_GW = griddedInterpolant(GWc,Vcopt,'linear','nearest');
Mission.Optim.Cruise.Alt_fn_GW = griddedInterpolant(GWc,hcopt,'linear','nearest');


Mission.Optim.Loiter.TAS_fn_GW = griddedInterpolant(GWc,Vcopt,'linear','nearest');
Mission.Optim.Loiter.Alt_fn_GW = griddedInterpolant(GWc,hcopt,'linear','nearest');

Mission.History = struct2table(Mission.History);
















