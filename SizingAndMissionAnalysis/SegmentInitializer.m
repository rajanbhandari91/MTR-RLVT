function [History] = SegmentInitializer(Segment,k,PropTypes, varargin)

global Settings

npts = Segment.Points;

History.Name = repmat({Segment.Name},[npts,1]);
History.SegNum = k*ones(npts,1);
History.Type = repmat({Segment.Type},[npts,1]);
History.Time = zeros(npts,1);
History.dTime = zeros(npts,1);
History.Alt = zeros(npts,1);
History.rho = zeros(npts,1);
History.EAS = zeros(npts,1);
History.TAS = zeros(npts,1);
History.GS = zeros(npts,1);
History.Mach = zeros(npts,1);
History.Ps = zeros(npts,1);
History.EnHt = zeros(npts,1);
History.Dist = zeros(npts,1);
History.dDist = zeros(npts,1);
History.dVdt = zeros(npts,1);
History.dhdt = zeros(npts,1);
History.AOA = zeros(npts,1);
History.FPA = zeros(npts,1);
History.dFPAdt = zeros(npts,1);
History.THROT = zeros(npts,PropTypes);
History.CTRL = zeros(npts,3);
History.Mass = zeros(npts,1);
History.EMass = zeros(npts,PropTypes);
History.dMass = zeros(npts,1);
History.dMdt = zeros(npts,PropTypes);
History.Energy = zeros(npts,PropTypes);
History.dEnergy = zeros(npts,PropTypes);
History.dEdt = zeros(npts,PropTypes);
History.ADDL = zeros(npts,length(Settings.AddlNames));
History.States = zeros(npts, length(Settings.StateNames));

if nargin>3
    History_Prev = varargin{1};

    History.Time(1) = History_Prev.Time(end);
    History.Alt(1) = History_Prev.Alt(end);
    History.TAS(1) = History_Prev.TAS(end);
    History.EAS(1) = History_Prev.EAS(end);
    History.Mach(1) = History_Prev.Mach(end);
    History.Dist(1) = History_Prev.Dist(end);
    History.Mass(1) = History_Prev.Mass(end);
    History.Energy(1,:) = History_Prev.Energy(end,:);
    History.rho(1) = History_Prev.rho(end);
    History.States(1,:) = History_Prev.States(end,:);
end



History = struct2table(History);