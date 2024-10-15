function [LS] = GeomEval_InitLiftingSurface(varargin)


LS.Name = [];
if nargin>0
    LS.Name = {varargin{1}};
end


LS.Type = 'liftingsurface';
LS.PlanformArea = 1;
LS.AspectRatio = 4;
LS.Directionality = 1;
LS.TaperDefn = [];
LS.SweepDefn = [];
LS.Dihedral = [];
LS.Twist = [];
LS.t_min = [];
LS.RootIncidence = 0;
LS.RootDihedral = 0;
LS.RefPtLocation = [0;0;0];
LS.RefPtChordFrac = 0.25;
LS.ExposedEtas = [0,1];
LS.SpecEtas = [];
LS.BlockedEtas = [];
LS.AirfoilName{1}.name= 'symmetricAF';
LS.AirfoilEta = [0 1];
LS.Controls = [];
LS.PropEtas = [];
LS.PropDiams = [];
LS.Span =sqrt(LS.PlanformArea*LS.AspectRatio);
LS.ProjectedSpan = [];
LS.ProjectedAspectRatio = [];
LS.t_min = [];
LS.t_max = [];
LS.eta_CG_spec = [];
LS.chordfrac_CG_spec = [];
LS.Mass = 0.0000000000000000001;
LS.Architecture = {'-'};
LS.CG = [0 0 0];
LS.Ixx = 0;
LS.Iyy = 0;
LS.Izz = 0;
LS.Ixy = 0;
LS.Iyz = 0;
LS.Izx = 0;

LS.eta = [];    % leave empty
LS.StripDist = []; % leave empty

LS.EtaMounting = 0;                 % spanwise mounting of another lifting surface onto this one. Default = 0
LS.StripMaxWidth = 0.2; 
LS.GenUniformSpacing = 0;
LS.Stn = [];