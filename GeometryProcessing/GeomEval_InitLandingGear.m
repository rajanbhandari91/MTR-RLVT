function [LG, WP, Wheel] = GeomEval_InitLandingGear(varargin)
WP = [];
Wheel = [];



% LG.Name = [];
% if nargin>0
%     LG.Name = {varargin{1}};
% end


% first initialize as a lifting surface
[LG] = GeomEval_InitLiftingSurface(varargin{1});

% then make some alterations
LG.Type = 'landinggear';
LG.RootDihedral = -90;
LG.StrutChord = [];
LG.StripMaxWidth = 0.1;
LG.GearTrack = 2;


% if a wheelpant is to be defined...
if nargin > 1
    LG.HasWheelPant = 1;

    WP_name =  varargin{2};

    [WP] = GeomEval_InitFuselage(WP_name);

    WP.LineDef = 'Generic_WheelPant_Lines';
    WP.RefPt_FS = 0.30;
    WP.CrossSections(1).Name = 'CS1';
    WP.CrossSections(1).Defn = 'CS_rect';

    WP.MaxDistBetweenStations = 0.03;

    WP.FinenessRatio = 4;

    % wheel
    Wheel_name =  varargin{3};

    [Wheel] = GeomEval_InitLiftingSurface(Wheel_name);
    
    Wheel.Type = 'LG_Wheel';
    Wheel.Directionality = 1;
    Wheel.TaperDefn = 1;
    Wheel.SweepDefn = [0,1;0.5,0.5;0,0];
    Wheel.t_min = [];
    Wheel.RootDihedral = 0;
    Wheel.AirfoilName = 'LG_Circ_CS';
    Wheel.Dihedral = 0;
    Wheel.Twist = 0;

    Wheel.StripMaxWidth = 0.01;
    Wheel.RefPtLocation = [0,0,0];

end



