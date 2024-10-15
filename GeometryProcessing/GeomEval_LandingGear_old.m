function [LG, WP, Wheel] =  GeomEval_LandingGear(LG, varargin)

WP = [];
Wheel = [];

if nargin>1
    WP = varargin{1};
    Wheel = varargin{2};
end


% Check if the landing gear leg is on centerline
CenteredLG = LG.RefPtLocation(2)==0;


% If it is not on centerline
if ~CenteredLG

    % set directionality based on y-coordinate of mounting point
    LG.Directionality = sign(LG.RefPtLocation(2));

    % calculate gear projection in vertical direction
    dz = LG.GearHeight;
    % calculate gear projection in lateral direction
    dy = LG.GearTrack/2 - abs(LG.RefPtLocation(2));
    % calculate root dihedral
    %     LG.RootDihedral = -atand(dz/dy);
    LG.RootDihedral = 0;

    % calculate leg length
    %     LG.LegLength = sqrt(dz^2+dy^2);


end


% If it is on centerline
if CenteredLG
    LG.LegLength = LG.GearHeight;
end


if ~isempty(LG.StrutChord)
    LG.AspectRatio = LG.LegLength/LG.StrutChord;
end

LG.PlanformArea = LG.LegLength^2/LG.AspectRatio;



[LG] = GeomEval_LiftingSurface(LG);

if LG.HasWheelPant

    WP.RefPtLocation = [LG.Stn.xQC(end), LG.Stn.yQC(end), LG.Stn.zQC(end)];

    WP.CrossSections(1).Length = 0.008;

    [WP] = GeomEval_Fuselage(WP);
end

if LG.HasWheel  
    
    Wheel.AspectRatio = Wheel.Span / Wheel.Chord;
    Wheel.PlanformArea = Wheel.Span^2 / Wheel.AspectRatio;
     
    Wheel.RefPtLocation = [LG.Stn.xcg(end) + Wheel.Chord/4, LG.Stn.yQC(end), LG.Stn.zQC(end)];
    [Wheel] = GeomEval_LiftingSurface(Wheel);
end


