function [LG, WP, Wheel] =  GeomEval_LandingGear(LG, varargin)

WP = [];
Wheel = [];

if nargin>1
    WP = varargin{1};
    Wheel = varargin{2};
end

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


