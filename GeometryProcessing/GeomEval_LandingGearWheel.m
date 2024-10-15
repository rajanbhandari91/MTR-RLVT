function [Wheel] =  GeomEval_LandingGearWheel(Wheel)

if Wheel.Directionality == 1
    Wheel.RefPtLocation(2) = Wheel.RefPtLocation(2) - 0.5*Wheel.Span;
end
if Wheel.Directionality == -1
    Wheel.RefPtLocation(2) = Wheel.RefPtLocation(2) + 0.5*Wheel.Span;
end
[Wheel] = GeomEval_LiftingSurface(Wheel);



