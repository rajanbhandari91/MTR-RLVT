function [Vehicle] = StabilizerGeometryCalculations(Vehicle,Fus_HTail,Fus_VTail)

Vehicle.Geom.RHTail.RefPtChordFrac = 1;
Vehicle.Geom.VTail.RefPtChordFrac = 1;
% size horizontal based on volume ratio
C_HT = Vehicle.DesignPoint.HTVolumeRatio;
Vehicle.Recalcs.HTailArea_TOT_m2 = (C_HT * Vehicle.Geom.LWing.MAC * Vehicle.Recalcs.WingArea_TOT_m2)/Vehicle.Recalcs.HTailMomentArm_m; % 1.0*c_bar*S_w/L_VT
Vehicle.Recalcs.HTailArea_Each_m2 = Vehicle.Recalcs.HTailArea_TOT_m2 / 2;

% find horizontal stab span (each)
Vehicle.Recalcs.HorzStabSpan_m = sqrt(Vehicle.Recalcs.HTailArea_TOT_m2*Vehicle.DesignPoint.HTAspectRatio);
Vehicle.Geom.LHTail.PlanformArea = Vehicle.Recalcs.HTailArea_Each_m2;
Vehicle.Geom.RHTail.PlanformArea = Vehicle.Recalcs.HTailArea_Each_m2;

%%%%% VERTICAL TAIL DIMENSIONS
% size vertical based on volume ratio
C_VT = Vehicle.DesignPoint.VTVolumeRatio;
Vehicle.Recalcs.VTailArea_TOT_m2 = (C_VT * Vehicle.Geom.LWing.MAC * Vehicle.Recalcs.WingArea_TOT_m2)/Vehicle.Recalcs.VTailMomentArm_m; % 1.0*c_bar*S_w/L_VT
Vehicle.Recalcs.VTailArea_Each_m2 = Vehicle.Recalcs.VTailArea_TOT_m2;

% find horizontal stab span (each)
Vehicle.Recalcs.VertStabSpan_m = sqrt(Vehicle.Recalcs.VTailArea_TOT_m2*Vehicle.DesignPoint.VTAspectRatio);
Vehicle.Geom.VTail.PlanformArea = Vehicle.Recalcs.VTailArea_Each_m2;

% RHTail
% Get the root chord
croot = 2*Vehicle.Recalcs.HTailArea_TOT_m2/ (Vehicle.Recalcs.HorzStabSpan_m * ( 1 + Vehicle.Geom.RHTail.TaperDefn(2,2)));
x_HTail = Vehicle.Geom.Prop_5.RefPtLocation(1) - 0.5*Vehicle.Geom.Prop_5.Diam - 0.2;
if abs(x_HTail) > (abs(Fus_HTail(1)) - 1.1*croot)
        Vehicle.Geom.Fuselage.CrossSections(end).Length = Vehicle.Geom.Fuselage.CrossSections(end).Length + abs(x_HTail) - (abs(Fus_HTail(1)) - 1.1*croot);
end
Vehicle.Geom.RHTail.RefPtLocation(1) = Fus_HTail(1);
Vehicle.Geom.RHTail.RefPtLocation(3) = Fus_HTail(2) - 0.25*Fus_HTail(3);

Vehicle.Geom.RHTail.PlanformArea = Vehicle.Recalcs.HTailArea_Each_m2;
Vehicle.Geom.RHTail.AspectRatio = 0.5*Vehicle.DesignPoint.HTAspectRatio;
[Vehicle.Geom.RHTail] = GeomEval_LiftingSurface(Vehicle.Geom.RHTail);

Vehicle.Geom.LHTail = Vehicle.Geom.RHTail;
Vehicle.Geom.LHTail.Name = {'Left Horizontal Tail'};
Vehicle.Geom.LHTail.Directionality = - Vehicle.Geom.RHTail.Directionality;
Vehicle.Geom.LHTail = GeomEval_LiftingSurface(Vehicle.Geom.LHTail);


% VTail
% ctipVS = 2*Vehicle.Recalcs.VTailArea_TOT_m2 * Vehicle.Geom.VTail.TaperDefn(2,end)/ (Vehicle.Recalcs.VertStabSpan_m * ( 1 + Vehicle.Geom.VTail.TaperDefn(2,end)));
Vehicle.Geom.VTail.RefPtLocation(1) = Fus_HTail(1);
Vehicle.Geom.VTail.RefPtLocation(3) = Fus_VTail(2) - 0.25*Fus_VTail(3);
Vehicle.Geom.VTail.PlanformArea = Vehicle.Recalcs.VTailArea_Each_m2;
Vehicle.Geom.VTail.AspectRatio = Vehicle.DesignPoint.VTAspectRatio;
Vehicle.Geom.VTail = GeomEval_LiftingSurface(Vehicle.Geom.VTail);

end