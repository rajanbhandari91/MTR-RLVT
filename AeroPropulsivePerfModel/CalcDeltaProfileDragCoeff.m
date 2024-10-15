function deltaCD_p = CalcDeltaProfileDragCoeff(LSName, LSIndices, CSName, CSIndices, CSType, CSChordRatio, def,Vehicle)
if strcmp(CSType,'Plain')
    delta_cdfpGI = Vehicle.Aero.PlainTEFlapGI.delta_cdfp;
end

if strcmp(CSType,'SingleSlotted')
    delta_cdfpGI = Vehicle.Aero.SingleSlottedTEFlapGI.delta_cdfp;
end

Sref = Vehicle.Aero.RefArea;
% Extract area from StripDef 
dA = Vehicle.Aero.StripDef.Area;

% IBD Flap
% 2-D profile drag increment
delcd_p_IBDFlaperon_L = delta_cdfpGI(CSChordRatio,abs(def));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_IBDFlaperon_L = delcd_p_IBDFlaperon_L* Vehicle.Aero.(LSName).(CSName).Kb * Vehicle.Geom.(LSName).ExposedPlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
index = LSIndices(CSIndices);
% distribution of the 3-D drag value to each of the strip proportional to the strip area
deltaCD_p = delCD_p_IBDFlaperon_L * dA(index)/ sum(dA(index));
end
