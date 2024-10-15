function [deltaCD_p] = EMPAERO_CalcdeltaProfileDrag(Aero,Geom, WingIndices, CSIndices, CSFCR, def_deg, delta_cdfpGI, Vehicle, CSName)
Sref = Vehicle.Aero.RefArea;
dA = Vehicle.Aero.StripDef.Area;
delcd_p_IBDFlaperon_L = delta_cdfpGI(CSFCR,abs(def_deg));
% 2-D to 3-D conversion (also normalized by reference area)
delCD_p_IBDFlaperon_L = delcd_p_IBDFlaperon_L* Aero.(CSName).Kb * Geom.PlanformArea/Sref;
% indices of the lifting surface on which the control surface lies
index = WingIndices(CSIndices);
% distribution of the 3-D drag value to each of the strip proportional to the strip area
deltaCD_p = delCD_p_IBDFlaperon_L * dA(index)/ sum(dA(index));
end