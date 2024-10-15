function [OutRow] = InterpolateBetweenTableRows(R1,R2,f)


OutRow = R2;

OutRow.Time = lininterp(R1.Time,R2.Time,f);
OutRow.Alt = lininterp(R1.Alt,R2.Alt,f);
OutRow.rho = lininterp(R1.rho,R2.rho,f);
OutRow.EAS = lininterp(R1.EAS,R2.EAS,f);
OutRow.TAS = lininterp(R1.TAS,R2.TAS,f);
OutRow.GS = lininterp(R1.GS,R2.GS,f);
OutRow.Mach = lininterp(R1.Mach,R2.Mach,f);
OutRow.Ps = lininterp(R1.Ps,R2.Ps,f);
OutRow.EnHt = lininterp(R1.EnHt,R2.EnHt,f);
OutRow.Dist = lininterp(R1.Dist,R2.Dist,f);

OutRow.AOA = lininterp(R1.AOA,R2.AOA,f);
OutRow.ADDL = lininterp(R1.ADDL,R2.ADDL,f);
OutRow.FPA = lininterp(R1.FPA,R2.FPA,f);
OutRow.THROT = lininterp(R1.THROT,R2.THROT,f);
OutRow.CTRL = lininterp(R1.CTRL,R2.CTRL,f);
OutRow.Mass = lininterp(R1.Mass,R2.Mass,f);
OutRow.EMass = lininterp(R1.EMass,R2.EMass,f);
OutRow.dMdt = lininterp(R1.dMdt,R2.dMdt,f);
OutRow.Energy = lininterp(R1.Energy,R2.Energy,f);
OutRow.dEdt = lininterp(R1.dEdt,R2.dEdt,f);






    function [opval] = lininterp(a,b,f)
        
        opval = a + (b-a)*f;
        
    end



end