function [StatePenalty] = StatePenalty(StateVec,FltCon)
global Vehicle

StatePenalty = 0;

[FltCon] = CompleteFltCon(FltCon);


if FltCon.KEAS<=Vehicle.Aero.VThreshold/0.51444
    AOA = StateVec(1);
    
    StatePenalty = 100*(AOA)^2;
    
end