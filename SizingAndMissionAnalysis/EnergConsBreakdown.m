% This script is intended to capture and display the mission energy
% consumption broken down over the mission segments



M = Vehicle.Mission.HistoryIU;



nseg = length(unique(M.Name))

for i = 1:nseg
    
    
   MC = M(M.SegNum == i,:);
   
   SegName(i,1) = MC.Name(1,1);
   
   EF1_i(i,1) = MC.EFrac(1,1);
   EF1_f(i,1) = MC.EFrac(end,1);
   dEF1(i,1) = EF1_f(i,1) - EF1_i(i,1);
    
   
   EF2_i(i,1) = MC.EFrac(1,2);
   EF2_f(i,1) = MC.EFrac(end,2);
   dEF2(i,1) = EF2_f(i,1) - EF2_i(i,1);
   
end


EFSummary.Name = SegName;

EFSummary.EF1_i = EF1_i;
EFSummary.EF1_f = EF1_f;
EFSummary.dEF1 = dEF1;


EFSummary.EF2_i = EF2_i;
EFSummary.EF2_f = EF2_f;
EFSummary.dEF2 = dEF2;

EFSummary = struct2table(EFSummary)