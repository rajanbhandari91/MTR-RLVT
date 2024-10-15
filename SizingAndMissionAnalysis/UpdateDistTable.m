function [DT,TT] = UpdateDistTable(History,DT,TT,i)

[nr,nc] = size(DT);

DT(1,i+1) = History.Dist(end);
TT(1,i+1) = History.Time(end);

for k = 1:1:i
    
   for m =  i+1:nc
    
       DT(k,m) =  DT(1,m)-DT(1,k);
       TT(k,m) =  TT(1,m)-TT(1,k);
   end
    
end