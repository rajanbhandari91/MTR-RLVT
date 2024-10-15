function X = ConvertBAtoSAorSAtoBA(X_ba, X_sa, angle)

AOA = angle;

% SA to BA rotation tensor 
RBS = [cos(AOA) 0 -sin(AOA);
           0    1     0    ;
       sin(AOA) 0  cos(AOA)];


if isempty(X_sa)
    if size(X_ba) == [3 1]
        X_sa = RBS'*X_ba ;
    else
        X_sa = RBS'*X_ba*RBS;
    end
    X = X_sa;
else
    if size(X_sa) == [3 1]
        X_ba = RBS*X_sa ;
    else
        X_ba = RBS*X_sa*RBS';
    end
    X = X_ba;
end


end