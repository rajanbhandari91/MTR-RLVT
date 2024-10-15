function [ModeDesc] = AnalyzeModes(A,EVFactor,VarNames,VarRef)

% find the eigenvalues of the system matrix
[V,L] = eig(A,'vector');

% capture the eigenvalues from the diagonal elements of L
%L = diag(Lmatrix);


% get the norms of the eigenvalues
NormL = sqrt(real(L).^2 + imag(L).^2);

% sort based on magnitude
[NormL,sortindex] = sort(NormL);


V = V(:,sortindex);
L = L(sortindex);


nmodes = length(L);

ModeDesc = table();

[~,ind] = find(strcmpi(VarNames,VarRef));

for i=1:1:nmodes
    Lsel = L(i) * EVFactor;
    Sigma(i,1) = -real(Lsel);
    UNF(i,1) = norm(Lsel);
    DNF(i,1) = imag(Lsel);
    
    Stability(i,1) = {'-'};
    if imag(Lsel)==0
        Type(i,1) = {'Non-oscillatory'};
    end
    if imag(Lsel)~=0
        Type(i,1) = {'Oscillatory'};
    end
    
    if real(Lsel)<0
        Stability(i,1) = {'Stable'};
        
    end
    if real(Lsel)>0
        Stability(i,1) = {'Unstable'};
    end
    if real(Lsel)==0
        Stability(i,1) = {'Neutral'};
    end
    
    Eigenvalue(i,1) = Lsel;
    
    Zeta(i,1) = Sigma(i,1)/UNF(i,1);
    
    T(i,1) = 2*pi/DNF(i,1);
    
    TTHD(i,1) = log(2)/abs(Sigma(i,1));
    
    NTHD(i,1) = 0.110*DNF(i,1)/abs(Sigma(i,1));
    
    Zeta(isnan(Zeta))=0;
    TTHD(isnan(TTHD))=0; TTHD(isinf(TTHD))=0;
    NTHD(isnan(NTHD))=0; NTHD(isinf(NTHD))=0;
    T(isinf(T))=0; T(isnan(T))=0;
    
    
    % capture the eigenvalue
    Vana = V(:,i);
    
    MAG = sqrt(real(Vana).^2 + imag(Vana).^2);
    PHASE = atan2d(imag(Vana),real(Vana));

    if MAG(ind)~=0
        MAG = MAG/MAG(ind);
        PHASE = PHASE - PHASE(ind);
    end
    
    if MAG(ind)==0
        [~,indi] = max(MAG);
        MAG = MAG/MAG(indi);
        PHASE = PHASE - PHASE(indi);
    end
    
    nm = '';
    
    for j = 1:1:6
        
        nm = strcat(nm,sprintf(' %s: %0.4f/%0.2f, ',VarNames{j},MAG(j),PHASE(j)));
        
    end
    
    Description(i,1) = {nm};
end

ModeDesc.Type = Type;
ModeDesc.Eigenvalue = Eigenvalue;
ModeDesc.Stability = Stability;
ModeDesc.Sigma = Sigma;
ModeDesc.DNF = DNF;
ModeDesc.UNF = UNF;
ModeDesc.Zeta = Zeta;
ModeDesc.T = T;
ModeDesc.TTHD = TTHD;
ModeDesc.NTHD = NTHD;

ModeDesc.Description = Description;

% ModeDesc = ModeDesc(imag(ModeDesc.Eigenvalue)>=0,:);

