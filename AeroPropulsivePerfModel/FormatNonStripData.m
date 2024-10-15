
data = xlsread(['Non-Lifting.xlsx'],'NSG');

forces = data(1:end,[4 5 6]);

moments = data(1:end,[10 11 12]);


CX = forces(:,1); CY = forces(:,2); CZ = forces(:,3);


Cmx = moments(:,1); Cmy = moments(:,2); Cmz = moments(:,3);


% Expects column vectors with the following names in the workspace
% CX, CY, CZ, Cmx, Cmy, Cmz

[AOAgrid, BETAgrid] = ndgrid([-90:5:90],[0:5:90]);

[r,c] = size(AOAgrid);

% CX force coefficient
CXTable = reshape(CX,[c,r])';
CXTable = CXTable(:,:)

% CY force coefficient
CYTable = reshape(CY,[c,r])';
CYTable = CYTable - CYTable(:,1)        % enforce symmetry

% CZ force coefficient
CZTable = reshape(CZ,[c,r])'

% Cmx moment coefficient
CmxTable = reshape(Cmx,[c,r])';
CmxTable = CmxTable - CmxTable(:,1)     % enforce symmetry

% Cmy moment coefficient
CmyTable = reshape(Cmy,[c,r])'

% Cmz moment coefficient
CmzTable = reshape(Cmz,[c,r])';
CmzTable = CmzTable - CmzTable(:,1)     % enforce symmetry



if anynan(forces) || anynan(moments)
    
    fprintf('\n\n....interpolate the unconverged coefficients...\n\n\n')

    [IDNanForces,~] = find(isnan(forces))
    [IDNanMoments,~] = find(isnan(moments))

end

