function [Setup] = CompleteRotorSetup(PropDef,QMIL, nRotor, nrad, npsi, Setup)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROTOR SETUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% if "nrad" is a single value, then it refers to the number of radial
% stations desired. Space these out equally
if length(nrad)==1
    Setup.nrad = nrad;
    Setup.r = linspace(QMIL.r(1),QMIL.r(end),nrad+1); 
end

% if "nrad" contains more than one value, these are interpreted as
% normalized radial stations corresponding to midpoints of blade elements
if length(nrad)>1
   rnorm = nrad;
   Setup.nrad = length(nrad); 
   r_mids = rnorm * QMIL.r(end);
   dr_edges = diff(r_mids);
   Setup.r = [ r_mids(1) - dr_edges(1)/2,  r_mids(1:end-1) + dr_edges/2, r_mids(end) + dr_edges(end)/2];
end



Setup.nRotor = nRotor;                          % number of rotors

Setup.nBlades = PropDef.Nblades;                          % number of blades per rotor
Setup.RotorRadius = QMIL.r(end);                % rotor radius (m)
Setup.RotorLoc(:,:) = zeros(3,Setup.nRotor);
Setup.RotorAxisTheta = 0*ones(1,Setup.nRotor);
Setup.RotorAxisPhi = 0*ones(1,Setup.nRotor);

Setup.PsiStep = 360/npsi;


Setup.RootCutOutFraction = PropDef.HubRadiusRatio;
Setup.SpinDir = ones(1,nRotor);
Setup.HealthStatus =  ones(1,nRotor);
Setup.PropDef = PropDef;

RefSetup.r = QMIL.r';
RefSetup.PitchDistDeg = QMIL.relbeta';  % so that pitch of 75%R = 0 in reference condition
RefSetup.ChordDist = QMIL.c';



Setup.PitchDistDeg = interp1(RefSetup.r, RefSetup.PitchDistDeg, Setup.r, 'pchip','extrap');
Setup.ChordDist = interp1(RefSetup.r, RefSetup.ChordDist, Setup.r, 'pchip','extrap');


Setup.DiskArea = pi*Setup.RotorRadius^2;    % m2, rotor disk area
Setup.REdges = zeros(1,Setup.nrad+1);

Setup.REdges = Setup.r;

Setup.dR = diff(Setup.REdges);
Setup.RMids = Setup.REdges(1:end-1) + Setup.dR/2;
Setup.RNorm = Setup.RMids/Setup.RotorRadius;
Setup.PsiVec = (0:Setup.PsiStep:359)';                            % azimuthal discretization
Setup.npsi = length(Setup.PsiVec);
Setup.nrad = length(Setup.RMids);

Setup.Chord = repmat(interp1(Setup.REdges,Setup.ChordDist',Setup.RMids,'linear'),[Setup.npsi,1,Setup.nRotor]);

dA = 0.5*(Setup.ChordDist(1:end-1) + Setup.ChordDist(2:end)).*Setup.dR;

Setup.PitchDeg = repmat(interp1(Setup.REdges,Setup.PitchDistDeg',Setup.RMids,'linear'),[Setup.npsi,1,Setup.nRotor]);
Setup.Pitch = Setup.PitchDeg*pi/180;

Setup.dA = repmat(dA,[Setup.npsi,1,Setup.nRotor]);

Setup.PSI = repmat(Setup.PsiVec,[1,Setup.nrad,Setup.nRotor]);
Setup.R = repmat(Setup.RMids,[Setup.npsi,1,Setup.nRotor]);

Setup.dXBlades_RA = Setup.R.*cosd(Setup.PSI);
Setup.dYBlades_RA = Setup.R.*sind(Setup.PSI);
Setup.dZBlades_RA = 0*Setup.R;

Setup.XBlades_RA = Setup.dXBlades_RA + reshape(Setup.RotorLoc(1,:),[1,1,Setup.nRotor]);
Setup.YBlades_RA = Setup.dYBlades_RA + reshape(Setup.RotorLoc(2,:),[1,1,Setup.nRotor]);
Setup.ZBlades_RA = Setup.dZBlades_RA + + reshape(Setup.RotorLoc(3,:),[1,1,Setup.nRotor]);

Setup.Airfoil = PropDef.Airfoil;

Setup.ThrustScaler = 1;
Setup.TorqueScaler = 1;
end








