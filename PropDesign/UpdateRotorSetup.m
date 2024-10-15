function [Setup] = UpdateRotorSetup(Setup)

Setup.DiskArea = pi*Setup.RotorRadius^2;    % m2, rotor disk area


Setup.r = Setup.r_R * Setup.RotorRadius;
Setup.ChordDist = Setup.c_R * Setup.RotorRadius;

% DISCRETIZATION SETTINGS
Setup.REdges = zeros(1,Setup.nrad+1);
Setup.REdges = linspace(Setup.r(1),Setup.r(end),Setup.nrad + 1);

Setup.ChordDistInterp = interp1(Setup.r,Setup.ChordDist,Setup.REdges,'linear');
Setup.PitchDistDegInterp = interp1(Setup.r, Setup.PitchDistDeg, Setup.REdges,'linear');

Setup.dR = diff(Setup.REdges);
Setup.RMids = Setup.REdges(1:end-1) + Setup.dR/2;
Setup.RNorm = Setup.RMids/Setup.RotorRadius;
Setup.PsiVec = (0:Setup.PsiStep:359)';                            % azimuthal discretization
Setup.npsi = length(Setup.PsiVec);
Setup.nrad = length(Setup.RMids);

Setup.Chord = repmat(interp1(Setup.REdges,Setup.ChordDistInterp',Setup.RMids,'linear'),[Setup.npsi,1,Setup.nRotor]);

Setup.PitchDeg = repmat(interp1(Setup.REdges,Setup.PitchDistDegInterp',Setup.RMids,'linear'),[Setup.npsi,1,Setup.nRotor]);
Setup.Pitch = Setup.PitchDeg*pi/180;

Setup.PSI = repmat(Setup.PsiVec,[1,Setup.nrad,Setup.nRotor]);
Setup.R = repmat(Setup.RMids,[Setup.npsi,1,Setup.nRotor]);

Setup.dXBlades_RA = Setup.R.*cosd(Setup.PSI);
Setup.dYBlades_RA = Setup.R.*sind(Setup.PSI);
Setup.dZBlades_RA = 0*Setup.R;

Setup.XBlades_RA = Setup.dXBlades_RA + reshape(Setup.RotorLoc(1,:),[1,1,Setup.nRotor]);
Setup.YBlades_RA = Setup.dYBlades_RA + reshape(Setup.RotorLoc(2,:),[1,1,Setup.nRotor]);
Setup.ZBlades_RA = Setup.dZBlades_RA + + reshape(Setup.RotorLoc(3,:),[1,1,Setup.nRotor]);

dA = 0.5*(Setup.ChordDistInterp(1:end-1) + Setup.ChordDistInterp(2:end)).*Setup.dR;
Setup.dA = repmat(dA,[Setup.npsi,1,Setup.nRotor]);

Setup.ThrustScaler = 1;
Setup.TorqueScaler = 1;


end


