function Vehicle = EnergyZoneCalculator(Vehicle)

EZ = Vehicle.Propulsion.EnergyZones;

for i = 1:height(EZ)

    % capture the component geometry
    Comp = Vehicle.Geom.(EZ.Parent{i});

    % what is the component type
    CompType = Comp.Type;

    % limits of energy zone
    Limits = EZ.Limits(i,:);

    % if it is a fuselage type component
    if strcmpi(CompType,'fuselage')
        % cumulative volume at the two limits
        CumuVolumeQuery = interp1(Comp.Stn.FS, Comp.Stn.sdV, Limits', 'linear');
        % geometric volume
        GeomVol = abs(diff(CumuVolumeQuery));
        % mid-volume
        MidVolume = mean(CumuVolumeQuery);
        % query parameters at mid volume
        CentroidQuery = interp1(Comp.Stn.sdV, [Comp.Stn.Xc, Comp.Stn.Yc, Comp.Stn.Zc],MidVolume);
    end

    % if it is a lifting surface type component
    if strcmpi(CompType,'liftingsurface')
        % cumulative volume at the two limits
        CumuVolumeQuery = interp1(Comp.Stn.eta, Comp.Stn.sdV, Limits', 'linear');
        % geometric volume
        GeomVol = abs(diff(CumuVolumeQuery));
        % mid-volume
        MidVolume = mean(CumuVolumeQuery);
        % query parameters at mid volume
        CentroidQuery = interp1(Comp.Stn.sdV, [Comp.Stn.xcg, Comp.Stn.ycg, Comp.Stn.zcg],MidVolume);
    end

    EZ.Centroid(i,:) = CentroidQuery;
    EZ.GeomVol(i) = GeomVol;
    EZ.EffVol(i) = GeomVol * EZ.Efficiency(i);

end

EZ.EffVol_L = EZ.EffVol * 1000;
EZ.EffVol_USG = EZ.EffVol_L / 3.78541;
EZ.EMass_kg = EZ.EffVol.*Vehicle.SOTA.EnergyDensity_kgm3;

Vehicle.Propulsion.EnergyZones = EZ;

end