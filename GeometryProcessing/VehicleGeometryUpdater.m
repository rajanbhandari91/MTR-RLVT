function Vehicle = VehicleGeometryUpdater(Vehicle)


%% Plotting
global Settings
if Settings.plotFLAG == 1
    figure; %('Position',[250 200 1000 800])
    axis equal
    xlim([-5, 5])
    ylim([-5, 5])
    zlim([-5, 5])
    set ( gca, 'ydir', 'reverse' )
    set ( gca, 'zdir', 'reverse' )
    set ( gca, 'Clipping', 'off')
    hold on
    view([1,1,1])
    h = gca;
    plot3(h.XLim, [0 0], [0 0], '.-r')
    plot3([0, 0], h.YLim, [0 0], '.-r');
    plot3([0, 0], [0 0], h.ZLim, '.-r');
    set(gcf,'Position',[406 120 980 795])
    % xticks([]);
    % yticks([]);
    % zticks([]);
    % plot_darkmode
end


%% Component Settings
% Electrical
Elec_ref_L = 0.21;
Elec_ref_h = 0.8;
% Flight Controls
FC_ref_L = 0.3;
FC_ref_h = 0.75;
% Hydraulics
Hyd_ref_L = 0.55;
Hyd_ref_h = 0;
% Avionics
AV_ref_L = .4976;
AV_ref_h = 0.85;
% Furnishings
Fur_ref_L = 0.3;
Fur_ref_h = 0;
% Power Cabling
PC_ref_L = 0.4;
PC_ref_h = 0.5;

% Wing mounting height on fuselage
WingMountingHeight = 0.95;

% Rotor Arc Clearance
RotorArcClearance = 0.12;
WingPropClearance = 0.1;

% update the number of time the geometry updater has run
Vehicle.Tracker.GeomUpdaterRuns = Vehicle.Tracker.GeomUpdaterRuns + 1;


%% Geometry Re-sizing rules
ResizingCalculations;

Vehicle.Geom.Fuselage = GeomEval_Fuselage1(Vehicle.Geom.Fuselage);
FusNoseTip = [Vehicle.Geom.Fuselage.Stn.Xc(1) Vehicle.Geom.Fuselage.Stn.Yc(1) Vehicle.Geom.Fuselage.Stn.Zc(1)];
% Evaluate the offset between the location of fuselage nosetip in VSP and
% in PEACE
RefOffset = Vehicle.Geom.Fuselage.RefPtLocationTemp - FusNoseTip;

Param = Vehicle.OpenVSP;
if Vehicle.Tracker.GeomUpdaterRuns == 1 && Vehicle.Tracker.MassIter == 1
    FusFields = fieldnames(Param.Fus);
    for i =  1: length(fieldnames(Param.Fus))
        if ~strcmp((Param.Fus.(FusFields{i}).Name) ,'Fuselage')
            Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).RefPtLocation = Vehicle.Geom.(Param.Fus.(FusFields{i}).Name).RefPtLocationTemp - RefOffset;
            Vehicle.Geom.(Param.Fus.(FusFields{i}).Name) = GeomEval_Fuselage1(Vehicle.Geom.(Param.Fus.(FusFields{i}).Name));
        end
    end

    GeomFields = fieldnames(Vehicle.Geom);
    for i =  1: length(GeomFields)
        if strcmp(Vehicle.Geom.(GeomFields{i}).Type, 'liftingsurface')
            Vehicle.Geom.(GeomFields{i}).RefPtLocation = Vehicle.Geom.(GeomFields{i}).RefPtLocationTemp - RefOffset;
            Vehicle.Geom.(GeomFields{i}) = GeomEval_LiftingSurface(Vehicle.Geom.(GeomFields{i}));
        end

        if strcmp(Vehicle.Geom.(GeomFields{i}).Type, 'LandingGear')
            Vehicle.Geom.(GeomFields{i}).RefPtLocation = Vehicle.Geom.(GeomFields{i}).RefPtLocationTemp - RefOffset;
            Vehicle.Geom.(GeomFields{i}) = GeomEval_LandingGear(Vehicle.Geom.(GeomFields{i}));
        end

        if strcmp(Vehicle.Geom.(GeomFields{i}).Type, 'Wheel')
            Vehicle.Geom.(GeomFields{i}).RefPtLocation = Vehicle.Geom.(GeomFields{i}).RefPtLocationTemp - RefOffset;
            Vehicle.Geom.(GeomFields{i}) = GeomEval_LandingGearWheel(Vehicle.Geom.(GeomFields{i}));
        end

        if isnumeric(Vehicle.Geom.(GeomFields{i}).Type)
            if Vehicle.Geom.(GeomFields{i}).Type == 1 || Vehicle.Geom.(GeomFields{i}).Type == 2
                Vehicle.Geom.(GeomFields{i}).RefPtLocation = Vehicle.Geom.(GeomFields{i}).RefPtLocationTemp - RefOffset;
                Vehicle.Geom.(GeomFields{i}) = GeomEval_DuctedFan(Vehicle.Geom.(GeomFields{i}));
            end
        end

    end

    %% Save the locations of wing and tails wrt fuselage, and booms with respect to wing
    wingeta_x = (Vehicle.Geom.Fuselage.Stn.Xc(1) - Vehicle.Geom.RWing.RefPtLocation(1)) / Vehicle.Geom.Fuselage.Length ;
    wingFus = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],Vehicle.Geom.RWing.RefPtLocation(1),'linear');
    wingeta_z = (wingFus(1) + 0.5*wingFus(2) - Vehicle.Geom.RWing.RefPtLocation(3)) / wingFus(2);
    Vehicle.Recalcs.wingeta_x = wingeta_x;
    Vehicle.Recalcs.wingeta_z = wingeta_z;

    y_ibd_eta = abs(Vehicle.Geom.Pylons_1.RefPtLocation(2))/Vehicle.Geom.RWing.Span;
    Vehicle.Recalcs.y_ibd_eta = y_ibd_eta;

    % Re-evaluate wing to get the propetas for slipstream modeling
    EtaOutboard = min(1,abs(Vehicle.Geom.Prop_1.RefPtLocation(2)) / abs(Vehicle.Geom.LWing.Stn.yQC(end)));
    EtaInboard = min(1,abs(Vehicle.Geom.Prop_2.RefPtLocation(2)) / abs(Vehicle.Geom.LWing.Stn.yQC(end)));
    PropEtas =  [EtaOutboard EtaInboard];
    Vehicle.Geom.LWing.PropEtas = PropEtas;
    Vehicle.Geom.LWing.PropDiams = Vehicle.Recalcs.MainPropDiam_m * ones(1,2);
    FusWidth = interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.Width,Vehicle.Recalcs.wingeta_x,'linear');
    Blockedetas = [0 interp1(Vehicle.Geom.RWing.Stn.yQC,Vehicle.Geom.RWing.Stn.eta,0.5*FusWidth,'linear')];
    Vehicle.Geom.LWing.BlockedEtas = Blockedetas;

    [Vehicle.Geom.LWing] = GeomEval_LiftingSurface(Vehicle.Geom.LWing);

    Vehicle.Geom.RWing.PropEtas = Vehicle.Geom.LWing.PropEtas;
    Vehicle.Geom.RWing.PropDiams = Vehicle.Geom.LWing.PropDiams;
    Vehicle.Geom.RWing.BlockedEtas = Blockedetas;
    [Vehicle.Geom.RWing] = GeomEval_LiftingSurface(Vehicle.Geom.RWing);

    % Get the information for horizontal and vertical tail for weight
    % buildup
    Vehicle.Recalcs.HTailMomentArm_m = Vehicle.Geom.LWing.LocMAC(1) - Vehicle.Geom.RHTail.LocMAC(1);
    Vehicle.Recalcs.VTailMomentArm_m = Vehicle.Geom.LWing.LocMAC(1) - Vehicle.Geom.VTail.LocMAC(1);
    Vehicle.Recalcs.VTailArea_Each_m2 = Vehicle.Geom.VTail.PlanformArea;
    Vehicle.Recalcs.VertStabSpan_m = Vehicle.Geom.VTail.Span;

    % Get Landing gear belly ground clearance for weight build up
    NGFus = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],...
        Vehicle.Geom.NoseWheel.RefPtLocation(1),'linear');
    NGFus_z = NGFus(1) + 0.5*NGFus(2);
    Vehicle.Geom.LandingGear_Nose.BellyGroundClearance = Vehicle.Geom.NoseWheel.Stn.zQC(1) - NGFus_z ;

    MGFus = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],...
        Vehicle.Geom.LandingGear_Main_1.RefPtLocation(1),'linear');
    MGFus_z = MGFus(1) + 0.5*MGFus(2);
    Vehicle.Geom.LandingGear_Main_1.BellyGroundClearance = Vehicle.Geom.MainWheel_1.Stn.zQC(1) - MGFus_z ;
    Vehicle.Geom.LandingGear_Main_2.BellyGroundClearance = Vehicle.Geom.LandingGear_Main_1.BellyGroundClearance;

    % Get the eta location of Nacelles mounting
    Vehicle.Recalcs.Nac1Wing = interp1(Vehicle.Geom.Nacelle_1.Stn.Xc,Vehicle.Geom.Nacelle_1.Stn.FS,Vehicle.Geom.LWing.Stn.xQC(end),'linear');
    Vehicle.Recalcs.Rotor1Nac = interp1(Vehicle.Geom.Nacelle_1.Stn.Xc,Vehicle.Geom.Nacelle_1.Stn.FS,Vehicle.Geom.Prop_1.RefPtLocation(1),'linear');
    Vehicle.Recalcs.Rotor2Nac = interp1(Vehicle.Geom.Nacelle_2.Stn.Xc,Vehicle.Geom.Nacelle_2.Stn.FS,Vehicle.Geom.Prop_2.RefPtLocation(1),'linear');
    Vehicle.Recalcs.Nac1LenBase = Vehicle.Geom.Nacelle_1.Length;
    Vehicle.Recalcs.Nac1HtBase = Vehicle.Geom.Nacelle_1.MaxHeight;
    Vehicle.Recalcs.Nac2LenBase = Vehicle.Geom.Nacelle_2.Length;
    Vehicle.Recalcs.Nac2HtBase = Vehicle.Geom.Nacelle_2.MaxHeight;
    Vehicle.Recalcs.BaselineDia = Vehicle.Geom.Prop_1.Diam;
    Vehicle.Recalcs.Nac2Pylonoff = Vehicle.Geom.Pylons_1.Stn.Xc(1) - Vehicle.Geom.Nacelle_2.Stn.Xc(end);

    % eta location of landing gears
    FusNLdg = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],Vehicle.Geom.LandingGear_Nose.RefPtLocation(1),'linear');
    FusMLdg = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height,Vehicle.Geom.Fuselage.Stn.Width],...
        Vehicle.Geom.LandingGear_Main_1.RefPtLocation(1),'linear');
    Vehicle.Recalcs.NoseLDGxeta = interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.LandingGear_Nose.RefPtLocation(1),'linear');
    Vehicle.Recalcs.MainLDGxeta = interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.LandingGear_Main_1.RefPtLocation(1),'linear');
    Vehicle.Recalcs.NoseLDGzeta = (FusNLdg(1) + 0.5*FusNLdg(2) - Vehicle.Geom.LandingGear_Nose.RefPtLocation(3)) / FusNLdg(2);
    Vehicle.Recalcs.MainLDGzeta = (FusMLdg(1) + 0.5*FusMLdg(2) - Vehicle.Geom.LandingGear_Main_1.RefPtLocation(3)) / FusMLdg(2);
    Vehicle.Recalcs.MainLDGyeta = abs(Vehicle.Geom.LandingGear_Main_1.RefPtLocation(2) / (0.5*FusMLdg(3)));
end

if Vehicle.Tracker.GeomUpdaterRuns > 1 || Vehicle.Tracker.MassIter > 1
    % LWing
    FusWing = interp1(Vehicle.Geom.Fuselage.Stn.FS,[Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],Vehicle.Recalcs.wingeta_x,'linear');
    Vehicle.Geom.LWing.RefPtLocation(1) = FusWing(1);
    Vehicle.Geom.LWing.RefPtLocation(3) = FusWing(2) + 0.5*FusWing(3) - Vehicle.Recalcs.wingeta_z*FusWing(3);
    Vehicle.Geom.LWing.PlanformArea = Vehicle.Recalcs.WingArea_Each_m2;
    Vehicle.Geom.LWing.AspectRatio = 0.5*Vehicle.DesignPoint.WingAspectRatio;
    [Vehicle.Geom.LWing] = GeomEval_LiftingSurface(Vehicle.Geom.LWing);

    Vehicle.Geom.RWing.RefPtLocation = Vehicle.Geom.LWing.RefPtLocation;
    Vehicle.Geom.RWing.PlanformArea = Vehicle.Recalcs.WingArea_Each_m2;
    Vehicle.Geom.RWing.AspectRatio = 0.5*Vehicle.DesignPoint.WingAspectRatio;
    [Vehicle.Geom.RWing] = GeomEval_LiftingSurface(Vehicle.Geom.RWing);

    % Evaluate the prop diameter based on the allowable wingspan and the
    % maximum allowable diameter
    y_ibdlim = max(Vehicle.Recalcs.MainPropDiam_m/2,Vehicle.Geom.Fuselage.MaxWidth/2) + 0.1;
    y_obdlim = Vehicle.Recalcs.WingSpan_m / 2;
    y_availablespan = y_obdlim - y_ibdlim;
    MaxPropDiam = y_availablespan / (1.5+0.05);
    PropDiam_m = min([Vehicle.Recalcs.MaxPropDiam_m,MaxPropDiam,Vehicle.Recalcs.MainPropDiam_m]);

    % Map the Nacelle length and width to the prop diameter with the
    % baseline diameter at 3.0488 m
    Nac1Len = Vehicle.Recalcs.Nac1LenBase - ((Vehicle.Recalcs.BaselineDia - PropDiam_m)/Vehicle.Recalcs.BaselineDia) * Vehicle.Recalcs.Nac1LenBase * 0.5;
    Nac1Ht  = Vehicle.Recalcs.Nac1HtBase - ((Vehicle.Recalcs.BaselineDia - PropDiam_m)/Vehicle.Recalcs.BaselineDia) * Vehicle.Recalcs.Nac1HtBase * 0.5;
    Nac2Len = Vehicle.Recalcs.Nac2LenBase - ((Vehicle.Recalcs.BaselineDia - PropDiam_m)/Vehicle.Recalcs.BaselineDia) * Vehicle.Recalcs.Nac2LenBase * 0.5;
    Nac2Ht  = Vehicle.Recalcs.Nac2HtBase - ((Vehicle.Recalcs.BaselineDia - PropDiam_m)/Vehicle.Recalcs.BaselineDia) * Vehicle.Recalcs.Nac2HtBase * 0.5;

    % Evaluate the Pylons
    RotPylOff = 0.6;
    PropWing = interp1(Vehicle.Geom.LWing.Stn.yQC,[Vehicle.Geom.LWing.Stn.xLE,Vehicle.Geom.LWing.Stn.xTE,Vehicle.Geom.LWing.Stn.c],-y_ibdlim-0.5*PropDiam_m,'linear');
    RotorWing = PropWing(2) - 0.05 - 0.5*PropDiam_m;
    Vehicle.Geom.Pylons_1.CrossSections(2).Length = RotPylOff + 2*abs(RotorWing);
    Vehicle.Geom.Pylons_1.MaxHeight = max(Vehicle.Recalcs.BoomHeight,Vehicle.Recalcs.BoomWidth);
    Vehicle.Geom.Pylons_1.MaxWidth = max(Vehicle.Recalcs.BoomHeight,Vehicle.Recalcs.BoomWidth);
    Vehicle.Geom.Pylons_1.RefPtLocation(2) = -y_ibdlim-0.5*PropDiam_m;
    Vehicle.Geom.Pylons_1.RefPtLocation(1) = 0.5*sum(arrayfun(@(x) x.Length, Vehicle.Geom.Pylons_1.CrossSections));
    PylWing = interp1(Vehicle.Geom.LWing.Stn.yQC,[Vehicle.Geom.LWing.Stn.zQC,Vehicle.Geom.LWing.Stn.c],-y_ibdlim - 0.5*PropDiam_m,'linear');
    Vehicle.Geom.Pylons_1.RefPtLocation(3) = PylWing(1) + 0.5*(PylWing(2) * Vehicle.Geom.LWing.Thickness_to_chord);
    [Vehicle.Geom.Pylons_1] = GeomEval_Fuselage1(Vehicle.Geom.Pylons_1);

    Vehicle.Geom.Pylons_2 = Vehicle.Geom.Pylons_1;
    Vehicle.Geom.Pylons_2.RefPtLocation(2) = -Vehicle.Geom.Pylons_1.RefPtLocation(2);
    Vehicle.Geom.Pylons_2.Name = {'Pylons_2'};
    [Vehicle.Geom.Pylons_2] = GeomEval_Fuselage1(Vehicle.Geom.Pylons_2);


    Vehicle.Geom.Nacelle_1.CrossSections(3).Length = Vehicle.Geom.Nacelle_1.CrossSections(3).Length; % - (Vehicle.Recalcs.Nac1LenBase - Nac1Len);
    Vehicle.Geom.Nacelle_1.MaxHeight = Nac1Ht;
    Vehicle.Geom.Nacelle_1.MaxWidth = Nac1Ht;
    Vehicle.Geom.Nacelle_1.RefPtLocation(2) = -y_obdlim;
    Vehicle.Geom.Nacelle_1.RefPtLocation(3) = Vehicle.Geom.LWing.Stn.zQC(end);
    Vehicle.Geom.Nacelle_1.RefPtLocation(1) = Vehicle.Geom.LWing.Stn.xQC(end) + Vehicle.Recalcs.Nac1Wing*Nac1Len;
    [Vehicle.Geom.Nacelle_1] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_1);

    Vehicle.Geom.Nacelle_4 = Vehicle.Geom.Nacelle_1;
    Vehicle.Geom.Nacelle_4.RefPtLocation(2) = -Vehicle.Geom.Nacelle_1.RefPtLocation(2);
    Vehicle.Geom.Nacelle_4.Name = {'Nacelle_4'};
    [Vehicle.Geom.Nacelle_4] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_4);

    Vehicle.Geom.Nacelle_2.CrossSections(2).Length = Vehicle.Geom.Nacelle_2.CrossSections(2).Length; % - (Vehicle.Recalcs.Nac2LenBase - Nac2Len);
    Vehicle.Geom.Nacelle_2.MaxHeight = Vehicle.Geom.Pylons_1.MaxHeight;
    Vehicle.Geom.Nacelle_2.MaxWidth = Vehicle.Geom.Pylons_1.MaxHeight;
    Vehicle.Geom.Nacelle_2.RefPtLocation = Vehicle.Geom.Pylons_1.RefPtLocation;
    Vehicle.Geom.Nacelle_2.RefPtLocation(1) = abs(RotorWing) + 0.5*Vehicle.Geom.Nacelle_2.MaxWidth + sum(arrayfun(@(x) x.Length, Vehicle.Geom.Nacelle_2.CrossSections));
    [Vehicle.Geom.Nacelle_2] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_2);

    Vehicle.Geom.Nacelle_3 = Vehicle.Geom.Nacelle_2;
    Vehicle.Geom.Nacelle_3.RefPtLocation(2) = -Vehicle.Geom.Nacelle_2.RefPtLocation(2);
    Vehicle.Geom.Nacelle_3.Name = {'Nacelle_3'};
    [Vehicle.Geom.Nacelle_3] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_3);

    Vehicle.Geom.Nacelle_5.CrossSections(2).Length = Vehicle.Geom.Nacelle_5.CrossSections(2).Length; % - (Vehicle.Recalcs.Nac2LenBase - Nac2Len);
    Vehicle.Geom.Nacelle_5.MaxHeight = Vehicle.Geom.Nacelle_2.MaxHeight;
    Vehicle.Geom.Nacelle_5.MaxWidth = Vehicle.Geom.Nacelle_2.MaxWidth;
    Vehicle.Geom.Nacelle_5.RefPtLocation(1) = RotorWing;
    Vehicle.Geom.Nacelle_5.RefPtLocation(2) = Vehicle.Geom.Pylons_1.RefPtLocation(2);
    Vehicle.Geom.Nacelle_5.RefPtLocation(3) = Vehicle.Geom.Pylons_1.RefPtLocation(3) + Vehicle.Geom.Pylons_1.MaxHeight/2 - sum(arrayfun(@(x) x.Length, Vehicle.Geom.Nacelle_5.CrossSections));
    [Vehicle.Geom.Nacelle_5] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_5);

    Vehicle.Geom.Nacelle_6 = Vehicle.Geom.Nacelle_5;
    Vehicle.Geom.Nacelle_6.RefPtLocation(2) = -Vehicle.Geom.Nacelle_5.RefPtLocation(2);
    Vehicle.Geom.Nacelle_6.Name = {'Nacelle_6'};
    [Vehicle.Geom.Nacelle_6] = GeomEval_Fuselage1(Vehicle.Geom.Nacelle_6);

    % Landing Gears Re-evaluate
    x_NLDG = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Recalcs.NoseLDGxeta,'linear');
    Vehicle.Geom.LandingGear_Nose.RefPtLocation(1) = x_NLDG;
    FusNLdg = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],x_NLDG,'linear');
    Vehicle.Geom.LandingGear_Nose.RefPtLocation(3) = FusNLdg(1) + 0.5*FusNLdg(2) - Vehicle.Recalcs.NoseLDGzeta*FusNLdg(2)  ;
    [Vehicle.Geom.LandingGear_Nose] = GeomEval_LandingGear(Vehicle.Geom.LandingGear_Nose);

    x_MLDG = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Recalcs.MainLDGxeta,'linear');
    Vehicle.Geom.LandingGear_Main_1.RefPtLocation(1) = x_MLDG;
    FusMLdg = interp1(Vehicle.Geom.Fuselage.Stn.Xc,[Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],x_MLDG,'linear');
    Vehicle.Geom.LandingGear_Main_1.RefPtLocation(3) = FusMLdg(1) + 0.5*FusMLdg(2) - Vehicle.Recalcs.NoseLDGzeta*FusMLdg(2)  ;
    [Vehicle.Geom.LandingGear_Main_1] = GeomEval_LandingGear(Vehicle.Geom.LandingGear_Main_1);

    Vehicle.Geom.LandingGear_Main_2 = Vehicle.Geom.LandingGear_Main_1 ;
    Vehicle.Geom.LandingGear_Main_2.RefPtLocation(2) = -Vehicle.Geom.LandingGear_Main_1.RefPtLocation(2);
    Vehicle.Geom.LandingGear_Main_2.Directionality = -Vehicle.Geom.LandingGear_Main_1.Directionality;
    Vehicle.Geom.LandingGear_Main_2.Name = {'LandingGear_Main_2'};
    [Vehicle.Geom.LandingGear_Main_2] = GeomEval_LandingGear(Vehicle.Geom.LandingGear_Main_2);

    Vehicle.Geom.MainWheel_1.RefPtLocation(1) = Vehicle.Geom.LandingGear_Main_1.Stn.xcg(end);
    Vehicle.Geom.MainWheel_1.RefPtLocation(3) = Vehicle.Geom.LandingGear_Main_1.Stn.zcg(end);
    [Vehicle.Geom.MainWheel_1] = GeomEval_LandingGearWheel(Vehicle.Geom.MainWheel_1);

    Vehicle.Geom.MainWheel_1.RefPtLocation(1) = Vehicle.Geom.LandingGear_Main_2.Stn.xcg(end);
    Vehicle.Geom.MainWheel_1.RefPtLocation(3) = Vehicle.Geom.LandingGear_Main_2.Stn.zcg(end);
    [Vehicle.Geom.MainWheel_2] = GeomEval_LandingGearWheel(Vehicle.Geom.MainWheel_2);

    Vehicle.Geom.NoseWheel.RefPtLocation(1) = Vehicle.Geom.LandingGear_Nose.Stn.xcg(end);
    Vehicle.Geom.NoseWheel.RefPtLocation(2) = 0;
    Vehicle.Geom.NoseWheel.RefPtLocation(3) = Vehicle.Geom.MainWheel_1.RefPtLocation(3);
    [Vehicle.Geom.NoseWheel] = GeomEval_LandingGearWheel(Vehicle.Geom.NoseWheel);

    % Props and Rotors
    Vehicle.Geom.Prop_1.RefPtLocation(1) = interp1(Vehicle.Geom.Nacelle_1.Stn.FS,Vehicle.Geom.Nacelle_1.Stn.Xc,Vehicle.Recalcs.Rotor1Nac,'linear');
    Vehicle.Geom.Prop_1.RefPtLocation(2) = Vehicle.Geom.Nacelle_1.RefPtLocation(2);
    Vehicle.Geom.Prop_1.RefPtLocation(3) = Vehicle.Geom.Nacelle_1.RefPtLocation(3);
    Vehicle.Geom.Prop_1.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(1);
    [Vehicle.Geom.Prop_1] = GeomEval_DuctedFan(Vehicle.Geom.Prop_1);

    Vehicle.Geom.Prop_4 = Vehicle.Geom.Prop_1;
    Vehicle.Geom.Prop_4.RefPtLocation(2) = -Vehicle.Geom.Prop_1.RefPtLocation(2);
    Vehicle.Geom.Prop_1.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(4);
    Vehicle.Geom.Prop_4.Name = {'Prop_4'};
    [Vehicle.Geom.Prop_4] = GeomEval_DuctedFan(Vehicle.Geom.Prop_4);

    Vehicle.Geom.Prop_2.RefPtLocation(1) = interp1(Vehicle.Geom.Nacelle_2.Stn.FS,Vehicle.Geom.Nacelle_2.Stn.Xc,Vehicle.Recalcs.Rotor2Nac,'linear');
    Vehicle.Geom.Prop_2.RefPtLocation(2) = Vehicle.Geom.Nacelle_2.RefPtLocation(2);
    Vehicle.Geom.Prop_2.RefPtLocation(3) = Vehicle.Geom.Nacelle_2.RefPtLocation(3);
    Vehicle.Geom.Prop_2.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(2);
    [Vehicle.Geom.Prop_2] = GeomEval_DuctedFan(Vehicle.Geom.Prop_2);

    Vehicle.Geom.Prop_3 = Vehicle.Geom.Prop_2;
    Vehicle.Geom.Prop_3.RefPtLocation(2) = -Vehicle.Geom.Prop_2.RefPtLocation(2);
    Vehicle.Geom.Prop_3.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(3);
    Vehicle.Geom.Prop_3.Name = {'Prop_3'};
    [Vehicle.Geom.Prop_3] = GeomEval_DuctedFan(Vehicle.Geom.Prop_3);

    Vehicle.Geom.Prop_5.RefPtLocation(3) = interp1(Vehicle.Geom.Nacelle_5.Stn.FS,Vehicle.Geom.Nacelle_5.Stn.Zc,Vehicle.Recalcs.Rotor2Nac,'linear');
    Vehicle.Geom.Prop_5.RefPtLocation(2) = Vehicle.Geom.Nacelle_5.RefPtLocation(2);
    Vehicle.Geom.Prop_5.RefPtLocation(1) = Vehicle.Geom.Nacelle_5.RefPtLocation(1);
    Vehicle.Geom.Prop_5.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(1);
    [Vehicle.Geom.Prop_5] = GeomEval_DuctedFan(Vehicle.Geom.Prop_5);

    Vehicle.Geom.Prop_6 = Vehicle.Geom.Prop_5;
    Vehicle.Geom.Prop_6.RefPtLocation(2) = -Vehicle.Geom.Prop_5.RefPtLocation(2);
    Vehicle.Geom.Prop_6.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(2);
    Vehicle.Geom.Prop_6.Name = {'Prop_6'};
    [Vehicle.Geom.Prop_6] = GeomEval_DuctedFan(Vehicle.Geom.Prop_6);

    % Tail Geometry and Tail Sizing
    Fus_HTail = interp1(Vehicle.Geom.Fuselage.Stn.FS,[Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],1,'linear');
    Fus_VTail = interp1(Vehicle.Geom.Fuselage.Stn.FS,[Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.Zc,Vehicle.Geom.Fuselage.Stn.Height],1,'linear');
    % on odd executions, run stabilizer geometry calculations only
    % if(mod(Vehicle.Tracker.GeomUpdaterRuns,2)==1)
        [Vehicle] = StabilizerGeometryCalculations(Vehicle,Fus_HTail,Fus_VTail);
    % end

    % % on even executions, run the stabilizer sizing calculations
    % if(mod(Vehicle.Tracker.GeomUpdaterRuns,2)==0 && Vehicle.Tracker.GeomUpdaterRuns >= 2)
    %     [Vehicle] = TailSizingCalculations(Vehicle,Fus_HTail,Fus_VTail);
    % end


    if Vehicle.Architecture == 3
        % locate the turboshaft engine
        Fus_x_ts1 = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Propulsion.EnergyZones.Limits(2,1),'linear');
        x_ts1 = Fus_x_ts1;
        y_ts1 = 0;
        z_ts1 =  Vehicle.Propulsion.EnergyZones.Centroid(1,3) - 2*Vehicle.Recalcs.TurboshaftDiam_m;

        Vehicle.Geom.Turboshaft.RefPtLocation = [x_ts1,y_ts1,z_ts1];

        Vehicle.Geom.Turboshaft.MaxHeight = Vehicle.Recalcs.TurboshaftDiam_m;
        Vehicle.Geom.Turboshaft.MaxWidth = Vehicle.Recalcs.TurboshaftDiam_m;

        if Vehicle.Recalcs.TurboshaftLength_m > Vehicle.Geom.Turboshaft.Length
            Vehicle.Geom.Turboshaft.CrossSections(2).Length = Vehicle.Geom.Turboshaft.CrossSections(2).Length + (Vehicle.Recalcs.TurboshaftLength_m - Vehicle.Geom.Turboshaft.Length) ;
        end

        % evaluate the turboshaft component
        [Vehicle.Geom.Turboshaft] = GeomEval_Fuselage1(Vehicle.Geom.Turboshaft);

    end
end

%% Motors
MotorOffsetMP = [0.5*Vehicle.Recalcs.LiftMotorLength_m 0 0];
MotorOffsetLP = [0 0 0.5*Vehicle.Recalcs.LiftMotorLength_m];
Vehicle.Geom.Motors.RefPtLocation = [Vehicle.Geom.Prop_1.RefPtLocation - MotorOffsetMP; Vehicle.Geom.Prop_2.RefPtLocation - MotorOffsetMP;...
    Vehicle.Geom.Prop_3.RefPtLocation - MotorOffsetMP; Vehicle.Geom.Prop_4.RefPtLocation - MotorOffsetMP;...
    Vehicle.Geom.Prop_5.RefPtLocation + MotorOffsetLP; Vehicle.Geom.Prop_6.RefPtLocation + MotorOffsetLP];
Vehicle.Geom.Motors = GeomEval_GenericComponent(Vehicle.Geom.Motors);

%% Energy Zone Calculations
% zone 1
z1 = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS == 1,:);
z2 = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS == 2,:);
Vehicle.Propulsion.EnergyZones.Limits(1,:) = [z1.FS(1),z2.FS(end)];

z3 = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS == 4,:);
z4 = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS == 5,:);
Vehicle.Propulsion.EnergyZones.Limits(2,:) = [z3.FS(1) + 0.45*(z3.FS(end) - z3.FS(1)), z3.FS(end)];
Vehicle.Propulsion.EnergyZones.Limits(3,:) = [z4.FS(1) ,z4.FS(1) + 0.3*(z4.FS(end) - z4.FS(1))];

Vehicle = EnergyZoneCalculator(Vehicle);

%% Architecture Specific Calculations
if Vehicle.Architecture == 1
    Vehicle.Geom.Batteries.RefPtLocation = Vehicle.Propulsion.EnergyZones.Centroid([1,2],:);
    Vehicle.Geom.Batteries = GeomEval_GenericComponent(Vehicle.Geom.Batteries);
end

% for HE architecture and TE architecture
if(Vehicle.Architecture == 3)
    % locate gearboxes
    Vehicle.Geom.Gearbox.RefPtLocation = Vehicle.Geom.Turboshaft.RefPtLocation  - [Vehicle.Recalcs.TurboshaftLength_m/2,0,0];

    % locate generators
    Vehicle.Geom.Generator.RefPtLocation = Vehicle.Geom.Gearbox.RefPtLocation  + [0,0,0.5*Vehicle.Recalcs.GeneratorDiam_m];

    % evaluate the gearbox component
    [Vehicle.Geom.Gearbox] = GeomEval_GenericComponent(Vehicle.Geom.Gearbox);

    % evaluate the generator component
    [Vehicle.Geom.Generator] = GeomEval_GenericComponent(Vehicle.Geom.Generator);

    % locate emergency battery
    Elec_x = Vehicle.Propulsion.EnergyZones.Centroid(3,1);
    Elec_y = 0;
    Elec_z = Vehicle.Propulsion.EnergyZones.Centroid(3,3);

    Vehicle.Geom.Batteries.RefPtLocation(1,1:3) = [Elec_x Elec_y Elec_z];
    %
    %Evaluate the emergency battery
    Vehicle.Geom.Batteries = GeomEval_GenericComponent(Vehicle.Geom.Batteries);

    % Fuel Tank
    Vehicle.Geom.FuelTank_1.RefPtLocation(1) = Vehicle.Propulsion.EnergyZones.Centroid(2,1) + 0.5*Vehicle.Geom.FuelTank_1.Length;
    Vehicle.Geom.FuelTank_1.RefPtLocation(3) = Vehicle.Propulsion.EnergyZones.Centroid(2,3) + 0.5*Vehicle.Geom.FuelTank_1.MaxHeight;
    [Vehicle.Geom.FuelTank_1] = GeomEval_Fuselage1(Vehicle.Geom.FuelTank_1);

    Vehicle.Geom.FuelTank_2.RefPtLocation(1) = Vehicle.Propulsion.EnergyZones.Centroid(1,1) + Vehicle.Geom.FuelTank_2.Length;
    Vehicle.Geom.FuelTank_2.RefPtLocation(3) = Vehicle.Propulsion.EnergyZones.Centroid(1,3);
    [Vehicle.Geom.FuelTank_2] = GeomEval_Fuselage1(Vehicle.Geom.FuelTank_2);

    % set fuel energy zones
    Vehicle.Geom.Fuel.RefPtLocation = [Vehicle.Geom.FuelTank_1.RefPtLocation;Vehicle.Geom.FuelTank_2.RefPtLocation];

    % evaluate fuel
    [Vehicle.Geom.Fuel] = GeomEval_GenericComponent(Vehicle.Geom.Fuel);

    % adjust fuselage crosssection_2 length
    if(mod(Vehicle.Tracker.GeomUpdaterRuns,2)==0)
        Cross2length = Vehicle.Geom.Fuselage.CrossSections(2).Length;

        if Vehicle.Architecture == 3
            Cross2length = max(0.75 * Vehicle.Recalcs.TurboshaftLength_m,Cross2length);
        end
        % reference point is defined as center of Section 3
        LNoseToSec3MidPoint = Vehicle.Geom.Fuselage.CrossSections(1).Length + Vehicle.Geom.Fuselage.CrossSections(2).Length + Vehicle.Geom.Fuselage.CrossSections(3).Length*0.5;
        LTotal = sum(arrayfun(@(x) x.Length, Vehicle.Geom.Fuselage.CrossSections)) + Vehicle.Geom.Fuselage.CrossSections(2).Length - Cross2length;
        Vehicle.Geom.Fuselage.RefPt_FS = LNoseToSec3MidPoint/LTotal;                                                       % FS of fuselage coinciding with ref pt
        Vehicle.Geom.Fuselage.CrossSections(2).Length = Cross2length;

    end

end

%% Electrical
Elec_x = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Elec_ref_L)+0.2;
Elec_y = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Yc,Elec_ref_L);
Elec_z = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Zc,Elec_ref_L)...
    + Elec_ref_h * interp1(Vehicle.Geom.Fuselage.Stn.FS,...
    Vehicle.Geom.Fuselage.Stn.Height,Elec_ref_L)/2;

Vehicle.Geom.Electrical.RefPtLocation(1,1:3) = [Elec_x Elec_y Elec_z];

[Vehicle.Geom.Electrical] = GeomEval_GenericComponent(Vehicle.Geom.Electrical);

%% Flight Controls
FC_ref_L = mean(Vehicle.Propulsion.EnergyZones.Limits(1,:));
FC_x = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,FC_ref_L);
FC_y = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Yc,FC_ref_L);
FC_z = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Zc,FC_ref_L)...
    - FC_ref_h * interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Height,FC_ref_L)/2;

Vehicle.Geom.Flight_Controls.RefPtLocation(1,1:3) = [FC_x FC_y FC_z];

[Vehicle.Geom.Flight_Controls] = GeomEval_GenericComponent(Vehicle.Geom.Flight_Controls);

%% Hydraulics
Hyd_x = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Hyd_ref_L);
Hyd_y = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Yc,Hyd_ref_L);
Hyd_z = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Zc,Hyd_ref_L)...
    - Hyd_ref_h * interp1(Vehicle.Geom.Fuselage.Stn.FS,...
    Vehicle.Geom.Fuselage.Stn.Height,Hyd_ref_L)/2;

Vehicle.Geom.Hydraulics.RefPtLocation(1,1:3) = [Hyd_x Hyd_y Hyd_z];

[Vehicle.Geom.Hydraulics] = GeomEval_GenericComponent(Vehicle.Geom.Hydraulics);

%% Avionics
AV_x = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,AV_ref_L);
AV_y = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Yc,AV_ref_L);
AV_z = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Zc,AV_ref_L)...
    - AV_ref_h * interp1(Vehicle.Geom.Fuselage.Stn.FS,...
    Vehicle.Geom.Fuselage.Stn.Height,AV_ref_L)/2;

Vehicle.Geom.Avionics.RefPtLocation(1,1:3) = [AV_x AV_y AV_z];

[Vehicle.Geom.Avionics] = GeomEval_GenericComponent(Vehicle.Geom.Avionics);

%% Furnishings
Fur_x = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Xc,Fur_ref_L);
Fur_y = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Yc,Fur_ref_L);
Fur_z = interp1(Vehicle.Geom.Fuselage.Stn.FS,Vehicle.Geom.Fuselage.Stn.Zc,Fur_ref_L)...
    - Fur_ref_h * interp1(Vehicle.Geom.Fuselage.Stn.FS,...
    Vehicle.Geom.Fuselage.Stn.Height,Fur_ref_L)/2;

Vehicle.Geom.Furnishings.RefPtLocation(1,1:3) = [Fur_x Fur_y Fur_z];

[Vehicle.Geom.Furnishings] = GeomEval_GenericComponent(Vehicle.Geom.Furnishings);

%% Passengers

FrontPAXSeatPivot_search = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS==3,:);
FrontPAXSeatPivot = [FrontPAXSeatPivot_search.Xc(1),0,FrontPAXSeatPivot_search.Zc(1)];

% AftPAXSeatPivot_search = Vehicle.Geom.Fuselage.Stn(Vehicle.Geom.Fuselage.Stn.CS==4,:);
% AftPAXSeatPivot = [AftPAXSeatPivot_search.Xc(end), 0,  + AftPAXSeatPivot_search.Height(end)/2];

PAX_y = 0.3;

% Standard CG from origin [0.228 0.0059 0.248]
CG_std = [0.228, 0.000, -0.248];

Vehicle.Geom.Passengers.RefPtLocation(1,1:3) = [FrontPAXSeatPivot(1) + CG_std(1), -PAX_y, FrontPAXSeatPivot(3) + CG_std(3)];
Vehicle.Geom.Passengers.RefPtLocation(2,1:3) = Vehicle.Geom.Passengers.RefPtLocation(1,1:3) - [0.76 0 0];
Vehicle.Geom.Passengers.RefPtLocation(3,1:3) = Vehicle.Geom.Passengers.RefPtLocation(2,1:3) - [0.76 0 0];
Vehicle.Geom.Passengers.RefPtLocation(4,1:3) = Vehicle.Geom.Passengers.RefPtLocation(1,1:3) .* [1 -1 1];
Vehicle.Geom.Passengers.RefPtLocation(5,1:3) = Vehicle.Geom.Passengers.RefPtLocation(2,1:3) .* [1 -1 1];
Vehicle.Geom.Passengers.RefPtLocation(6,1:3) = Vehicle.Geom.Passengers.RefPtLocation(3,1:3) .* [1 -1 1];

[Vehicle.Geom.Passengers] = GeomEval_GenericComponent(Vehicle.Geom.Passengers);

%% AERO-PROPULSIVE MODEL UPDATES
% #########################################################UpdateAeroModel is commented########################################
% % aerodynamic model
Vehicle.Propulsion.RunQMILQPROP = 1;
[Vehicle] = UpdateAeroModel(Vehicle);

%% Propeller Setup and Design
% LPSetup
Vehicle.Propulsion.LPSetup.RotorLoc = [Vehicle.Geom.Prop_5.RefPtLocation; Vehicle.Geom.Prop_6.RefPtLocation]';
Vehicle.Propulsion.CPSetup.RotorLoc = [Vehicle.Geom.Prop_1.RefPtLocation;Vehicle.Geom.Prop_2.RefPtLocation;...
                                        Vehicle.Geom.Prop_3.RefPtLocation;Vehicle.Geom.Prop_4.RefPtLocation]';
if Vehicle.Propulsion.RunQMILQPROP == 1
    if Vehicle.Tracker.MassIter == 1 && Vehicle.Tracker.GeomUpdaterRuns == 1
        DesignMTOM_kg = Vehicle.MassProp.MTOM_kg;
        % Update Prop requirements after resizing
        nCase = height(Vehicle.Propulsion.LPCases);
        Vehicle.Propulsion.LPDef.r_t = Vehicle.Propulsion.LiftPropDiam_m/2;
        Vehicle.Propulsion.LPCases.Mass_kg = DesignMTOM_kg*ones(nCase,1);

        % Hover Cases (TWR = 1 for hover calculations)
        HovId = 1:nCase;
        Vehicle.Propulsion.LPCases.Treq_N(HovId) = 1/3*Vehicle.Propulsion.LPCases.TWR(HovId) .* DesignMTOM_kg ...
            * 9.81./Vehicle.Propulsion.LPCases.nProps(HovId);

        Settings.FileName = [];                         % Don't read the excelsheet

        % Input to CompleteRotorSetup
        Vehicle.Propulsion.LPSetup.nRotor = 2;
        Vehicle.Propulsion.LPSetup.SpinDir = [-1, 1];
        Vehicle.Propulsion.LPSetup.HealthStatus = [1 1];
        Vehicle.Propulsion.LPSetup.RotorAxisTheta = Vehicle.Geom.Prop_5.Theta .*ones(1,2);
        Vehicle.Propulsion.LPSetup.RotorAxisPhi = [0 0];


        % Prop Sizing where propeller are designed using QMIL and Setup file is
        % created using CompleteRotorSetup
        [DesignCases, Setup] = PropDesignSizing(Settings, Vehicle.Propulsion.LPDef, Vehicle.Propulsion.LPCases,Vehicle.Propulsion.LPSetup);
        Vehicle.Propulsion.LPSetup = Setup;
        Vehicle.Propulsion.LPSetup.DesignCases = DesignCases;

        % Check if any failed cases, if yes, run QMIL/QPROP again by reducing the
        % design CL

        if any(isnan(DesignCases.Tact_N))

            maxIter = 2;
            del_CL = 0.05;

            for i = 1:maxIter

                fprintf('\n\n...Propeller failed at off-design case...\n\n')
                disp(DesignCases)
                fprintf('\n\n...Design CL reduced by %.2f...\n\n',del_CL*i)


                Vehicle.Propulsion.LPDef.CLdes = Vehicle.Propulsion.LPDef.CLdes - del_CL;

                [DesignCases, Setup] = PropDesignSizing(Settings, Vehicle.Propulsion.LPDef, Vehicle.Propulsion.Cases,Vehicle.Propulsion.LPSetup);
                Vehicle.Propulsion.LPSetup = Setup;
                Vehicle.Propulsion.LPSetup.DesignCases = DesignCases;
                if ~any(isnan(DesignCases.Tact_N(2:end)))

                    fprintf('\n\n PROPELLER DESIGN CONVERGED...!\n\n')
                    disp(DesignCases)

                    break
                end

            end
        end
    else
        Vehicle.Propulsion.LPSetup.RotorRadius = 0.5*Vehicle.Recalcs.MainPropDiam_m;
        Vehicle.Propulsion.LPSetup.RotorDiam_m = Vehicle.Recalcs.MainPropDiam_m;
        Vehicle.Propulsion.LPSetup = UpdateRotorSetup(Vehicle.Propulsion.LPSetup);
    end
    Vehicle.Geom.Prop_5.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(1);
    Vehicle.Geom.Prop_6.SpinDir = Vehicle.Propulsion.LPSetup.SpinDir(2);

    Vehicle.Geom.Prop_5.Diam = Vehicle.Propulsion.LPSetup.RotorDiam_m;
    Vehicle.Geom.Prop_6.Diam = Vehicle.Propulsion.LPSetup.RotorDiam_m;
    clear Setup


    % run the update script to commit these changes to the rotor BEMT model
    [Vehicle.Propulsion.LPSetup] = UpdateRotorSetup(Vehicle.Propulsion.LPSetup);
    Vehicle.Propulsion.LPSetup = TestProp(Vehicle,Vehicle.Propulsion.LPSetup);
    

    %% CPSetup
    if Vehicle.Tracker.MassIter == 1  && Vehicle.Tracker.GeomUpdaterRuns == 1
        nCase = height(Vehicle.Propulsion.CPCases);
        Vehicle.Propulsion.CPDef.r_t = Vehicle.Propulsion.MainPropDiam_m/2;
        Vehicle.Propulsion.CPCases.Mass_kg = DesignMTOM_kg*ones(nCase,1);

        L_D = 10;  % fixed based on baseline aerodynamics

        % FWD flight, compute thrust required based on updated Mass
        % if isfield(Vehicle,'PointPerf.Results')
        %     LDMission1 = Vehicle.PointPerf.Results.LD(1);
        %     Vehicle.Propulsion.CPCases.Treq_N(1) =  DesignMTOM_kg * 9.81 * (1/LDMission1) / Vehicle.Propulsion.CPCases.nProps ;
        % else
            Vehicle.Propulsion.CPCases.Treq_N(1) = (DesignMTOM_kg * 9.81 * (1/L_D) / Vehicle.Propulsion.CPCases.nProps(1)) ;
            Vehicle.Propulsion.CPCases.Treq_N(2) = 2/3*DesignMTOM_kg * 9.81 ./ Vehicle.Propulsion.CPCases.nProps(2);
            Vehicle.Propulsion.CPCases.Treq_N(3) = 2/3*Vehicle.Propulsion.CPCases.TWR(3) .* DesignMTOM_kg ...
                                                    * 9.81./Vehicle.Propulsion.CPCases.nProps(3);
        % end

        Vehicle.Propulsion.CPCases.TWR = Vehicle.Propulsion.CPCases.Treq_N.*Vehicle.Propulsion.CPCases.nProps...
            /(DesignMTOM_kg*9.81);

        Settings.FileName = [];                         % Don't read the excelsheet

        % Input to CompleteRotorSetup
        Vehicle.Propulsion.CPSetup.nRotor = 4;
        Vehicle.Propulsion.CPSetup.SpinDir = [1 -1 1 -1];
        Vehicle.Propulsion.CPSetup.HealthStatus = [1 1 1 1];
        Vehicle.Propulsion.CPSetup.RotorAxisTheta = [-90 -90 -90 -90];
        Vehicle.Propulsion.CPSetup.RotorAxisPhi = [0 0 0 0] ;

        % Prop Sizing where propeller are designed using QMIL and Setup file is
        % created using CompleteRotorSetup
        [DesignCases, Setup] = PropDesignSizing(Settings, Vehicle.Propulsion.CPDef, Vehicle.Propulsion.CPCases,Vehicle.Propulsion.CPSetup);
        Vehicle.Propulsion.CPSetup = Setup;
        Vehicle.Propulsion.CPSetup.DesignCases = DesignCases;

        % Check if any failed cases, if yes, run QMIL/QPROP again by reducing the
        % design CL

        if any(isnan(DesignCases.Tact_N))

            maxIter = 5;
            del_CL = 0.05;

            for i = 1:maxIter

                fprintf('\n\n...Propeller failed at off-design case...\n\n')
                disp(DesignCases)
                fprintf('\n\n...Design CL reduced by %.2f...\n\n',del_CL*i)


                Vehicle.Propulsion.PropDef.CLdes = Vehicle.Propulsion.CPDef.CLdes - del_CL;

                [DesignCases, Setup] = PropDesignSizing(Settings, Vehicle.Propulsion.CPDef, Vehicle.Propulsion.CPCases,Vehicle.Propulsion.CPSetup);
                Vehicle.Propulsion.CPSetup = Setup;
                Vehicle.Propulsion.CPSetup.DesignCases = DesignCases;
                if ~any(isnan(DesignCases.Tact_N(2:end)))

                    fprintf('\n\n PROPELLER DESIGN CONVERGED...!\n\n')
                    disp(DesignCases)

                    break
                end

            end
        end


    else
        Vehicle.Propulsion.CPSetup.RotorRadius = 0.5*Vehicle.Recalcs.MainPropDiam_m;
        Vehicle.Propulsion.CPSetup.RotorDiam_m = Vehicle.Recalcs.MainPropDiam_m;
        Vehicle.Propulsion.CPSetup = UpdateRotorSetup(Vehicle.Propulsion.CPSetup);
    end
    Vehicle.Geom.Prop_1.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(1);
    Vehicle.Geom.Prop_2.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(2);
    Vehicle.Geom.Prop_3.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(3);
    Vehicle.Geom.Prop_4.SpinDir = Vehicle.Propulsion.CPSetup.SpinDir(4);

    Vehicle.Geom.Prop_1.Diam = Vehicle.Propulsion.CPSetup.RotorDiam_m;
    Vehicle.Geom.Prop_2.Diam = Vehicle.Propulsion.CPSetup.RotorDiam_m;
    Vehicle.Geom.Prop_3.Diam = Vehicle.Propulsion.CPSetup.RotorDiam_m;
    Vehicle.Geom.Prop_4.Diam = Vehicle.Propulsion.CPSetup.RotorDiam_m;

    clear Setup

    Vehicle.Propulsion.CPSetup = UpdateRotorSetup(Vehicle.Propulsion.CPSetup);

    Vehicle.Propulsion.CPSetup = TestProp(Vehicle,Vehicle.Propulsion.CPSetup);
end
% % Wing mounted (Prop 1 and 4)
% Hinge_14 = [Vehicle.Geom.NAC_1.RefPtLocation' Vehicle.Geom.NAC_2.RefPtLocation'  ];
Hinge1 = [0 Vehicle.Geom.Nacelle_1.RefPtLocation(2) Vehicle.Geom.Nacelle_1.RefPtLocation(3)]';
Hinge4 = [0 Vehicle.Geom.Nacelle_4.RefPtLocation(2) Vehicle.Geom.Nacelle_4.RefPtLocation(3)]';
Hinge_14 = [Hinge1 Hinge4];

% % Boom mounted (Prop 2 and 3)

x2 =  Vehicle.Geom.Nacelle_2.Stn.Xc(end);
y2 =  Vehicle.Geom.Nacelle_2.Stn.Yc(end);
z2 =  Vehicle.Geom.Nacelle_2.Stn.Zc(end);

x3 =  Vehicle.Geom.Nacelle_3.Stn.Xc(end);
y3 =  Vehicle.Geom.Nacelle_3.Stn.Yc(end);
z3 =  Vehicle.Geom.Nacelle_3.Stn.Zc(end);

Hinge_23 = [x2 x3
    y2 y3
    z2 z3];

Vehicle.Propulsion.CPSetup.HingeLoc = [Hinge_14(:,1) Hinge_23(:,1) Hinge_23(:,2)  Hinge_14(:,2)];


% update rotor locations (3 x 6 matrix)
Vehicle.Propulsion.CPSetup.RotorLoc = [Vehicle.Geom.Prop_1.RefPtLocation; Vehicle.Geom.Prop_2.RefPtLocation;...
    Vehicle.Geom.Prop_3.RefPtLocation; Vehicle.Geom.Prop_4.RefPtLocation]';

Vehicle.Propulsion.CPSetup.HingeToHub = Vehicle.Propulsion.CPSetup.RotorLoc - Vehicle.Propulsion.CPSetup.HingeLoc;

%% Attach stripdef to the prop definitions from the assigned strip indices

LWingStripDef = Vehicle.Aero.StripDef(strcmp(Vehicle.Aero.StripDef.Name,"Left Wing"),:);
RWingStripDef = Vehicle.Aero.StripDef(strcmp(Vehicle.Aero.StripDef.Name,"Right Wing"),:);

% Prop 1 
Vehicle.Geom.LWing.Prop1.StripDef = LWingStripDef(Vehicle.Geom.LWing.Prop1.StripIndices,:);

% Prop 2
Vehicle.Geom.LWing.Prop2.StripDef = LWingStripDef(Vehicle.Geom.LWing.Prop2.StripIndices,:);

% Prop 3
Vehicle.Geom.RWing.Prop2.StripDef =  RWingStripDef(Vehicle.Geom.RWing.Prop2.StripIndices,:);

% Prop 4
Vehicle.Geom.RWing.Prop1.StripDef =  RWingStripDef(Vehicle.Geom.RWing.Prop1.StripIndices,:);

%% AC3D Geometry
if Settings.ac3dFLAG == 1 || Settings.ac3dFLAG == 2
    for i = 1:Vehicle.Propulsion.LPSetup.nRotor

        SpecSetup = Vehicle.Propulsion.LPSetup;
        SpecSetup.Chord = Vehicle.Propulsion.LPSetup.ChordDist;
        SpecSetup.Pitch = Vehicle.Propulsion.LPSetup.PitchDistDeg;
        MCSFlag = 0;

        [TempRotor] = GenerateRotors(Vehicle.Propulsion.LPSetup.RotorLoc(:,i)',Vehicle.Propulsion.LPSetup.RotorAxisPhi(i),Vehicle.Propulsion.LPSetup.RotorAxisTheta(i),Vehicle.Propulsion.LPSetup.SpinDir(i),SpecSetup,SpecSetup.AirfoilDef.Airfoil,1,[1,0,0],'blank',MCSFlag);
        PropName = sprintf('Prop_%0.0f',i+4);

        Vehicle.Geom.(PropName).Vertices = TempRotor.Vertices;
        Vehicle.Geom.(PropName).Surfaces = TempRotor.Surfaces;

    end

    for i = 1:Vehicle.Propulsion.CPSetup.nRotor

        SpecSetup = Vehicle.Propulsion.CPSetup;
        SpecSetup.Chord = Vehicle.Propulsion.CPSetup.ChordDist;
        SpecSetup.Pitch = Vehicle.Propulsion.CPSetup.PitchDistDeg;
        MCSFlag = 0;

        [TempCP] = GenerateRotors(Vehicle.Propulsion.CPSetup.RotorLoc(:,i)',Vehicle.Propulsion.CPSetup.RotorAxisPhi(i),Vehicle.Propulsion.CPSetup.RotorAxisTheta(i),Vehicle.Propulsion.CPSetup.SpinDir(i),SpecSetup,SpecSetup.AirfoilDef.Airfoil,1,[1,0,0],'blank',MCSFlag);

        PropName = sprintf('Prop_%0.0f',i);

        Vehicle.Geom.(PropName).Vertices = TempCP.Vertices;
        Vehicle.Geom.(PropName).Surfaces = TempCP.Surfaces;

    end
end


%% Set up lift propulsor control distribution matrix
x_cg = Vehicle.MassProp.TargetCG(1); y_cg = Vehicle.MassProp.TargetCG(2); z_cg = Vehicle.MassProp.TargetCG(3);

dth = [90 90 90 90];

cosd_dth = cosd(dth); sind_dth = sind(dth);
% propulsor hub locations during FFM
xloc = Vehicle.Propulsion.CPSetup.HingeLoc(1,:) + ...
    Vehicle.Propulsion.CPSetup.HingeToHub(1,:).*cosd_dth + ...
    Vehicle.Propulsion.CPSetup.HingeToHub(3,:).*sind_dth;

yloc = Vehicle.Propulsion.CPSetup.HingeLoc(2,:);

zloc = Vehicle.Propulsion.CPSetup.HingeLoc(3,:) + ...
    - Vehicle.Propulsion.CPSetup.HingeToHub(1,:).*sind_dth + ...
    Vehicle.Propulsion.CPSetup.HingeToHub(3,:).*cosd_dth;

% generate the lift rotor control distribution matrix dBmat
Lx =  ([xloc Vehicle.Propulsion.LPSetup.RotorLoc(1,:)] - x_cg);
Lx = Lx/max(abs(Lx));

Ly =  -([yloc Vehicle.Propulsion.LPSetup.RotorLoc(2,:)] - y_cg);
Ly = Ly/max(abs(Ly));

Lx_Ly = abs(Lx)./abs(Ly);

Lx = Lx .* Lx_Ly;
Lx = Lx./max(abs(Lx));

dBmat(:,1) = Ly';
dBmat(:,2) = Lx';

dBmat(:,3) = zeros(6,1);

Vehicle.Controls.dBmat = dBmat;



% slipstream modeling parameters
% Compute distance from propellers to midchord of wings they are mounted on
% This is the hub to hinge distance in the current model

% Prop 1 and 4
RefPtProp_1 =  Vehicle.Geom.Prop_1.RefPtLocation;
WingProp_1 = [Vehicle.Geom.LWing.Stn.xQC(end) Vehicle.Geom.LWing.Stn.yQC(end) Vehicle.Geom.LWing.Stn.zQC(end)];

RefPtProp_4 =  Vehicle.Geom.Prop_4.RefPtLocation;
WingProp_4 = [WingProp_1(1) -WingProp_1(2) WingProp_1(3)];

% Prop 2 and 3
RefPtProp_2 = Vehicle.Geom.Prop_2.RefPtLocation;
xQCyQCzQCProp_2 = interp1(Vehicle.Geom.LWing.Stn.yQC, ...
    [Vehicle.Geom.LWing.Stn.xQC, Vehicle.Geom.LWing.Stn.yQC, Vehicle.Geom.LWing.Stn.zQC], RefPtProp_2(2));
WingProp_2 = [xQCyQCzQCProp_2(1), xQCyQCzQCProp_2(2) xQCyQCzQCProp_2(3)];

RefPtProp_3 = Vehicle.Geom.Prop_3.RefPtLocation;
WingProp_3 = [WingProp_2(1) -WingProp_2(2) WingProp_2(3)];


s = abs([RefPtProp_1(1) - WingProp_1(1), RefPtProp_2(1) - WingProp_2(1), RefPtProp_3(1) - WingProp_3(1), RefPtProp_4(1) - WingProp_4(1)]);

% calculate kd parameters: kd = 1 + s/sqrt(s^2+R^2)
kd = 1 + s./sqrt(s.^2 + Vehicle.Propulsion.CPSetup.RotorRadius^2);

% calculate r/c (radius of prop divided by MAC)
rc = Vehicle.Propulsion.CPSetup.RotorRadius./Vehicle.Geom.LWing.MAC;
beta = -0.3612 * rc.^5 + 2.2635 * rc.^4 - 5.0715 * rc.^3 + 4.3912 * rc.^2 - 0.3255 * rc;
beta(rc>=1.9) = 1;

Vehicle.Propulsion.CPSetup.s = s;
Vehicle.Propulsion.CPSetup.kd = kd;
Vehicle.Propulsion.CPSetup.beta = beta;
Vehicle.Propulsion.CPSetup.kdxbeta = kd.*beta;

