function Vehicle = VehicleGeometryUpdater(Vehicle)


%% Plotting
global Settings
% Settings.plotFLAG = 1;
if Settings.plotFLAG == 1
    figure(1); %('Position',[250 200 1000 800])
    drawnow
    axis equal
    xlim([-10, 10])
    ylim([-10, 10])
    zlim([-10, 10])
    set ( gca, 'ydir', 'reverse' )
    set ( gca, 'zdir', 'reverse' )
    set ( gca, 'Clipping', 'off')
    hold on
    view([1,1,1])

    h = gca;
    plot3(h.XLim, [0 0], [0 0], '.-r')
    plot3([0, 0], h.YLim, [0 0], '.-r');
    plot3([0, 0], [0 0], h.ZLim, '.-r');
end



% settings
FanArm_m = 4.4;
PaxCabinHeight_m = 4.5/3.28;

% horizontal stabilizer positioning
fs_hstab = 0.98;
VT_HT_ChordRatio = 1.1;

cs_gap = 0.02;


PropXStagger_m = 0.20;


PropLocOnNacelle_FS = 0.20;
NacelleEndOnWingChord = 0.50;


%% Component Mounting etas
% Wing
WingMountingHeight = 0.80;
% Vertical stabilizer
VertStab_FS = 0.95;
VertStabMountingHeight = 1;
% Landing Gear
LDGeta = 0.2;
LDGdiff = 0.55;
% Power Cabling
PC_ref_L = 0.5;
PC_ref_h = 0;
% Electrical
Elec_ref_L = 0.6;
Elec_ref_h = 0;
% Flight Controls
FC_ref_L = 0.5;
FC_ref_h = 0;
% Hydraulics
Hyd_ref_L = 0.6;
Hyd_ref_h = 0;
% Avionics
AV_ref_L = .4;
AV_ref_h = 0;
% Furnishings
Fur_ref_L = 0.55;
Fur_ref_h = 0.2;
% Batteries
BAT_ref_L = 0.15;
BAT_ref_h = 0.5;
% Passengers
PAX_ref_L = 0.35;
PAX_ref_h = 0;
aPAX_ref_L = 0.65;

% Fore Center Rotor 1
rot1_Ref_L = 0.09;
rot1_Ref_ht = 0.22;
% Aft Center Rotor 2
rot2_Ref_L = 0.79;



% landing gear
MainGear_fs = 0.65;
NoseGear_fs = 0.15;

% booms (and lift props)
eta_IBD_liftprops = 0.3605;
eta_OBD_liftprops = 0.7675;
etamax_OBD_liftprops = 0.90;


HS_refx = 0.85;
HS_refz = 0.95;

Can_refx = 0.1;
Can_refz = -.15;

Wing_refx = 0.55;
Wing_refz =.9;

VS_refx = 1;
VS_refz = 0.9;


eta_eng = 0;   %original was 0.5
eta_eng_fus = 0.6;
PylonSpan = 0.25;
% update the number of time the geometry updater has run
Vehicle.Tracker.GeomUpdaterRuns = Vehicle.Tracker.GeomUpdaterRuns + 1;



%% Geometry Re-sizing rules
ResizingCalculations;


%% Fuselage
% Vehicle.Geom.Fus.MaxHeight = PaxCabinHeight_m + Recalcs.WingRootThickness;
[Vehicle.Geom.Fus] =  GeomEval_Fuselage(Vehicle.Geom.Fus);

%% Cargo cuboid with x-location placed at 0 
Vehicle.Geom.LD_3.RefPtLocation(1) = 0;
Vehicle.Geom.LD_3.RefPtLocation(2) = 0;
CargoZLoc = interp1(Vehicle.Geom.Fus.Stn.Xc, [Vehicle.Geom.Fus.Stn.Zc, Vehicle.Geom.Fus.Stn.Height], 0);

Vehicle.Geom.LD_3.RefPtLocation(3) = CargoZLoc(1) + CargoZLoc(2)*0.0;

Vehicle.Geom.LD_3.xDim_m = Vehicle.Geom.LD_3.Length_x;
Vehicle.Geom.LD_3.yDim_m = Vehicle.Geom.LD_3.Width_y /2;
Vehicle.Geom.LD_3.zDim_m = Vehicle.Geom.LD_3.Height_z /2;

Vehicle.Geom.LD_3 = GeomEval_GenericComponent(Vehicle.Geom.LD_3);

%% WING GEOMETRY
Vehicle.Recalcs = Recalcs;
Vehicle = WingGeometryCalcs(Vehicle);
 
%% Stabilizer Sizing (Included in SnC eval now)
[Vehicle] = StabilizerGeometryCalculations(Vehicle);


%% Wing-mounted propeller geometry calculations
% set diameter
for iprop = 1:Vehicle.Propulsion.NProps
    PropName = horzcat('Prop_', num2str(iprop));
    Vehicle.Geom.(PropName).Diam = Vehicle.Recalcs.PropDiam_m;
end

% locate the plane of the propellers in the forward flight position
% note: coincides with junction between cockpit and cabin sections
% note: driven by blade failure considerations
fusquery = Vehicle.Geom.Fus.Stn(Vehicle.Geom.Fus.Stn.CS==2, :);
propplane_x = fusquery.Xc(end);



PropXStagger_m = 0.70*Vehicle.Geom.LWing.MAC;
PropXPosnsLeftWing = interp1(Vehicle.Geom.LWing.Stn.eta, Vehicle.Geom.LWing.Stn.xLE, Vehicle.Recalcs.PropEtas,'linear') + PropXStagger_m;
PropXPosnsRightWing = interp1(Vehicle.Geom.RWing.Stn.eta, Vehicle.Geom.RWing.Stn.xLE, Vehicle.Recalcs.PropEtas,'linear') + PropXStagger_m;

% get the y locations
PropYPosnsLeftWing = interp1(Vehicle.Geom.LWing.Stn.eta, Vehicle.Geom.LWing.Stn.yQC, Vehicle.Recalcs.PropEtas,'linear');
PropYPosnsRightWing = interp1(Vehicle.Geom.RWing.Stn.eta, Vehicle.Geom.RWing.Stn.yQC, Vehicle.Recalcs.PropEtas,'linear');

% get the z locations
PropZPosnsLeftWing = interp1(Vehicle.Geom.LWing.Stn.eta, Vehicle.Geom.LWing.Stn.zQC, Vehicle.Recalcs.PropEtas,'linear');
PropZPosnsRightWing = interp1(Vehicle.Geom.RWing.Stn.eta, Vehicle.Geom.RWing.Stn.zQC, Vehicle.Recalcs.PropEtas,'linear');

% evaluate props, left OBD to right OBD
for iprop = 1:Vehicle.Propulsion.NProps

    PropName = horzcat('Prop_', num2str(iprop));  

    if iprop <= Vehicle.Propulsion.NProps/2   % left half
        posID = Vehicle.Propulsion.NProps/2 - (iprop-1);
        Vehicle.Geom.(PropName).RefPtLocation = [PropXPosnsLeftWing(posID) PropYPosnsLeftWing(posID) PropZPosnsLeftWing(posID)]';
    else                                      % right half
        posID = iprop - Vehicle.Propulsion.NProps/2;
        Vehicle.Geom.(PropName).RefPtLocation = [PropXPosnsRightWing(posID) PropYPosnsRightWing(posID) PropZPosnsRightWing(posID)]' ;
    end

    Vehicle.Geom.(PropName) = GeomEval_DuctedFan(Vehicle.Geom.(PropName));
end


% generate data for AC3D geometry
for i = 1:Vehicle.Propulsion.Setup.nRotor

    SpecSetup = Vehicle.Propulsion.Setup;
    SpecSetup.Chord = Vehicle.Propulsion.Setup.ChordDist;
    SpecSetup.Pitch = Vehicle.Propulsion.Setup.PitchDistDeg;
    MCSFlag = 0;

    PropName = sprintf('Prop_%0.0f',i);
    Vehicle.Propulsion.Setup.RotorLoc(:,i) = Vehicle.Geom.(PropName).RefPtLocation;
    [TempRotor] = GenerateRotors(Vehicle.Propulsion.Setup.RotorLoc(:,i)',Vehicle.Propulsion.Setup.RotorAxisPhi(i),Vehicle.Propulsion.Setup.RotorAxisTheta(i),Vehicle.Propulsion.Setup.SpinDir(i),SpecSetup,Vehicle.Propulsion.Setup.Airfoil,1,[1,0,0],'blank',MCSFlag);

    Vehicle.Geom.(PropName).Vertices = TempRotor.Vertices;
    Vehicle.Geom.(PropName).Surfaces = TempRotor.Surfaces;
end

%% TURBOSHAFT ENGINES
% query mounting point of turbohaft engine
TS_fs_search = Vehicle.Geom.Fus.Stn(Vehicle.Geom.Fus.Stn.CS==3,:);
TS_fs = TS_fs_search.FS(end);
TS_Mtg = interp1(Vehicle.Geom.Fus.Stn.FS,[Vehicle.Geom.Fus.Stn.Xc],TS_fs,'linear');

Engine_x = TS_Mtg(1) - 0.15; %Vehicle.MassProp.TargetCG(1) - 0.1;

Engine_x = Vehicle.Geom.LD_3.RefPtLocation(1) - Vehicle.Geom.LD_3.xDim_m/2 - 0.60;

% ENGINE 1
fus_query = interp1(Vehicle.Geom.Fus.Stn.Xc, [Vehicle.Geom.Fus.Stn.Zc, Vehicle.Geom.Fus.Stn.Height] , Engine_x,'linear');

Engine_y = 0.6/2;
Engine_z = fus_query(1) + fus_query(2)/4 - Vehicle.Geom.Turboshaft1.Nac.MaxHeight;


Vehicle.Geom.Turboshaft1.Nac.Directionality = -1;

Vehicle.Geom.Turboshaft1.Nac.RefPtLocation = [Engine_x, Engine_y, Engine_z];

% evaluate engine 1
[Vehicle.Geom.Turboshaft1] = GeomEval_PylonNacelle(Vehicle.Geom.Turboshaft1);
Vehicle.Geom.Turboshaft1.CG = Vehicle.Geom.Turboshaft1.Nac.RefPtLocation;

% evaluate engine 2
Vehicle.Geom.Turboshaft2 = Vehicle.Geom.Turboshaft1;
Vehicle.Geom.Turboshaft2.Name = {'Turboshaft2'};
Vehicle.Geom.Turboshaft2.Nac.RefPtLocation = [Engine_x, -Engine_y, Engine_z];
Vehicle.Geom.Turboshaft2.CG = Vehicle.Geom.Turboshaft2.Nac.RefPtLocation;

[Vehicle.Geom.Turboshaft2] = GeomEval_PylonNacelle(Vehicle.Geom.Turboshaft2);

% locate gearboxes
Vehicle.Geom.Gearboxes.RefPtLocation(1,:) =  Vehicle.Geom.Turboshaft1.Nac.RefPtLocation  + [Vehicle.Geom.Turboshaft1.xDim_m/2,0,0];
Vehicle.Geom.Gearboxes.RefPtLocation(2,:) =  Vehicle.Geom.Turboshaft2.Nac.RefPtLocation  + [Vehicle.Geom.Turboshaft2.xDim_m/2,0,0];


% locate generators
Vehicle.Geom.Generators.RefPtLocation(1,:) = Vehicle.Geom.Gearboxes.RefPtLocation(1,:)  + [0,0,Vehicle.Geom.Generators.zDim_m(1) * 2];
Vehicle.Geom.Generators.RefPtLocation(2,:) = Vehicle.Geom.Gearboxes.RefPtLocation(1,:)  + [0,0,Vehicle.Geom.Generators.zDim_m(2) * 2];
Vehicle.Geom.Generators.RefPtLocation(3,:) = Vehicle.Geom.Gearboxes.RefPtLocation(2,:)  + [0,0,Vehicle.Geom.Generators.zDim_m(3) * 2];
Vehicle.Geom.Generators.RefPtLocation(4,:) = Vehicle.Geom.Gearboxes.RefPtLocation(2,:)  + [0,0,Vehicle.Geom.Generators.zDim_m(3) * 2];


% evaluate the Gearboxes component
[Vehicle.Geom.Gearboxes] = GeomEval_GenericComponent(Vehicle.Geom.Gearboxes);
% evaluate the generator component
[Vehicle.Geom.Generators] = GeomEval_GenericComponent(Vehicle.Geom.Generators);

%% SPONSON
SponsonChord = 2;
SponsonSpan = 2.5;
SponsonArea = SponsonChord * SponsonSpan;
SponsonAR = SponsonSpan^2/SponsonArea;

Sponson_x = 0;
Sponson_y = -SponsonSpan/2;
Sponson_query = interp1(Vehicle.Geom.Fus.Stn.Xc, [Vehicle.Geom.Fus.Stn.Height, Vehicle.Geom.Fus.Stn.Zc],Sponson_x, 'linear');
Sponson_z = Sponson_query(2) + Sponson_query(1)/2;


Vehicle.Geom.Sponson.RefPtLocation = [Sponson_x, Sponson_y, Sponson_z];
Vehicle.Geom.Sponson.PlanformArea = SponsonArea;
Vehicle.Geom.Sponson.AspectRatio = SponsonAR;
[Vehicle.Geom.Sponson] = GeomEval_LiftingSurface(Vehicle.Geom.Sponson);





%% LANDING GEAR

GearTrack = 2;                  % lateral separation between main gear, m
BellyGroundClearance = 0.5;     % distance between belly and ground, m
StrutChord = 0.15;              % chord of landing gear strut, m
NGStaticLoadPerc = 0.10;        % percentage of static load on nose gear
MGEtaLocOnSponson = 0.95;


% fuselage lowest point
ZFusLowestPt = max(Vehicle.Geom.Fus.Stn.Zc + Vehicle.Geom.Fus.Stn.Height/2);


% query mounting point of nose gear
NoseGear_fs_search = Vehicle.Geom.Fus.Stn(Vehicle.Geom.Fus.Stn.CS==1,:);
NoseGear_fs = NoseGear_fs_search.FS(end);
NGMtg = interp1(Vehicle.Geom.Fus.Stn.FS,[Vehicle.Geom.Fus.Stn.Xc,  Vehicle.Geom.Fus.Stn.Zc, Vehicle.Geom.Fus.Stn.Width, Vehicle.Geom.Fus.Stn.Height],NoseGear_fs,'linear');

Xcenter = NGMtg(1);
Zcenter = NGMtg(2);
SemiHeight = NGMtg(4)/2;

xng = Xcenter;
yng = 0;
zng = Zcenter + 0.9*SemiHeight;

Vehicle.Geom.NG.t_min = 0.10;
Vehicle.Geom.NG.RefPtLocation = [xng,yng,zng];
Vehicle.Geom.NG.StrutChord = StrutChord;
Vehicle.Geom.NG.BellyGroundClearance = BellyGroundClearance;

% Use the height computed to get NG gear height as well
Vehicle.Geom.NG.GearHeight = BellyGroundClearance + ZFusLowestPt - zng;


% Evaluate nose gear 
[Vehicle.Geom.NG, Vehicle.Geom.NG_Wheel] = GeomEval_LandingGear(Vehicle.Geom.NG, Vehicle.Geom.NG_Wheel);


% find location of main gear to get desired weight split between nose and
% main gear
Lng = abs(xng - Vehicle.MassProp.TargetCG(1));
Lmg = Lng * NGStaticLoadPerc/(1-NGStaticLoadPerc);
xmlg = Vehicle.MassProp.TargetCG(1) - Lmg;

mt_lmg = interp1(Vehicle.Geom.Sponson.Stn.eta, [Vehicle.Geom.Sponson.Stn.yQC,Vehicle.Geom.Sponson.Stn.zQC],1-MGEtaLocOnSponson,'linear');
ymlg = mt_lmg(1);
zmlg = mt_lmg(2);
Vehicle.Geom.LMG.RefPtLocation = [xmlg,ymlg,zmlg];

% compute tail strike angle and adjust the length accordingly
% set 12 degree
TailStrAng = 12;
VSx =  Vehicle.Geom.VS.Stn.xTE(1);
VSz =  Vehicle.Geom.VS.Stn.zTE(1);
Gz = tand(TailStrAng) * (xmlg - VSx) + VSz ;

Vehicle.Geom.LMG.GearHeight = Vehicle.Geom.NG.GearHeight - 0.2; %Gz - zmlg;

Vehicle.Geom.LMG.BellyGroundClearance = BellyGroundClearance;
Vehicle.Geom.LMG.GearTrack = GearTrack;
Vehicle.Geom.LMG.StrutChord = StrutChord;
Vehicle.Geom.LMG.t_min = 0.10;

% Use the height computed to get NG gear height as well
Vehicle.Geom.NG.GearHeight = Vehicle.Geom.LMG.GearHeight + ...
    abs(Vehicle.Geom.LMG.RefPtLocation(3) - Vehicle.Geom.NG.RefPtLocation(3));


% Evaluate nose gear 
% [Vehicle.Geom.NG, Vehicle.Geom.NG_Wheel] = GeomEval_LandingGear(Vehicle.Geom.NG, Vehicle.Geom.NG_Wheel);

% evaluate left main gear
[Vehicle.Geom.LMG, Vehicle.Geom.LMG_Wheel] = GeomEval_LandingGear(Vehicle.Geom.LMG, Vehicle.Geom.LMG_Wheel);



mt_rmg = interp1(Vehicle.Geom.Sponson.Stn.eta, [Vehicle.Geom.Sponson.Stn.yQC,Vehicle.Geom.Sponson.Stn.zQC],MGEtaLocOnSponson,'linear');
xmrg = xmlg;
ymrg = mt_rmg(1);
zmrg = mt_rmg(2);
Vehicle.Geom.RMG.RefPtLocation = [xmrg,ymrg,zmrg];
Vehicle.Geom.RMG.GearHeight = Vehicle.Geom.LMG.GearHeight;
Vehicle.Geom.RMG.BellyGroundClearance = BellyGroundClearance;
Vehicle.Geom.RMG.GearTrack = GearTrack;
Vehicle.Geom.RMG.StrutChord = StrutChord;
Vehicle.Geom.RMG.t_min = 0.10;

% evaluate right main gear
[Vehicle.Geom.RMG, Vehicle.Geom.RMG_Wheel] = GeomEval_LandingGear(Vehicle.Geom.RMG, Vehicle.Geom.RMG_Wheel);


% ground contact points
Vehicle.Operations.GCLoc = [Vehicle.Geom.NG_Wheel.RefPtLocation; 
                            Vehicle.Geom.LMG_Wheel.RefPtLocation; 
                            Vehicle.Geom.RMG_Wheel.RefPtLocation; ];

%% Speedbrake 
Speedbrake_Fus_Stn = 0.4;

if isfield(Vehicle.MassProp, 'CG')
    SB_MTG_LOC = interp1( Vehicle.Geom.Fus.Stn.Zc, [Vehicle.Geom.Fus.Stn.Xc, Vehicle.Geom.Fus.Stn.Width./2], Vehicle.MassProp.CG(3));
else
    SB_MTG_LOC = [0,0,0];
end

% warning('arbitrary area for speedbrake - update')
Vehicle.Geom.Speedbrake.PlanformArea = 0.2 * Vehicle.Geom.Fus.MaxWidth*0.40; 

Speedbrake_Mounting_Point = interp1(Vehicle.Geom.Fus.Stn.FS, [Vehicle.Geom.Fus.Stn.Xc, Vehicle.Geom.Fus.Stn.Yc, Vehicle.Geom.Fus.Stn.Zc + Vehicle.Geom.Fus.Stn.Height*0.49],Speedbrake_Fus_Stn);
Vehicle.Geom.Speedbrake.RefPtLocation(1) = Speedbrake_Mounting_Point(1);
Speedbrake_span = sqrt(Vehicle.Geom.Speedbrake.AspectRatio * Vehicle.Geom.Speedbrake.PlanformArea);
Vehicle.Geom.Speedbrake.RefPtLocation(2) = Speedbrake_Mounting_Point(2) - Speedbrake_span/2;
Vehicle.Geom.Speedbrake.RefPtLocation(3) = Speedbrake_Mounting_Point(3);
Vehicle.Geom.Speedbrake.Directionality = 1;
Vehicle.Geom.Speedbrake.eta_CG_spec = 0.5;
[Vehicle.Geom.Speedbrake] = GeomEval_LiftingSurface(Vehicle.Geom.Speedbrake);

%% Nacelles
% set nacelle reference points
for iprop = 1:Vehicle.Propulsion.NProps
    NacName = horzcat('NAC_', num2str(iprop));
    PropName = horzcat('Prop_', num2str(iprop));
    Vehicle.Geom.(NacName).RefPtLocation = Vehicle.Geom.(PropName).RefPtLocation;
    Vehicle.Geom.(NacName).RefPt_FS = PropLocOnNacelle_FS;
end

% query wing LE and TW x-coordinates at prop etas
Xwing = interp1(Vehicle.Geom.LWing.eta, [Vehicle.Geom.LWing.Stn.xLE, Vehicle.Geom.LWing.Stn.xTE], Vehicle.Recalcs.PropEtas, 'linear');
XNacelleAftEnd = Xwing(:,1) + (Xwing(:,2)-Xwing(:,1)) * NacelleEndOnWingChord;

% find nacelle lengths
for iprop = 1:Vehicle.Propulsion.NProps
    NacName = horzcat('NAC_', num2str(iprop));
    if iprop <= Vehicle.Propulsion.NProps/2   % left half
        posID = Vehicle.Propulsion.NProps/2 - (iprop-1);
        Vehicle.Geom.(NacName).CrossSections.Length = (Vehicle.Geom.(NacName).RefPtLocation(1) - XNacelleAftEnd(posID))/(1-PropLocOnNacelle_FS);
    else                                      % right half
        posID = iprop - Vehicle.Propulsion.NProps/2;
        Vehicle.Geom.(NacName).CrossSections.Length = (Vehicle.Geom.(NacName).RefPtLocation(1) - XNacelleAftEnd(posID))/(1-PropLocOnNacelle_FS);
    end
    Vehicle.Geom.(NacName).FinenessRatio = 4;
    % Vehicle.Geom.(NacName).FinenessRatio = Vehicle.Geom.(NacName).CrossSections.Length/Vehicle.Recalcs.NacelleDiam_m;
end




%% Nacelle dimension updates

for iprop = 1:Vehicle.Propulsion.NProps
    NacName = horzcat('NAC_', num2str(iprop));
    [Vehicle.Geom.(NacName)] =  GeomEval_DEPNacelle(Vehicle.Geom.(NacName));
end


%% PROP MOTORS
LocPropMot = [];
for iprop = 1:Vehicle.Propulsion.NProps
    PropName = horzcat('Prop_', num2str(iprop));
    LocPropMot = vertcat(LocPropMot, Vehicle.Geom.(PropName).RefPtLocation - [Vehicle.Propulsion.PropMotorLength_m,0,0]);
end
Vehicle.Geom.PropMotors.RefPtLocation = LocPropMot;

%% energy zone calculations
% adjust energy zone limits
% % zone 1: left wing 
LWlimits = [0.05, 0.4];
Vehicle.Propulsion.EnergyZones.Limits(1,:) = LWlimits;
% % zone 2: right wing 
RWlimits = [0.05, 0.4];
Vehicle.Propulsion.EnergyZones.Limits(2,:) = RWlimits;
% zone 3: sponson 
% Sponsonlimits = [0, 1.0];
% Vehicle.Propulsion.EnergyZones.Limits(1,:) = Sponsonlimits;

Vehicle = EnergyZoneCalculator(Vehicle);

% set fuel energy zones
Vehicle.Propulsion.EnergyZones.Centroid(2) = 0; 
Vehicle.Geom.Fuel.RefPtLocation = Vehicle.Propulsion.EnergyZones.Centroid;

% evaluate fuel
[Vehicle.Geom.Fuel] = GeomEval_GenericComponent(Vehicle.Geom.Fuel);

%% Electrical
Elec_x = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,Elec_ref_L);
Elec_y = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Yc,Elec_ref_L);
Elec_z = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Zc,Elec_ref_L)...
    - Elec_ref_h * interp1(Vehicle.Geom.Fus.Stn.FS,...
    Vehicle.Geom.Fus.Stn.Height,Elec_ref_L)/2;

Vehicle.Geom.Electrical.RefPtLocation(1,1:3) = [Elec_x Elec_y Elec_z];

[Vehicle.Geom.Electrical] = GeomEval_GenericComponent(Vehicle.Geom.Electrical);
%
%% Flight Controls
FC_x = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,FC_ref_L);
FC_y = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Yc,FC_ref_L);
FC_z = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Zc,FC_ref_L)...
    - FC_ref_h * interp1(Vehicle.Geom.Fus.Stn.FS,...
    Vehicle.Geom.Fus.Stn.Height,FC_ref_L)/2;

Vehicle.Geom.Flight_Controls.RefPtLocation(1,1:3) = [FC_x FC_y FC_z];

[Vehicle.Geom.Flight_Controls] = GeomEval_GenericComponent(Vehicle.Geom.Flight_Controls);

%% Hydraulics
Hyd_x = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,Hyd_ref_L);
Hyd_y = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Yc,Hyd_ref_L);
Hyd_z = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Zc,Hyd_ref_L)...
    - Hyd_ref_h * interp1(Vehicle.Geom.Fus.Stn.FS,...
    Vehicle.Geom.Fus.Stn.Height,Hyd_ref_L)/2;

Vehicle.Geom.Hydraulics.RefPtLocation(1,1:3) = [Hyd_x Hyd_y Hyd_z];

[Vehicle.Geom.Hydraulics] = GeomEval_GenericComponent(Vehicle.Geom.Hydraulics);

%% Avionics
AV_x = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,AV_ref_L);
AV_y = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Yc,AV_ref_L);
AV_z = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Zc,AV_ref_L)...
    - AV_ref_h * interp1(Vehicle.Geom.Fus.Stn.FS,...
    Vehicle.Geom.Fus.Stn.Height,AV_ref_L)/2;

Vehicle.Geom.Avionics.RefPtLocation(1,1:3) = [AV_x AV_y AV_z];

[Vehicle.Geom.Avionics] = GeomEval_GenericComponent(Vehicle.Geom.Avionics);

%% Furnishings
Fur_x = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,Fur_ref_L);
Fur_y = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Yc,Fur_ref_L);
Fur_z = interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Zc,Fur_ref_L)...
    - Fur_ref_h * interp1(Vehicle.Geom.Fus.Stn.FS,...
    Vehicle.Geom.Fus.Stn.Height,Fur_ref_L)/2;

Vehicle.Geom.Furnishings.RefPtLocation(1,1:3) = [Fur_x Fur_y Fur_z];

[Vehicle.Geom.Furnishings] = GeomEval_GenericComponent(Vehicle.Geom.Furnishings);

%% Batteries

BAT_x = Vehicle.Geom.LD_3.RefPtLocation(1) + Vehicle.Geom.LD_3.xDim_m/2 + 1.0; % interp1(Vehicle.Geom.Fus.Stn.FS,Vehicle.Geom.Fus.Stn.Xc,BAT_ref_L);
BAT_y = 0;
BAT_z = interp1(Vehicle.Geom.Fus.Stn.Xc,Vehicle.Geom.Fus.Stn.Zc,BAT_x)...
    + interp1(Vehicle.Geom.Fus.Stn.Xc,...
    Vehicle.Geom.Fus.Stn.Height,BAT_x)/4;

Vehicle.Geom.Batteries.RefPtLocation = [BAT_x BAT_y BAT_z];

[Vehicle.Geom.Batteries] = GeomEval_GenericComponent(Vehicle.Geom.Batteries);

%% AERO-PROPULSIVE MODEL UPDATES
% % aerodynamic model

Vehicle.Propulsion.RunQMILQPROP = 0;

% Update prop diameter
Vehicle.Propulsion.PropDiam_m = Vehicle.Recalcs.PropDiam_m;

[Vehicle] = EMPAERO_UpdateAeroModel(Vehicle);

end