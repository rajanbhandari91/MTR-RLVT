function Vehicle = PrepMassPropertiesTable(Vehicle)
global Settings
Geom = Vehicle.Geom;

%%%% Function to run through each component of the Vehicle.Geom structure,
%%%% prepare the mass properties table, and append it to Vehicle.MassProp

% each component of Vehicle.Geom must have the following elements
% Mass
% CG
% Ixx
% Iyy
% Izz
% Ixy
% Iyz
% Izx

Components = fieldnames(Geom);

n = length(Components);

WBU = table();
WBU.Component = repmat({''},[n,1]);
WBU.Mass = zeros(n,1);
WBU.xCG = zeros(n,1);
WBU.yCG = zeros(n,1);
WBU.zCG = zeros(n,1);
WBU.Ixx = zeros(n,1);
WBU.Iyy = zeros(n,1);
WBU.Izz = zeros(n,1);
WBU.Ixy = zeros(n,1);
WBU.Iyz = zeros(n,1);
WBU.Izx = zeros(n,1);

% warning('Change i to 1')
for i = 1:n
    % component name
    WBU.Component(i) = Geom.(Components{i}).Name;
    % component architecture type
    %WBU.Architecture(i) = Geom.(Components{i}).Architecture;
    % component mass
    WBU.Mass(i) = sum(Geom.(Components{i}).Mass);
    % component cg X position
    WBU.xCG(i) = sum(Geom.(Components{i}).Mass'.*Geom.(Components{i}).CG(:,1))./sum(Geom.(Components{i}).Mass);
    % component cg Y position
    WBU.yCG(i) = sum(Geom.(Components{i}).Mass'.*Geom.(Components{i}).CG(:,2))./sum(Geom.(Components{i}).Mass);
    % component cg Z position
    WBU.zCG(i) = sum(Geom.(Components{i}).Mass'.*Geom.(Components{i}).CG(:,3))./sum(Geom.(Components{i}).Mass);
    % Ixx
    WBU.Ixx(i) = sum(Geom.(Components{i}).Ixx);
    % Iyy
    WBU.Iyy(i) = sum(Geom.(Components{i}).Iyy);
    % Izz
    WBU.Izz(i) = sum(Geom.(Components{i}).Izz);
    % Ixy
    WBU.Ixy(i) = sum(Geom.(Components{i}).Ixy);
    % Iyz
    WBU.Iyz(i) = sum(Geom.(Components{i}).Iyz);
    % Izx
    WBU.Izx(i) = sum(Geom.(Components{i}).Izx);
    % Percent Gross Weight
    WBU.PercGW(i) = 100*(WBU.Mass(i)/Vehicle.MassProp.MTOM_kg);
    
    
    if(Settings.plotFLAG == 1 && WBU.Mass(i)>0.01)
        hold on
        scatter3(Geom.(Components{i}).CG(:,1),Geom.(Components{i}).CG(:,2),Geom.(Components{i}).CG(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k')
    end
    
end

%% Grouping Certain Components
% Structures Group
[Fuselage,~] = CombinedSpecifiedRows(WBU,{'Fuselage'},"Fuselage");
[Wing,~] = CombinedSpecifiedRows(WBU,{'Left Wing','Right Wing'},"Wing");
[HTail,~] = CombinedSpecifiedRows(WBU,{'Right Horizontal Tail','Left Horizontal Tail'},"Horizontal Tail");
[VTail,~] = CombinedSpecifiedRows(WBU,{'Vertical Tail'},"Vertical Tail");
[Prop_Rails,~] = CombinedSpecifiedRows(WBU,{'Pylons_1','Pylons_2',},"Propulsor Rails");
[Nacelles,~] = CombinedSpecifiedRows(WBU,{'Nacelle_1','Nacelle_2','Nacelle_3','Nacelle_4','Nacelle_5','Nacelle_6'},"Nacelles") ;
[LDG,~] = CombinedSpecifiedRows(WBU,{'LandingGear_Main_1','LandingGear_Main_2','LandingGear_Nose','MainWheel_1','MainWheel_2','NoseWheel'},"Landing Gears");
% Propulsion
[Props,~] = CombinedSpecifiedRows(WBU,{'Prop_1','Prop_2','Prop_3','Prop_4','Prop_5','Prop_6'},"Props");
[Motors,~] = CombinedSpecifiedRows(WBU,{'Motors'},"Motors");

[PowerCabling,~] = CombinedSpecifiedRows(WBU,{'Power Cabling'},"Power Cabling");

[TS,~] = CombinedSpecifiedRows(WBU,{'Turboshaft'},"Turboshafts");
[GEN,~] = CombinedSpecifiedRows(WBU,{'Generator'},"Generators");
[GB,~] = CombinedSpecifiedRows(WBU,{'Gearbox'},"Gearboxes");

% Systems
[Elec,~] = CombinedSpecifiedRows(WBU,{'Electrical'},"Electrical");
[FC,~] = CombinedSpecifiedRows(WBU,{'Flight Controls'},"Flight Controls");
[Hyd,~] = CombinedSpecifiedRows(WBU,{'Hydraulics'},"Hydraulics");
[AVI,~] = CombinedSpecifiedRows(WBU,{'Avionics'},"Avionics");
[Fur,~] = CombinedSpecifiedRows(WBU,{'Furnishings'},"Furnishings");
[AEE,~] = CombinedSpecifiedRows(WBU,{'All-Else Empty'},"All-Else Empty");
% Energy
[BAT,~] = CombinedSpecifiedRows(WBU,{'Batteries'},"Batteries");
% Payload
[PAX,~] = CombinedSpecifiedRows(WBU,{'Payload'},"Payload");



StructuresTemp = [Fuselage;Wing;HTail;VTail;Prop_Rails;Nacelles;LDG];
[Structures,Structures_Concat] = CombinedSpecifiedRows(StructuresTemp,[],">>> STRUCTURES");

PropulsionTemp = [Props;Motors;PowerCabling;TS;GEN;GB];
[Propulsion,Propulsion_Concat] = CombinedSpecifiedRows(PropulsionTemp,[],{'>>> PROPULSION'});

[System,System_Concat] = CombinedSpecifiedRows(WBU,{'Flight Controls',...
    'Hydraulics','Avionics','Furnishings','Electrical','FuelTank_1','FuelTank_2','All-Else Empty'},{'>>> SYSTEMS'});

[Energy,Energy_Concat] = CombinedSpecifiedRows(WBU,{'Fuel','Batteries'},{'>>>>> ENERGY'});

[Payload,Payload_Concat] = CombinedSpecifiedRows(WBU,{'Payload',...
    },">>>>> PAYLOAD");


EmptyWeightTable = [Structures; Propulsion; System;];
[OEW_STATE,OEW_Concat] = CombinedSpecifiedRows(EmptyWeightTable,[],">>>>> EMPTY WEIGHT");


ZeroEnergyTable = [OEW_STATE; Payload];
[ZEW_STATE,ZEW_Concat] = CombinedSpecifiedRows(ZeroEnergyTable,[],">>>>> ZERO ENERGY WEIGHT");

FullEnergyTable = [OEW_STATE;Energy];
[MEW_STATE,MEW_Concat] = CombinedSpecifiedRows(FullEnergyTable,[],">>>>> MAX ENERGY WEIGHT");

SubGroupTable = [OEW_STATE; Payload ; Energy];
[GROSSWEIGHT_STATE,TOTAL_Concat] = CombinedSpecifiedRows(SubGroupTable,[],">>>>> GROSS WEIGHT");



FullTable = [Structures_Concat; Propulsion_Concat; System_Concat; OEW_STATE;Payload_Concat; ZEW_STATE; Energy_Concat; GROSSWEIGHT_STATE];


% Check for errors. Did anything fall through the cracks? Did anything get
% summed twice?
% Do a straight summation of all components
[CROSSCHECK,~] = CombinedSpecifiedRows(WBU,[],"CrossCheck");

% mass correct?
MassOK = abs(CROSSCHECK.Mass - GROSSWEIGHT_STATE.Mass) < 1e-6;

% xcg correct?
xCGOK = abs(CROSSCHECK.xCG - GROSSWEIGHT_STATE.xCG) < 1e-6;

% ycg correct?
yCGOK = abs(CROSSCHECK.yCG - GROSSWEIGHT_STATE.yCG) < 1e-6;

% xcg correct?
zCGOK = abs(CROSSCHECK.zCG - GROSSWEIGHT_STATE.zCG) < 1e-6;


ErrorInGroupSummation = 1 - MassOK * xCGOK * yCGOK * zCGOK;

Vehicle.MassProp.WeightFrac = CROSSCHECK.Mass / Vehicle.MassProp.MTOM_kg;




Vehicle.MassProp.TOGM_kg = (CROSSCHECK.Mass);


Vehicle.MassProp.OEM_kg = (OEW_STATE.Mass);
% Vehicle.MassProp.PAYLOAD_kg = (Payload.Mass);
Vehicle.MassProp.ZEM_kg = (ZEW_STATE.Mass);
Vehicle.MassProp.FUEL_kg = (Energy_Concat.Mass(1));
Vehicle.MassProp.BATT_kg = (Energy_Concat.Mass(2));

Vehicle.MassProp.Mass = (CROSSCHECK.Mass);


Vehicle.MassProp.CG = [CROSSCHECK.xCG, CROSSCHECK.yCG, CROSSCHECK.zCG];
Vehicle.MassProp.r_cg = Vehicle.MassProp.CG';

Vehicle.MassProp.Ixx = (CROSSCHECK.Ixx);
Vehicle.MassProp.Iyy = (CROSSCHECK.Iyy);
Vehicle.MassProp.Izz = (CROSSCHECK.Izz);
Vehicle.MassProp.Ixy = (CROSSCHECK.Ixy);
Vehicle.MassProp.Iyz = (CROSSCHECK.Iyz);
Vehicle.MassProp.Ixz = (CROSSCHECK.Izx);

Vehicle.MassProp.MOI = [...
    Vehicle.MassProp.Ixx, Vehicle.MassProp.Ixy, Vehicle.MassProp.Iyz;
    Vehicle.MassProp.Ixy, Vehicle.MassProp.Iyy, Vehicle.MassProp.Iyz;
    Vehicle.MassProp.Ixz, Vehicle.MassProp.Iyz, Vehicle.MassProp.Izz];




% long form weight breakdown
Vehicle.MassProp.WBD_Long = [WBU;GROSSWEIGHT_STATE];


% abbreviated form weight breakdown
Vehicle.MassProp.WBD_Abbr = [OEW_Concat(1:end-1,:); TOTAL_Concat];

% table for paper
Vehicle.MassProp.WBD_Paper = [Fuselage;Wing;HTail;VTail;Nacelles;Prop_Rails;LDG;Structures;...
    Propulsion_Concat;...
    System_Concat...
    ;OEW_STATE;Payload;Energy;ZEW_STATE;MEW_STATE;GROSSWEIGHT_STATE];

Vehicle.MassProp.ErrorInGroupSummation = ErrorInGroupSummation;

% Vehicle.MassProp.WBD_Paper

if Settings.plotFLAG == 1
    hold on
    scatter3(CROSSCHECK.xCG,CROSSCHECK.yCG,CROSSCHECK.zCG,200,'MarkerEdgeColor','b','MarkerFaceColor','b')
end


Stopper = 1;
%% Embedded Functions
    function [CombinedRow,Concat_Table] = CombinedSpecifiedRows(Data,SpecNames,CombinedRowName)
        % assign group name
        CombinedRow.Component = CombinedRowName;
        
        % if some names are specified...
        if~isempty(SpecNames)
        for j = 1:length(SpecNames)
            for z = 1:length(Data.Component)
                y(z) = strcmp(SpecNames(j),Data.Component{z});
            end
            k(j) = find(y == 1);
            
        end
        end
        
        % if no names are specified, sum everything...
        if isempty(SpecNames)
            k = 1:height(Data);
        end
        
        % List components and then summation at bottom
        % sum masses
%         CombinedRow.Architecture = Architecture;
        CombinedRow.Mass = sum(Data.Mass(k));
        
        % calculate combined CG
        CombinedRow.xCG = sum(Data.xCG(k).*Data.Mass(k))/sum(Data.Mass(k));
        CombinedRow.yCG = sum(Data.yCG(k).*Data.Mass(k))/sum(Data.Mass(k));
        CombinedRow.zCG = sum(Data.zCG(k).*Data.Mass(k))/sum(Data.Mass(k));
        
        % add moments and products of inertia
        CombinedRow.Ixx = sum(Data.Ixx(k));
        CombinedRow.Iyy = sum(Data.Iyy(k));
        CombinedRow.Izz = sum(Data.Izz(k));
        
        CombinedRow.Ixy = sum(Data.Ixy(k));
        CombinedRow.Iyz = sum(Data.Iyz(k));
        CombinedRow.Izx = sum(Data.Izx(k));
        CombinedRow.PercGW = sum(Data.PercGW(k));
        CombinedRow = struct2table(CombinedRow);
        Concat_Table = [Data(k,:);CombinedRow];
    end
end