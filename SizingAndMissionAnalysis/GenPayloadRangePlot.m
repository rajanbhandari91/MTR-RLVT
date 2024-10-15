% This script is intended to generate a payload range chart


close all




HPL = ODMission(1).Payload_kg;
EPL = ODMission(2).Payload_kg;
FPL = ODMission(3).Payload_kg;

HRange = ODMission(1).Range_km;
ERange = ODMission(2).Range_km;
FRange = ODMission(3).Range_km;

PLPatch = [...
    0,0
    0,HPL
    HRange,HPL
    ERange,EPL
    FRange,FPL
    FRange,0
    0,0];


OEMPatch = [...
    0,0
    0,Vehicle.MassProp.OEM_kg
    FRange,Vehicle.MassProp.OEM_kg
    FRange,0
    0,0];
OEM = Vehicle.MassProp.OEM_kg;

PLPatch2 = PLPatch + [0,Vehicle.MassProp.OEM_kg];



HF = ODMission(1).EnergyMass_kg(1);
EF = ODMission(2).EnergyMass_kg(1);
FF = ODMission(3).EnergyMass_kg(1);

FPatch = [...
    0,0
    HRange,HF
    ERange,EF
    FRange,FF
    FRange,0
    0,0];

FPatch2 = FPatch + [...
    0,OEM + HPL; 
    0, OEM + HPL; 
    0, OEM + EPL; 
    0, OEM + FPL; 
    0, OEM + FPL; 
    0,OEM + FPL] ;


HGW = ODMission(1).TOGM_kg;
EGW = ODMission(2).TOGM_kg;
FGW = ODMission(3).TOGM_kg;

GWPatch = [...
    0,0
    0,OEM+HPL
    HRange,OEM+HPL + HF
    ERange,OEM+EPL + EF
    FRange,OEM+FPL + FF
    FRange,0];

figure

nr = 1;
nc = 2;
subplot(nr,nc,1)
hold on
xlabel('Range (km)')
ylabel('Payload (kg)')
patch(PLPatch(:,1),PLPatch(:,2),'y')
hold off

subplot(nr,nc,2)
hold on
xlabel('Range (km)')
ylabel('Mass (kg)')

patch(GWPatch(:,1),GWPatch(:,2),[0,0,0.8],'displayname','Fuel Mass')
patch(OEMPatch(:,1),OEMPatch(:,2),[0.5 0.5 0.5],'displayname','Empty Mass')
patch(PLPatch2(:,1),PLPatch2(:,2),'y','displayname','Payload Mass')
legend('show')
