clc
clear


% Read the mass properties case definitions
[NUM,TXT,RAW]=xlsread('TestPointsDefn.xlsx','UserDefn','A6:L100');
[numwtcond,~] = size(NUM);
MassPropMatrix = NUM(:,3:end);

MassPropCases = table();
MassPropCases.WtConfig = RAW(1:numwtcond,2);
MassPropCases.Mass = MassPropMatrix(:,1);
MassPropCases.xCG = MassPropMatrix(:,2);
MassPropCases.yCG = MassPropMatrix(:,3);
MassPropCases.zCG = MassPropMatrix(:,4);
MassPropCases.Ixx = MassPropMatrix(:,5);
MassPropCases.Iyy = MassPropMatrix(:,6);
MassPropCases.Izz = MassPropMatrix(:,7);
MassPropCases.Ixy = MassPropMatrix(:,8);
MassPropCases.Ixz = MassPropMatrix(:,9);
MassPropCases.Iyz = MassPropMatrix(:,10);



% Read the flight condition definitions
[NUM,TXT,RAW]=xlsread('TestPointsDefn.xlsx','UserDefn','P3:S100');
[numaltcond,~] = size(NUM);

Alt = [];
Vel = [];
VelType = [];
EAS = [];
TAS = [];
MACH = [];
AltIndex = [];

for i = 1:1:numaltcond
    
    AltRead = RAW{i,1};
    VelTypeRead = RAW(i,3);
    VelSpec = RAW{i,2};
    
    F = strcat('Vtemp=[',VelSpec,'];');
    eval(F);
    
    nvel = length(Vtemp);
    
    AltIndex = [AltIndex;repmat(i,[nvel,1])];
    
    AltRep = repmat(AltRead,[nvel,1]);
    VelRep = reshape(Vtemp,[nvel,1]);
    VelTypeRep = repmat(VelTypeRead,[nvel,1]);
    
    Alt = [Alt;AltRep];
    Vel = [Vel;VelRep];
    VelType = [VelType;VelTypeRep];
    size(VelType);
    
    
    [~,a,~,rho] = atmosisa(AltRep/3.28);
%     a = a';
%     rho = rho';
    
    if strcmpi(VelTypeRead,'KEAS')
        EAS1 = VelRep;
        TAS1 = EAS1.*sqrt(1.225./rho);
        MACH1 = 0.514444 * TAS1./a;
    end
    
    if strcmpi(VelTypeRead,'KTAS')
        TAS1 = VelRep;
        EAS1 = TAS1.*sqrt(rho./1.225);
        MACH1 = 0.514444 * TAS1./a;
    end
    
    if strcmpi(VelTypeRead,'MACH')
        MACH1 = VelRep;
        TAS1 = MACH1.*a / 0.514444;
        EAS1 = TAS1.*sqrt(rho./1.225);
    end
    
    EAS = [EAS;EAS1];
    TAS = [TAS;TAS1];
    MACH = [MACH;MACH1];
    
end

FltCondTbl = table();
FltCondTbl.Alt = Alt;
FltCondTbl.KTAS = TAS;
FltCondTbl.KEAS = EAS;
FltCondTbl.MACH = MACH;



[nfltcond,~] = size(FltCondTbl);

TestPoints = [];
ctr = 1;

for i= 1:1:nfltcond
    
    MPIndexRead = RAW{AltIndex(i),4};
    
    if length(MPIndexRead)>1
        F = strcat('MPIndex=[',MPIndexRead,'];');
    end
    if length(MPIndexRead)==1
        F = strcat('MPIndex=',num2str(MPIndexRead));
    end
    eval(F);
    
    T = repmat(FltCondTbl(i,:),[length(MPIndex),1]);
    
    S = MassPropCases(MPIndex,:);
    
    TPnew = [T,S];
    
    TestPoints = [TestPoints;TPnew];
end

TestPoints = sortrows(TestPoints,'WtConfig','ascend');
[ntp,~] = size(TestPoints);

TPNum = table();
TPNum.TestPoint = [1:1:ntp]';

TestPoints = [TPNum,TestPoints];





xlswrite('TestPointsDefn.xlsx',repmat({''},[1000,1000]),'TestPoints','A5')
xlswrite('TestPointsDefn.xlsx',table2cell(TestPoints),'TestPoints','A5')
xlswrite('TestPointsDefn.xlsx',table2cell(FltCondTbl),'TestPoints','R5')

% TestPoints.VTAS = TestPoints.KTAS*0.514444;
% TestPoints.Alt_m = TestPoints.Alt/3.28;

TestPoints

save('./MADCASP/TestPointsFull.mat','TestPoints');
save('./MADCASP/MassPropCases.mat','MassPropCases');
