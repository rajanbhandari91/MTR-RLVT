function [Cases, PropDef, DuctDef, CenterbodyDef] = ReadSetupFile(FileName)

% check license
[~] = CheckClearToRun();


% Conversion factors
g = 9.81;
conv_ft_to_m = 1/3.28;
conv_kt_to_ms = 0.51444;
conv_fpm_to_ms = 0.00508;

% read the excel file
[NUM0,TXT0,~]=xlsread(FileName);

% capture propeller airfoil sectional properties
PropFoil = TXT0{8,2};
loadFileName = ['PropDesign/Airfoil/',PropFoil];
AFLoad = load(loadFileName);
PropDef = AFLoad.FoilDef;

PropDef.Airfoil = TXT0{9,2};

% FIXED CHARACTERISTICS OF THE PROPELLER
PropDef.Nblades = NUM0(1,1);                                            % Input the Number of Blades
PropDef.r_t = NUM0(2,1)/2;                                           % tip radius(m)
PropDef.HubRadiusRatio = NUM0(3,1);
PropDef.r_h = PropDef.HubRadiusRatio * PropDef.r_t;                           % hub radius(m)


Type = NUM0(4,1);

PropDef.Type(Type==1) = {'fixed-pitch'};
PropDef.Type(Type==2) = {'variable-pitch'};

PropDef.BP75R_min = NUM0(5,1);
PropDef.BP75R_max = NUM0(6,1);

PropDef.XIdes = (NUM0(~isnan(NUM0(:,16)),16))';
PropDef.CLdes = (NUM0(~isnan(NUM0(:,17)),17))';



DuctDef = [];
GenerateDuct = NUM0(11,1);

if GenerateDuct == 1
    
    DuctFoil = TXT0{13,2};
    loadFileName = ['PropDesign/Airfoil/',DuctFoil];
    DuctAF = load(loadFileName);
    DAF = DuctAF.DAF;
    
    Duct.Airfoil = DAF;                 % duct airfoil coordinates
    Duct.RotorRadius_m = PropDef.r_t;   % outer radius of the rotor that the duct goes around
    Duct.RotorPosnOnDuctChord = NUM0(13,1);    % Location of rotor tip on duct chord [0-1]
    Duct.TipClearanceRatio = NUM0(14,1);      % Tip clearance / Rotor outer radius ratio
    Duct.AspectRatio = NUM0(15,1);              % Duct diameter/chord
    Duct.IncidenceAngle = NUM0(16,1);            % Incidence angle of duct w.r.t. rotor axis (deg)
    Duct.RotorLoc = [0,0,0];            % location of center of rotor (m)
    Duct.PitchAngle = 0;                % pitch inclination of duct axis (deg)
    Duct.BankAngle = 0;                 % roll inclination of duct axis (deg)
    
    DuctDef = Duct;
    
end


CenterbodyDef = [];
GenerateCenterbody = NUM0(21,1);

if GenerateCenterbody == 1
    
    CBAirfoil = TXT0{23,2};
    loadFileName = ['PropDesign/Airfoil/',CBAirfoil];
    CBAirfoil = load(loadFileName);
    CBAF = CBAirfoil.CBAF;
    
    CenterbodyDef.Airfoil = CBAF;
    CenterbodyDef.RotorPosnOnCB = NUM0(23,1);
    CenterbodyDef.Length = NUM0(24,1);
    CenterbodyDef.RadiusRatio = NUM0(25,1);
    CenterbodyDef.FinenessRatio = NUM0(26,1);
    
    HowManyEmpty = sum(isempty(CenterbodyDef.Length) + isempty(CenterbodyDef.RadiusRatio) + isempty(CenterbodyDef.FinenessRatio));
    
    if HowManyEmpty>1
        warning('Insufficient information on centerbody dimensions. Using defaults.')
        CenterbodyDef.FinenessRatio = 6;
        CenterbodyDef.RadiusRatio = 0.2;
        CenterbodyDef.Length = [];
    end
    
    if isempty(CenterbodyDef.Length)||isnan(CenterbodyDef.Length)
       CBDiameter = 2 * PropDef.r_t * CenterbodyDef.RadiusRatio;
       CenterbodyDef.Diameter = CBDiameter;
       
       CenterbodyDef.Length = CBDiameter*CenterbodyDef.FinenessRatio; 
    end
    
    if isempty(CenterbodyDef.FinenessRatio)
        CBDiameter = 2 * PropDef.r_t * CenterbodyDef.RadiusRatio;
        CenterbodyDef.Diameter = CBDiameter;
        
        CenterbodyDef.FinenessRatio = CenterbodyDef.Length/CBDiameter;
    end
    
    if isempty(CenterbodyDef.RadiusRatio)
       CBDiameter = CenterbodyDef.Length * CenterbodyDef.FinenessRatio;
       CenterbodyDef.Diameter = CBDiameter;
       
       CenterbodyDef.RadiusRatio = (CBDiameter/2)/PropDef.r_t;
    end
    
    

end

[NUM,TXT,RAW]=xlsread(FileName,'E1:O40');
n = height(NUM);

Tbl = NUM;

[nr,nc] = size(Tbl);

Cases.CaseName = RAW(4:4+n-1,1);
Cases.Specified = repmat({'UNK'},[n,1]);
Cases.Alt_ft = Tbl(:,1);
Cases.KTAS = Tbl(:,2);
Cases.V = Cases.KTAS * conv_kt_to_ms;
Cases.Mtip = Tbl(:,3);
Cases.RPM = Tbl(:,4);
Cases.nProps = Tbl(:,5);
Cases.TWR = Tbl(:,6);
Cases.Mass_kg = Tbl(:,7);

if nc>7
    Cases.LDRatio = Tbl(:,8);
else
    Cases.LDRatio = zeros(nr,1);
end

if nc>8
    Cases.ROC_fpm = Tbl(:,9);
else
    Cases.ROC_fpm = zeros(nr,1);
end

if nc>9
    Cases.Pabs_kW = Tbl(:,10);
else
    Cases.Pabs_kW = repmat(nan,[nr,1]);
end


[~,a,~,rho] = atmosisa(Cases.Alt_ft * conv_ft_to_m);
Cases.Sig = rho/1.225;

[n,~]= size(Cases);


% if number of props is not given, set to 1
Cases.nProps(isnan(Cases.nProps)) = 1;

% if ROC not given, set to 0
Cases.ROC_fpm(isnan(Cases.ROC_fpm)) = 0;

% if L/D not given, set to 0
Cases.LDRatio(isnan(Cases.LDRatio)) = 0;

% find cases where RPM is given
indRPMgiven = find(~isnan(Cases.RPM));
% for these cases, determine Tip Mach
if~isempty(indRPMgiven)
    Cases.Mtip(indRPMgiven) = sqrt(  Cases.V(indRPMgiven).^2  +   (PropDef.r_t * Cases.RPM(indRPMgiven)*(2*pi/60)).^2   )./ a(indRPMgiven);
end

% find cases where tip Mach is given
indMtipgiven = find(~isnan(Cases.Mtip));
% for these cases, determine the RPM
if~isempty(indMtipgiven)
    Cases.RPM(indMtipgiven) = (sqrt(  (Cases.Mtip(indMtipgiven).*a(indMtipgiven)).^2 - Cases.V(indMtipgiven).^2     )   /PropDef.r_t)*60/(2*pi);
end


% for cases where TWR is given, compute thrust based on it
indTWRgiven = find(~isnan(Cases.TWR));
if~isempty(indTWRgiven)
    Cases.Treq_N(indTWRgiven,1) = Cases.TWR(indTWRgiven) .* Cases.Mass_kg(indTWRgiven) * g./Cases.nProps(indTWRgiven);
end

% for forward flight cases (V>0 and L/D given), compute thrust based on L/D
indfwdflt = find(Cases.KTAS>0 & ~isnan(Cases.LDRatio));
if~isempty(indfwdflt)
    Cases.Treq_N(indfwdflt,1) =  Cases.Mass_kg(indfwdflt) * g.* (1./Cases.LDRatio(indfwdflt) + Cases.ROC_fpm(indfwdflt) * conv_fpm_to_ms./(Cases.KTAS(indfwdflt) * conv_kt_to_ms))./Cases.nProps(indfwdflt) ;
    Cases.TWR(indfwdflt,1) = Cases.Treq_N(indfwdflt,1).*Cases.nProps(indfwdflt)./(g.*Cases.Mass_kg(indfwdflt));
end


% find cases where power absorbed is specified
indPwrgiven = find(~isnan(Cases.Pabs_kW));

% replace NANs by 0
Cases.Pabs_kW(isnan(Cases.Pabs_kW)) = 0;

% for such cases, zero out the following
Cases.Treq_N(indPwrgiven,1) = 0;
Cases.LDRatio(indPwrgiven) = 0;
Cases.ROC_fpm(indPwrgiven) = 0;

Cases = struct2table(Cases);

end




