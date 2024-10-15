function [LS] =  GeomEval_LiftingSurface(LS)
global Settings

% Change log
% 2024.05.08 - both maximum and minimum thickness constraint implemented and verified - Bikash 
% 2024.05.08 - eta_exp, ExposedSpan and ExposedPlanformArea computed and sent to LS- Bikash 

% To do list
% 1. DONE -- calculate control surface etas based on exposed etas
% 2. DONE -- throw control surface etas into the mix
% 2.5 DONE -- display control surfaces (rudimentary)
% 2.6 DONE -- log coordinates of control surface points
% 3. DONE -- mass properties - spread out mass over strips
% 8. DONE -- strip positioning - account for propeller locations and diameter
% 9. DONE -- strip positioning - determine number of strips based on geometry
% X. DONE -- AC3D export
% 4. DONE -- mass propertes - estimate cg location per strip
% 4.5 DONE -- mass properties - get CG location of wing
% 5. DONE -- mass properties - estimate moment of inertia per strip
% 6. DONE -- mass properties - transfer inertia to wing CG
% 7. DONE -- mass properties - transfer intertia to aircraft ref point


% 2.7 Constant chord control surfaces (as opposed to constant chord frac)





% TASKS FOR NOAH:
% 1. add wetted area calculation (only "exposed" part of wing)
% 2. mean aerodynamic chord (MAC) length and location of mean aerodynamic
% center (MAC)
% 3. mean geometric chord (MGC)
% 4. control surface planform areas
% 5. control surface mean chords

index = 1;
AF = ReadAF(LS,index);

% load(LS.AirfoilName{1}.name) % load default primary airfoil
% will generate an internal workspace variable called AF


% 1. For a single lifting surface, pointing to the right
% 2. Axis definitions: x forward, y right, z down
% 3. Component Reference Point (CRP)


% Outputs from function00
% 1. Geometry definitions (for internal plotting)
% 2. Strip definitions (for aero-propulsive analysis)
% 3. AC3D definitions (for geometry export)
% 4. CG information (relative to component reference point)
% 5. inertia information (relative to component CG)

% taper default - no taper
if isempty(LS.TaperDefn)
    LS.TaperDefn = [0.00, 1.00;
        1.00, 1.00];
end

% straight taper
if length(LS.TaperDefn)==1
    LS.TaperDefn = [0.00, 1.00;
        1.00, LS.TaperDefn];
end

% sweep default - no sweep
if isempty(LS.SweepDefn)
    LS.SweepDefn = [0.00, 1.00;
        0.25, 0.25;
        0.00, 0.00];
end

% dihedral default - no dihedral
if isempty(LS.Dihedral)
    LS.Dihedral = [0.00, 1.00;
        0.00, 0.00];
end

% constant dihedral
if length(LS.Dihedral) == 1
    LS.Dihedral = [0.00, 1.00;
        LS.Dihedral, LS.Dihedral];
end


% relative twist definition default - no twist
if isempty(LS.Twist)
    LS.Twist = [0.00, 1.00;
        0.00, 0.00];
end

% linear twist rate
if length(LS.Twist)==1
    LS.Twist = [0.00, 1.00;
        0.00, LS.Twist];
end


% AR = Span^2 / PlanformArea
% If planform area is empty, calculate it
if isempty(LS.PlanformArea)
    LS.PlanformArea = LS.Span^2/LS.AspectRatio;
end



% Use planform area and aspect ratio to compute span
b = sqrt(LS.AspectRatio * LS.PlanformArea);
LS.Span = b + 0*abs(LS.RefPtLocation(2));



% Mapping between eta and exposed eta variables
% eta = aa eta' + bb
% a = eta_obd - eta_ibd
% b = eta_ibd
eta_ibd = LS.ExposedEtas(1);
eta_obd = LS.ExposedEtas(2);
aa = eta_obd - eta_ibd;
bb = eta_ibd;

[~,numCS] = size(LS.Controls);
LS.numControls = numCS;

etaControls = [];

if numCS > 0

    for i = 1:numCS

        %LS.Controls(i).eta = aa.*LS.Controls(i).EtaFrac + bb;
        LS.Controls(i).eta = LS.Controls(i).EtaFrac;
        % span fractions of control surface edges
        %         ControlsDummy = [LS.Controls(i).eta(1) - 0.001, LS.Controls(i).eta(2) + 0.001];

        etaControls = [etaControls, LS.Controls(i).eta];
    end


end


[nblock,~] = size(LS.BlockedEtas);
BlockedEtas = [];
if nblock > 0
    BlockedEtas = reshape(LS.BlockedEtas, [1, 2*nblock]);
end

% account for propellers
numProps = length(LS.PropEtas); LS.numProps = numProps;
etaPropCenters = LS.PropEtas;
etaPropObdEdges = etaPropCenters + 0.5*LS.PropDiams/b;
etaPropIbdEdges = etaPropCenters - 0.5*LS.PropDiams/b;
etaProps = [etaPropCenters, etaPropObdEdges, etaPropIbdEdges];



% normalized spanwise coordinate
% at root = 0, at tip = 1

% Collect the "significant" eta values where wing geometry characteristics
% are defined, control surfaces start or finish, propellers are located,
% etc. eta = 0 and eta = 1 should always be included
eta_sig = [LS.ExposedEtas,LS.TaperDefn(1,:), LS.SweepDefn(1,:), LS.Dihedral(1,:), LS.Twist(1,:), LS.SpecEtas, etaControls, etaProps, BlockedEtas, LS.AirfoilEta, 0, 1];


% sort this list in ascending order, and retain only unique values
eta_sig = unique(sort(eta_sig, 'ascend'));





% Isolate the exposed eta locations
eta_exp_indices = find(eta_sig>=eta_ibd&eta_sig<=eta_obd);

% calculate the spanwise runs between existing eta stations
db = eta_sig * b;
ddb = diff(db);

% ns = LS.StripDist;
%
% if isempty(ns)
% find the number of strips in just the exposed intervals based on the
% maximum strip width
ns = 1+ceil(ddb(eta_exp_indices(1:end-1))/LS.StripMaxWidth);
%     LS.StripDist = ns;
% end


eta_exp = eta_sig(eta_exp_indices);

eta_coll = [];

for i= 2:length(eta_exp)
    eta_add = linspace(eta_exp(i-1),eta_exp(i),ns(i-1));
    eta_coll = [eta_coll,eta_add];
end

eta = unique(sort([eta_sig,eta_coll],'ascend'))';

eta(eta>1) = [];
eta(eta<0) = [];

LS.eta = eta;
LS.nStrips = length(eta)-1;


if LS.GenUniformSpacing == 1

    n = ceil(b/LS.StripMaxWidth);
    eta = linspace(0,1,n); eta = eta';

end


% update np
np = length(eta);

% recalculate spanwise run
db = eta * b;



% interpolated normalized chords
c_norm = interp1(LS.TaperDefn(1,:),LS.TaperDefn(2,:),eta,'linear');

% average normalized chords (average spanwise successive ones) and the
% spanwise normalized distance between them
c_av = 0.5 * (c_norm(1:end-1) + c_norm(2:end));
d_eta = diff(eta);

% Use this to compute the normalized planform area
S_norm = sum(c_av.*d_eta);




% Compute chord scaling factor
CSF = LS.PlanformArea / (sum(diff(db).*c_av));

% Find dimensional chords
c_dim = c_norm * CSF;
dA = 0.5 * (c_dim(1:end-1) + c_dim(2:end)).*diff(db);

% Planform area check
S_check = sum(diff(db).*(0.5*(c_dim(1:end-1)+c_dim(2:end))));












% From sweep definition, determine the chord line used to define sweep for
% each spanwise location
SweepLoc = zeros(1,np);
for i = 1:np
    SweepDefnIndices = LS.SweepDefn(1,:)>=eta(i);
    OutboardLocs = LS.SweepDefn(:,SweepDefnIndices);
    if isempty(OutboardLocs)
        OutboardLocs = [0;0;0];
    end
    SweepLoc(i) =  OutboardLocs(2,1);
end


SweepVal = interp1(LS.SweepDefn(1,:),LS.SweepDefn(3,:),eta,'linear');

%A = [eta',SweepLoc',SweepVal']


% calculate the dihedral distribution
% root dihedral + relative dihedral
interp_di =  interp1(LS.Dihedral(1,:),LS.Dihedral(2,:),eta,'linear');
di = - LS.Directionality * (LS.RootDihedral + interp_di);


% calculate the relative twist distribution
tw = interp1(LS.Twist(1,:),LS.Twist(2,:),eta,'linear');



% Calculate components of rotation tensor from body-fixed to strip frame
ri = LS.RootIncidence;
[R_BS,R_CS,PTS] =  CalcR_BS(di,tw,ri,LS.Directionality);

% capture R_BS components
R11_BS = R_BS(:,1);
R12_BS = R_BS(:,2);
R13_BS = R_BS(:,3);
R21_BS = R_BS(:,4);
R22_BS = R_BS(:,5);
R23_BS = R_BS(:,6);
R31_BS = R_BS(:,7);
R32_BS = R_BS(:,8);
R33_BS = R_BS(:,9);

% capture R_CS components
R11_CS = R_CS(:,1);
R12_CS = R_CS(:,2);
R13_CS = R_CS(:,3);
R21_CS = R_CS(:,4);
R22_CS = R_CS(:,5);
R23_CS = R_CS(:,6);
R31_CS = R_CS(:,7);
R32_CS = R_CS(:,8);
R33_CS = R_CS(:,9);

% capture euler angles
ph = PTS(:,1);
th = PTS(:,2);
ps = PTS(:,3);



% Lifting surface reference point location
x0 = LS.RefPtLocation(1);
y0 = LS.RefPtLocation(2);
z0 = LS.RefPtLocation(3);

% array size allocations
xwQC = zeros(np,1);
ywQC = zeros(np,1);
zwQC = zeros(np,1);

xQC = zeros(np,1);
yQC = zeros(np,1);
zQC = zeros(np,1);

xLE = zeros(np,1);
yLE = zeros(np,1);
zLE = zeros(np,1);

xTE = zeros(np,1);
yTE = zeros(np,1);
zTE = zeros(np,1);

xcg = zeros(np,1);
ycg = zeros(np,1);
zcg = zeros(np,1);

CSA = zeros(np,1);

xwQC(1) = x0 + (LS.RefPtChordFrac -0.25) * c_dim(1);
ywQC(1) = y0;
zwQC(1) = z0;

[nafp,~] = size(AF);
[AFTOP,Top] = min(AF(:,2));
[AFBOT,Bot] = max(AF(:,2));

LS.Thickness_to_chord = abs(max((AFTOP-AFBOT)));
AFPts = [];

% thickness-to-chord default - no tc specified
if isempty(LS.t_min)
    LS.t_min = 0;
end
if isempty(LS.t_max)
    LS.t_max = 1000;
end

xwQC0 = zeros(np,1); ywQC0 = zeros(np,1); zwQC0 = zeros(np,1);

% intialize a variable called 'af' with blended y-coordinates
af = repmat(AF(:,2), 1, np);

% assign AF name to the stations, and initialize with primary airfoil
AFName = strings(np, 1);
AFName(:,1) = LS.AirfoilName{1}.name;

% number of airfoils
nAF = length(LS.AirfoilName);


% Start airfoil blend from root to tip, and update AF properties
if nAF > 1    % multiple airfoils

    AFName(:,1) = "";
    % allocate unblended airfoil sections first
    AF1 = AF;
    nUnblended = 1:2:2*nAF-1;

    for iAF = 1:length(nUnblended)
        
        % load secondary airfoil and assign to AF2
        [~, AF2] = InterpAF(AF1, LS, iAF);

        % fill out the airfoil for non-blended sections first
        [idx_AF,~] = find(eta >= LS.AirfoilEta(1,nUnblended(iAF)) & eta <= LS.AirfoilEta(1,nUnblended(iAF) + 1));
        af(:,idx_AF(1):idx_AF(end)) = repmat(AF2(:,2), 1, length(idx_AF(1):idx_AF(end)));

        AFName(idx_AF(1):idx_AF(end),1) = LS.AirfoilName{iAF}.name;

    end

    % do the blending now for the transition sections
    nBlended = nUnblended(1)+1:2:nUnblended(end)-1;

    for jAF = 1:length(nBlended)
        % fill out the airfoil for non-blended sections first
        [jdx_AF,~] = find(eta >= LS.AirfoilEta(1,nBlended(jAF)) & eta <= LS.AirfoilEta(1,nBlended(jAF) + 1));
        AF1 = [AF(:,1) af(:,jdx_AF(1)) AF(:,3)];
        AF2 = [AF(:,1) af(:,jdx_AF(end)) AF(:,3)];

        afBlend = zeros(height(AF1), length(jdx_AF));
        for iblnd = 1:height(AF1)
            % compute blended y-coordinates of the airfoil
            afBlend(iblnd,:) = linspace(AF1(iblnd,2), AF2(iblnd,2), length(jdx_AF));
        end

        % assign blended y-coordinates to the transition span section
        af(:,jdx_AF(1):jdx_AF(end)) = afBlend;

        AFName(jdx_AF(1):jdx_AF(end),1) = "Transition";
    end

end





for i = 1:np

    % % update airfoil based on the blended y-coordinates "af" as well
    AF = [AF(:,1) af(:,i) AF(:,3)];

    % calculate sectional properties
    % Polygon = PolygonMoments (xy,mn,PlotFlag)    
    Polygon = PolygonMoments ([AF(:,1),AF(:,2)],[2,2],0);
    % note axis transformations:
    % function +x --> body axes +x
    % function +y --> body axes -z
    % function +z --> body axes +y
    ACx = Polygon.ACx;          % for unit chord
    ACz = -Polygon.ACy;         % for unit chord

    % % assiging values based on the AF definition to compute inertias
    % % downstream
    kxxn(i,:) = Polygon.kxx;
    kyyn(i,:) = Polygon.kzz;
    kzzn(i,:) = Polygon.kyy;
    kxzn(i,:) = Polygon.kxy;


    % calculate the normalized area of the airfoil. Multiplying this by the
    % (length scaling factor)^2 will give dimensional area
    NCSA = polyarea(AF(:,1),AF(:,2)); % normalized cross-sectional area

    [nafp,~] = size(AF);
    [AFTOP,Top] = min(AF(:,2));
    [AFBOT,Bot] = max(AF(:,2));

    LS.Thickness_to_chord = abs(max((AFTOP-AFBOT)));

    % % AF based updates end here for each span stations

    if i>1
        % determine y and z locations based on dihedral distribution
        ywQC(i,1) = ywQC(i-1) + abs(db(i)-db(i-1)) * cosd(di(i)) * (LS.Directionality);
        zwQC(i,1) = zwQC(i-1) + abs(db(i)-db(i-1)) * sind(di(i)) * (LS.Directionality);

        % the x-coordinate of the inboard station at the same chord location
        % where sweep of the current station is defined
        xref = xwQC(i-1) - (SweepLoc(i) - 0.25)*c_dim(i-1);

        % x-coordinate of the current station at the chord location where
        % sweep of the current station is defined
        xsweepchord = xref - (db(i)-db(i-1)) * tand(SweepVal(i));

        % x-coordinate of quarter chord of current station
        xwQC(i,1) = xsweepchord + (SweepLoc(i) - 0.25)*c_dim(i);

        % determine y locations without accounting for dihedral       
        ywQC0(i,1) = ywQC0(i-1) + abs(db(i)-db(i-1)) * (LS.Directionality);

    end

    xwQC0(i,1) = xwQC(i,1);
    zwQC0(i,1) = zwQC0(i,1);

    % locate leading edge and trailing edge with straight chords (no
    % dihedral or twist)
    xwLE = xwQC(i,1) + 0.25 * c_dim(i,1); ywLE = ywQC(i,1); zwLE = zwQC(i,1);
    xwTE = xwQC(i,1) - 0.75 * c_dim(i,1); ywTE = ywQC(i,1); zwTE = zwQC(i,1);

    % account for LE and TE coordinates when dihedral and twist are
    % accounted for
    xLE1 = xwQC(i,1) + R11_CS(i) * (xwLE-xwQC(i,1)) + R12_CS(i) * (ywLE-ywQC(i,1)) + R13_CS(i) * (zwLE-zwQC(i,1));
    yLE1 = ywQC(i,1) + R21_CS(i) * (xwLE-xwQC(i,1)) + R22_CS(i) * (ywLE-ywQC(i,1)) + R23_CS(i) * (zwLE-zwQC(i,1));
    zLE1 = zwQC(i,1) + R31_CS(i) * (xwLE-xwQC(i,1)) + R32_CS(i) * (ywLE-ywQC(i,1)) + R33_CS(i) * (zwLE-zwQC(i,1));

    xTE1 = xwQC(i,1) + R11_CS(i) * (xwTE-xwQC(i,1)) + R12_CS(i) * (ywTE-ywQC(i,1)) + R13_CS(i) * (zwTE-zwQC(i,1));
    yTE1 = ywQC(i,1) + R21_CS(i) * (xwTE-xwQC(i,1)) + R22_CS(i) * (ywTE-ywQC(i,1)) + R23_CS(i) * (zwTE-zwQC(i,1));
    zTE1 = zwQC(i,1) + R31_CS(i) * (xwTE-xwQC(i,1)) + R32_CS(i) * (ywTE-ywQC(i,1)) + R33_CS(i) * (zwTE-zwQC(i,1));

    % locate "CG" of each section
    xwcg = xwLE - c_dim(i,1)*ACx;
    ywcg = ywLE;
    zwcg = zwLE + c_dim(i,1)*ACz;

    % account for dihedral and twist
    xcg1 = xwQC(i,1) + R11_CS(i) * (xwcg-xwQC(i,1)) + R12_CS(i) * (ywcg-ywQC(i,1)) + R13_CS(i) * (zwcg-zwQC(i,1));
    ycg1 = ywQC(i,1) + R21_CS(i) * (xwcg-xwQC(i,1)) + R22_CS(i) * (ywcg-ywQC(i,1)) + R23_CS(i) * (zwcg-zwQC(i,1));
    zcg1 = zwQC(i,1) + R31_CS(i) * (xwcg-xwQC(i,1)) + R32_CS(i) * (ywcg-ywQC(i,1)) + R33_CS(i) * (zwcg-zwQC(i,1));


    AFdim = AF * c_dim(i);

    thickness = max(AFdim(:,2)) - min(AFdim(:,2));


    SF_min = max(1,LS.t_min/thickness);     % enforce min thickness constraint
    
    
    SF_max = min(1,LS.t_max/thickness);     % enforce max thickness constraint


    if SF_min > 1
        LS.Thickness_to_chord = LS.t_min;
    end

    AFdim(:,2) = AFdim(:,2).*SF_min .* SF_max;

    % log t/c ratio and thickness
    h(i,1) = max(AFdim(:,2)) - min(AFdim(:,2));
    tc(i,1) = h(i,1)/c_dim(i);



    CSA(i,1) =  NCSA * c_dim(i)^2;

    xAF0 = -AFdim(:,1) + xwLE;
    yAF0 = ywQC(i);
    zAF0 = -AFdim(:,2) + zwLE;

    % rotate airfoil sections about c/4 through dihedral and twist
    xAF1 = xwQC(i,1) + R11_CS(i) * (xAF0-xwQC(i,1)) + R12_CS(i) * (yAF0-ywQC(i,1)) + R13_CS(i) * (zAF0-zwQC(i,1));
    yAF1 = ywQC(i,1) + R21_CS(i) * (xAF0-xwQC(i,1)) + R22_CS(i) * (yAF0-ywQC(i,1)) + R23_CS(i) * (zAF0-zwQC(i,1));
    zAF1 = zwQC(i,1) + R31_CS(i) * (xAF0-xwQC(i,1)) + R32_CS(i) * (yAF0-ywQC(i,1)) + R33_CS(i) * (zAF0-zwQC(i,1));

    % locate airfoil sections in body-fixed basis
    xAF = x0 + (xAF1-x0) * cosd(ri) + (zAF1-z0) * sind(ri);
    yAF =  yAF1;
    zAF = z0 -(xAF1-x0) * sind(ri) + (zAF1-z0) * cosd(ri);

    AFPts = [AFPts; xAF,yAF,zAF,i*ones(nafp,1), AF(:,1)];
    AF_T(i,1:4) = [xAF(Top) yAF(Top) zAF(Top) i ]; % Picking out the top and bottom airfoil points
    AF_B(i,1:4) = [xAF(Bot) yAF(Bot) zAF(Bot) i ];

    % locate quarter chord coordinates in body-fixed basis
    xQC(i,1) = x0 + (xwQC(i,1)-x0) * cosd(ri) + (zwQC(i,1)-z0) * sind(ri);
    yQC(i,1) =  ywQC(i,1);
    zQC(i,1) = z0 -(xwQC(i,1)-x0) * sind(ri) + (zwQC(i,1)-z0) * cosd(ri);

    % locate LE coordinates in body-fixed basis
    xLE(i,1) = x0 + (xLE1-x0) * cosd(ri) + (zLE1-z0) * sind(ri);
    yLE(i,1) =  yLE1;
    zLE(i,1) = z0 -(xLE1-x0) * sind(ri) + (zLE1-z0) * cosd(ri);

    % locate TE coordinates in body-fixed basis
    xTE(i,1) = x0 + (xTE1-x0) * cosd(ri) + (zTE1-z0) * sind(ri);
    yTE(i,1) =  yTE1;
    zTE(i,1) = z0 -(xTE1-x0) * sind(ri) + (zTE1-z0) * cosd(ri);

    % locate "CG" coordinates in body-fixed basis
    xcg(i,1) = x0 + (xcg1-x0) * cosd(ri) + (zcg1-z0) * sind(ri);
    ycg(i,1) = ycg1;
    zcg(i,1) = z0 -(xcg1-x0) * sind(ri) + (zcg1-z0) * cosd(ri);


    % temporary plotting
    %     plot3(xAF,yAF,zAF,'k')

    AFCoordinate{i,1}.Coordinate = AF;
end
%% Calculating the quarter chord-chord sweep of the wing geometry

% leading edge
xwLE0 = xwQC0 + 0.25 * c_dim; ywLE0 = ywQC0; zwLE0 = zwQC0;

% trailing edge
xwTE0 = xwQC0 - 0.75 * c_dim; ywTE0 = ywQC0; zwTE0 = zwQC0;

% mid-chord
xwMC0 = 0.5*(xwLE0 + xwTE0); ywMC0 = 0.5*(ywLE0 + ywTE0); zwMC0 = 0.5*(zwLE0 + zwTE0);






% wing reference point is at the quarter chord of the root.
% sweep back is taken to be positive
x1_x2 = xQC(1,1)-xQC(np,1);
sweep_QC = atand(x1_x2/b);
LS.Sweep_qc_r2t = sweep_QC;

% leading edge sweep
Sw0 = - LS.Directionality * atand(diff(xwLE0)./diff(ywLE0));

% quarter chord sweep
Sw25 = - LS.Directionality * atand(diff(xwQC0)./diff(ywQC0));

% midchord sweep
Sw50 = - LS.Directionality * atand(diff(xwMC0)./diff(ywMC0));

% trailing edge sweep
Sw100 = - LS.Directionality * atand(diff(xwTE0)./diff(ywTE0));

%% Calculating the exposed area of the geometry
Peri_norm = sqrt((AF(2:end,1) - AF(1:end-1,1)).^2 + (((AF(2:end,2) - AF(1:end-1,2)).^2))); % Calculating perimeter of normalized airfoil (sqrt(dX^2 + dY^2))
sumPeri_norm = sum(Peri_norm); % Summing the distances between each point

sumPeri_dim = sumPeri_norm.*c_dim; % scaling perimeter based on chord lenth w/ taper

Peri_exp = sumPeri_dim(eta >= LS.ExposedEtas(1)); % Exlcuding values not exposed
db_exp = db(eta >= LS.ExposedEtas(1)); % Exlcuding values not exposed

dA_Wet = .5*(Peri_exp(1:end-1) + Peri_exp(2:end)).*diff(db_exp); % Calculating Exposed Area at each section
%% Calculating Mean Aerodynamic Chord and Mean Aerodynamic Center
% Sectional Calculations


tr = c_dim(2:end)./c_dim(1:end-1); % Taper Ratio
tr_frac = ((1 + 2.*(tr))./(3 + 3.*tr));
ddb2 = diff(db); % The
S = (diff(db).*(0.5*(c_dim(1:end-1)+c_dim(2:end)))); % Sectional Area Calculation (Taken from LINE 239)

MAChord_i = (2/3).*c_dim(1:end-1).*((1+tr+tr.^2)./(1 + tr)); % Mean Aerodynamic Chord
MAC2 = sum(MAChord_i.*S)./sum(S); % Finding the TE MA-Center Z - coord

% y_av = abs(0.25.*(yLE(1:end-1) + yTE(1:end-1))); % This is the midpoint of each chord in the Y-direction
y_av = (eta(1:end-1) + 0.5*diff(eta))*LS.Span;

y_MAC_i = eta(1:end-1)*LS.Span + ddb2.*tr_frac; % MAC Y-COORD
y_MAC_eta_i = y_MAC_i/LS.Span;

y_MAC = (sum(y_MAC_i.*S)/sum(S)); % Finding the MA-Center Y - coord
eta_MAC = y_MAC/LS.Span;   % non-dimensional spanwise location of MAC

LS.eta_MAC = eta_MAC;

% z_MAC_LE_i = zLE(1:end-1) + (zLE(2:end) - zLE(1:end-1)).*((1 + 2.*(tr))./(3 + 3.*tr)); % LE MAC Z-COORD
% z_MAC_TE_i = zTE(1:end-1) + (zTE(2:end) - zTE(1:end-1)).*((1 + 2.*(tr))./(3 + 3.*tr)); % TE MAC Z-COORD
%
% z_MAC_LE = sum(z_MAC_LE_i.*S)./sum(S); % Finding the LE MA-Center Z - coord
% z_MAC_TE = sum(z_MAC_TE_i.*S)./sum(S); % Finding the TE MA-Center Z - coord

% z_MAC = mean([z_MAC_LE z_MAC_TE]); % Finding the MA-Center z - coord
% norm(z_MAC) - norm(x_MAC)


LE_MAC_x = interp1(eta,xLE,eta_MAC,'linear');
TE_MAC_x = interp1(eta,xTE,eta_MAC,'linear');

LE_MAC_y = interp1(eta,yLE,eta_MAC,'linear');
TE_MAC_y = interp1(eta,yTE,eta_MAC,'linear');

LE_MAC_z = interp1(eta,zLE,eta_MAC,'linear');
TE_MAC_z = interp1(eta,zTE,eta_MAC,'linear');

%% Calculating lam50 and lam25
Sw_av = 0.5*(SweepVal(2:end) + SweepVal(1:end-1));
dAR = ddb2.^2./S;


lam50_deg = atand(tand(Sw_av) - (2./dAR.*((1 - tr)./(1 + tr))));
lam25_deg = atand(tand(Sw_av) - (1./dAR.*((1 - tr)./(1 + tr))));

MAChord = (diff([LE_MAC_x TE_MAC_x]));

x_MAC = interp1(eta,xQC,eta_MAC,'linear');
y_MAC = interp1(eta,yQC,eta_MAC,'linear');
z_MAC = interp1(eta,zQC,eta_MAC,'linear');

if Settings.plotFLAG == 1
    plot3([LE_MAC_x TE_MAC_x],[LE_MAC_y TE_MAC_y],[LE_MAC_z TE_MAC_z],'-k','LineWidth',2) % Plotting the MA-Chord
else
end

LS.MAC = abs(MAChord);
LS.MAC2 = MAC2;
LS.LocMAC = [x_MAC y_MAC z_MAC];

% changes
LS.LE_MAC = [LE_MAC_x LE_MAC_y LE_MAC_z];

if Settings.plotFLAG == 1
    plot3(AFPts(:,1),AFPts(:,2),AFPts(:,3),'k')
    plot3(xLE,yLE,zLE,'r')
    plot3(xTE,yTE,zTE,'r')
    plot3(AF_T(:,1),AF_T(:,2),AF_T(:,3),'r')
    plot3(AF_B(:,1),AF_B(:,2),AF_B(:,3),'r')
else
end

xcg = 0.5*(xcg(1:end-1) + xcg(2:end));
ycg = 0.5*(ycg(1:end-1) + ycg(2:end));
zcg = 0.5*(zcg(1:end-1) + zcg(2:end));

xcg = [xcg(1);xcg];
ycg = [ycg(1);ycg];
zcg = [zcg(1);zcg];








% internal volume calculations
dV = [0;0.5*(CSA(1:end-1) + CSA(2:end)).*diff(db)];


% if eta_cg (spanwise location of CG is not given
if isempty(LS.eta_CG_spec)
    % WING MASS DISTRIBUTION
    % APPROACH: Distribute mass to get a target spanwise location of CG
    % Roskam, Part V, Ch 8, Table 8.1 (rpg 1562)
    % WING TYPE             CG CHORD               CG SPAN
    % wing, unswept         38-42% from LE         40%
    % wing, swept           see below **           35%
    % HT                    42% from LE            38%
    % VT (low tail)         42% from LE            38%
    % VT (T-tail)           42% from LE            55%
    % VT (cruciform)        42% from LE            38-55%, interp based on zh/b

    % ** 70% of the distance between the front and rear spar behind front spar

    % Since this is a generalized script (with no knowledge of whether the
    % lifting surface is a wing, HT, or VT), the following simplified
    % relationship is used for eta_cg
    % --> eta_cg = 0.38 + (0.55 - 0.38)*(kappa)
    % kappa in [0,1] represents the spanwise mounting location of another
    % lifting surface onto this one.
    % kappa = 0 (default). kappa = 1 means tip mounting.
    eta_CG = 0.38 + (0.5 - 0.38)*LS.EtaMounting;
    LS.eta_CG = eta_CG;
end

% if it is user-provided, use it
if ~isempty(LS.eta_CG_spec)
    eta_CG = LS.eta_CG_spec;
    LS.eta_CG = LS.eta_CG_spec;
end


% 1. assume a linear mass per unit length: mu = ky + c, where y is the
% spanwise coordinate; i.e., mu = k eta b + c
% 2. \int (mu).dy in [0,b] yields mass of wing, m
% ---> (b^2/2) k + (b) c = m
% 3. \int (mu)y.dy in [0,b] = m y_cg = m b eta_cg
% ---> (b^3/3) k + (b^2/2) c = m eta_cg b
% 4. Solve the system AX = B, where:
% A = [b^2/2, b;  b^3/3, b^2/2], B = [m; m*eta_cg*b], X = [k;c]
A = [b^2/2, b;  b^3/3, b^2/2];
B = [LS.Mass; LS.Mass*eta_CG*b];
X = A\B;
k = X(1); c = X(2);
% compute mass per unit length at spanwise stations
mu = k.*eta.*b + c;
% average betweeen successive stations
mu_av = 0.5*(mu(1:end-1) + mu(2:end));
% get strip masses as dm = mu_av * (spanwise run) = mu_av * diff(db)
dm = [0; mu_av.*diff(db)];


% CG location

% get the x, y, z coordinates (in body-fixed basis) of the LE and TE of the
% lifting surface at spanwise station eta_CG

CG_LE = interp1(eta, [xLE, yLE, zLE], LS.eta_CG, 'linear');
CG_TE = interp1(eta, [xTE, yTE, zTE], LS.eta_CG, 'linear');

% set CG y-coordinate as the average of the LE and TE coordinates
Ycg = 0.5*(CG_LE(2) + CG_TE(2));

% if chordwise location of CG is not specified
% calculate it based on individual strip masses and locations
if isempty(LS.chordfrac_CG_spec)
    Xcg = sum(dm.*xcg)/LS.Mass;
    % Ycg - as calculated above
    Zcg = sum(dm.*zcg)/LS.Mass;
    % compute CG chord fraction
    LS.chordfrac_CG = norm([Xcg,Ycg,Zcg]-CG_LE)/norm(CG_LE-CG_TE);
end

% if chordwise location of CG is specified, then use it
if ~isempty(LS.chordfrac_CG_spec)
    LS.chordfrac_CG = LS.chordfrac_CG_spec;
    CG_loc = CG_LE + LS.chordfrac_CG * (CG_TE - CG_LE);
    Xcg = CG_loc(1);
end

Zcg = sum(dm.*zcg)/LS.Mass;

LS.CG = [Xcg,Ycg,Zcg];
LS.eta_CG = eta_CG;





% moment of inertia calculation
% about CG of each strip, centroidal axes
% Ixx = m (kxx^2 + dy^2/12)
% Iyy = m (kyy^2)
% Izz = m (kzz^2 + dz^2/12)

% use average chord as scaling factor
c_av = [0;0.5*(c_dim(1:end-1) + c_dim(2:end))];
dy = [0;diff(db)];
kxx = c_av .* kxxn;
kyy = c_av .* kyyn;
kzz = c_av .* kzzn;
kxz = c_av .* kxzn;

Ixx0 = dm.*(kxx.^2 + dy.^2/12);
Iyy0 = dm.*(kyy.^2);
Izz0 = dm.*(kzz.^2 + dy.^2/12);

Ixy0 = 0 * Ixx0;        % are they zero?
Iyz0 = 0 * Ixx0;       % are they zero?
Ixz0 = dm.*(kxz.^2); % is this legit?


R = [R11_BS,R12_BS,R13_BS,R21_BS,R22_BS,R23_BS,R31_BS,R32_BS,R33_BS];
% Ixx,Iyy,Izz,Ixy,Iyz,Ixz
Ica = [Ixx0,Iyy0,Izz0,Ixy0,Iyz0,Ixz0];

CG = [xcg,ycg,zcg];

[Ixx,Iyy,Izz,Ixy,Iyz,Izx] = ApplyParallelAxisTheorem(dm,CG,Ica,R);

LS.Ixx = sum(Ixx);
LS.Iyy = sum(Iyy);
LS.Izz = sum(Izz);
LS.Ixy = sum(Ixy);
LS.Iyz = sum(Iyz);
LS.Izx = sum(Izx);



Stn = table();

Stn.i = (1:length(eta))';
Stn.eta = eta;
Stn.c = c_dim;
Stn.h = h;
Stn.tw = tw;
Stn.di = di;
Stn.tr = [0;tr];
Stn.tc = tc;
Stn.Sw = SweepVal;
Stn.Sw_0 = [Sw0(1); Sw0];
Stn.Sw_25 = [Sw25(1); Sw25];
Stn.Sw_50 = [Sw50(1);Sw50];
Stn.Sw_100 = [Sw100(1);Sw100];
Stn.db = db;
Stn.xQC  = xQC;
Stn.yQC = yQC;
Stn.zQC = zQC;
Stn.xLE = xLE;
Stn.yLE = yLE;
Stn.zLE = zLE;
Stn.xTE = xTE;
Stn.yTE = yTE;
Stn.zTE = zTE;
Stn.xcg = xcg;
Stn.ycg = ycg;
Stn.zcg = zcg;
Stn.Ixx = Ixx;
Stn.Iyy = Iyy;
Stn.Izz = Izz;
Stn.Ixy = Ixy;
Stn.Iyz = Iyz;
Stn.Izx = Izx;
Stn.dA = [0;dA];
Stn.sdA = cumsum(Stn.dA);
Stn.dA_Wet = [zeros(size(Stn.dA,1)-size(dA_Wet,1),1);dA_Wet];
Stn.sdA_Wet = cumsum(Stn.dA_Wet);
Stn.CSA = CSA;
Stn.dV = dV;
Stn.sdV = cumsum(Stn.dV);
Stn.dm = dm;
Stn.ph = ph;
Stn.th = th;
Stn.ps = ps;

Stn.R11 = R11_BS;
Stn.R12 = R12_BS;
Stn.R13 = R13_BS;
Stn.R21 = R21_BS;
Stn.R22 = R22_BS;
Stn.R23 = R23_BS;
Stn.R31 = R31_BS;
Stn.R32 = R32_BS;
Stn.R33 = R33_BS;

Stn.Airfoil = AFName;
Stn.AFCoordinate = AFCoordinate;




LS.AFPts = AFPts;



%Stn
% calculate projected span and projected aspect ratio
LS.ProjectedSpan = 0;
LS.ProjectedAspectRatio = LS.ProjectedSpan^2/LS.PlanformArea;


LS.Stn = Stn;







%% Create strip definitions

% capture only the "exposed" strips from the "Stn" table
StripTbl = Stn(Stn.eta>=LS.ExposedEtas(1) & Stn.eta<=LS.ExposedEtas(2),:);


% find etas of strip midpoints
eta_mid = 0.5*(StripTbl.eta(1:end-1) + StripTbl.eta(2:end));
nstrips = length(eta_mid);

StripDef.Name = repmat(LS.Name,[nstrips,1]);

StripDef.eta = eta_mid;

% quarter-chord spanwise coordinates
StripDef.db_ibd = StripTbl.db(1:end-1) - StripTbl.db(1);
StripDef.db_mid = (StripTbl.db(1:end-1) + StripTbl.db(2:end))/2 - StripTbl.db(1);
StripDef.db_obd = StripTbl.db(2:end) - StripTbl.db(1);




% Length - spanwise dimension of each strip (find by differencing)
StripDef.Length = StripDef.db_obd - StripDef.db_ibd;
% Chord - mean chord (find by averaging strip edges)
StripDef.Chord = 0.5 * (StripTbl.c(1:end-1) + StripTbl.c(2:end));
% Area - use trapezoidal formula
StripDef.Area = StripDef.Chord.*StripDef.Length;

% Send exposed parameters to lifting surface
LS.ExposedPlanformArea = sum(StripDef.Area);
LS.ExposedSpan = sum(StripDef.Length);
StripDef.eta_exp = StripDef.db_mid/LS.ExposedSpan;

% % LE x coordinate - average between edges
% StripDef.LE_x = 0.5 * (StripTbl.xLE(1:end-1) + StripTbl.xLE(2:end));
% % LE y coordinate - average between edges
% StripDef.LE_y = 0.5 * (StripTbl.yLE(1:end-1) + StripTbl.yLE(2:end));
% % LE z coordinate - average between edges
% StripDef.LE_z = 0.5 * (StripTbl.zLE(1:end-1) + StripTbl.zLE(2:end));
%
% % TE x coordinate - average between edges
% StripDef.TE_x = 0.5 * (StripTbl.xTE(1:end-1) + StripTbl.xTE(2:end));
% % TE y coordinate - average between edges
% StripDef.TE_y = 0.5 * (StripTbl.yTE(1:end-1) + StripTbl.yTE(2:end));
% % TE z coordinate - average between edges
% StripDef.TE_z = 0.5 * (StripTbl.zTE(1:end-1) + StripTbl.zTE(2:end));

% quarter chord x coordinate - average between edges
StripDef.xAC = 0.5 * (StripTbl.xQC(1:end-1) + StripTbl.xQC(2:end));
% quarter chord y coordinate - average between edges
StripDef.yAC = 0.5 * (StripTbl.yQC(1:end-1) + StripTbl.yQC(2:end));
% quarter chord z coordinate - average between edges
StripDef.zAC = 0.5 * (StripTbl.zQC(1:end-1) + StripTbl.zQC(2:end));

% Chord - mean TC (find by averaging strip edges)
StripDef.TC = 0.5 * (StripTbl.tc(1:end-1) + StripTbl.tc(2:end));


di_av = interp1(eta,di,eta_mid,'linear');
tw_av = interp1(eta,tw,eta_mid,'linear');


[R_BS2,~,PTS2] = CalcR_BS(di_av,tw_av,ri,LS.Directionality);

% Strip rotation tensor elements
% StripDef.R11 = R_BS2(:,1);
% StripDef.R12 = R_BS2(:,2);
% StripDef.R13 = R_BS2(:,3);
% StripDef.R21 = R_BS2(:,4);
% StripDef.R22 = R_BS2(:,5);
% StripDef.R23 = R_BS2(:,6);
% StripDef.R31 = R_BS2(:,7);
% StripDef.R32 = R_BS2(:,8);
% StripDef.R33 = R_BS2(:,9);
%

StripDef.R_BS = R_BS2;

StripDef.R_SB = TransposeVectorizedDCM(StripDef.R_BS);

% strip euler angles
StripDef.ph = PTS2(:,1);
StripDef.th = PTS2(:,2);
StripDef.dth = StripDef.th - StripDef.th(1);
StripDef.ps = PTS2(:,3);

% CS: controls (initialize)
% the number in this column denotes how many controls are associated with
% that strip
StripDef.CS = zeros(nstrips,1);

% PROP: initialization
% the number in this column denotes the number of props associated with
% this strip
StripDef.PROP = zeros(nstrips,1);


% BLOCKAGE: initialization
StripDef.BLOCK = zeros(nstrips,1);

%% Process blockages
if nblock > 0

    for i = 1:nblock

        blocked_strips = find(StripDef.eta>=LS.BlockedEtas(i,1) & StripDef.eta<=LS.BlockedEtas(i,2));
        StripDef.BLOCK(blocked_strips) = StripDef.BLOCK(blocked_strips) + 1;

    end


end



%% Process control surfaces
if LS.GenUniformSpacing~=1

    if numCS > 0

    

        for i = 1:numCS

            HL_coll = [];       % hinge line collection
            pts_list = [];
            cspts_list = [];



            % find the inboard edge of the control surface
            IBD = Stn(Stn.eta == LS.Controls(i).eta(1),:);
            %             IBD_AF = AFPts(AFPts(:,4)==IBD.i,:);

            % find the outboard edge of the control surface
            OBD = Stn(Stn.eta == LS.Controls(i).eta(2),:);
            %             OBD_AF = AFPts(AFPts(:,4)==OBD.i,:);

            if isfield(LS.Controls(i),'GapPerc')
                if isempty(LS.Controls(i).GapPerc)
                    gap_perc = 0.00;
                else
                    gap_perc = LS.Controls(i).GapPerc;
                end
            else
                gap_perc = 0;
            end

            % control surface chord fraction
            CF(1) = LS.Controls(i).ChordFrac(1) + gap_perc * (LS.Controls(i).ChordFrac(2) - LS.Controls(i).ChordFrac(1));
            CF(2) = LS.Controls(i).ChordFrac(2);


            AFInd = find(AF(:,1)>=CF(1) & AF(:,1)<=CF(2));

            AFInd_top = AFInd(AFInd<=nafp/2);
            AFInd_bot = AFInd(AFInd>nafp/2);

            CSStns = [IBD.i:1:OBD.i];

            for j = 1:length(CSStns)

                af_slice = AFPts(AFPts(:,4)==CSStns(j),:);

                af_findinds = find(af_slice(:,5)==0);

                % top of the airfoil surface
                af_topsurf = flipud(af_slice(1:af_findinds,:));

                % bottom of the airfoil surface
                af_botsurf = af_slice(af_findinds+1:end,:);



                % control leading edge on top surface
                topsurf_LE = interp1(af_topsurf(:,5), af_topsurf(:,1:3),CF(1),'linear');

                % control leading edge on bottom surface
                botsurf_LE = interp1(af_botsurf(:,5), af_botsurf(:,1:3),CF(1),'linear');

                % track the hinge line
                HL_coll = [HL_coll; 0.5 * (topsurf_LE + botsurf_LE)];

                CS_thickness = norm(topsurf_LE - botsurf_LE);

                th = (0:6:180)';

                d_chord = (CS_thickness/2) * sind(th);
                d_thickness = - (CS_thickness/2) * cosd(th);

                stn_dihedral = Stn.di(CSStns(j));

                CS_nose = repmat(0.5*(topsurf_LE + botsurf_LE), length(th), 1);
                CS_nose(:,1) = CS_nose(:,1) + d_chord;
                CS_nose(:,2) = CS_nose(:,2) +  LS.Directionality * d_thickness * sind(abs(stn_dihedral));
                CS_nose(:,3) = CS_nose(:,3) +  d_thickness * cosd(abs(stn_dihedral));



                cspts = [...
                    af_slice(AFInd_top,1),af_slice(AFInd_top,2), af_slice(AFInd_top,3);
                    topsurf_LE;
                    CS_nose;
                    botsurf_LE;
                    af_slice(AFInd_bot,1),af_slice(AFInd_bot,2), af_slice(AFInd_bot,3)];

                cspts(:,4) = CSStns(j);
                if Settings.plotFLAG == 1
                    plot3(cspts(:,1),cspts(:,2), cspts(:,3),'b','linewidth',2)
                    patch(cspts(:,1),cspts(:,2), cspts(:,3),'b')

                else
                end
                cspts_list = [cspts_list; cspts];
            end

            % Save hinge line
            LS.Controls(i).HingeLine = [HL_coll(1,:); HL_coll(end,:)];

            % LS.Controls(i).Points = pts_list;
            [LS.Controls(i).Vertices,LS.Controls(i).Surfaces] = prepac3d(cspts_list);

            % find which strips are associated with this control surface
            cs_strips = find(StripDef.eta>=LS.Controls(i).eta(1) & StripDef.eta<=LS.Controls(i).eta(2));

            % log the strip indices under this control
            LS.Controls(i).StripIndices = cs_strips';

            % find and log which station indices are associated with this control
            % surface
            cs_stnindices = find(Stn.eta>=LS.Controls(i).EtaFrac(1) & Stn.eta<=LS.Controls(i).EtaFrac(2));
            LS.Controls(i).StnIndices = cs_stnindices';

            % and hike the "CS" counter in the strip table by 1
            StripDef.CS(cs_strips) = StripDef.CS(cs_strips) + 1;

            LS.(LS.Controls(i).Name) = LS.Controls(i);
            % Calculate control surface area
            CS_db = (diff(db(cs_strips))); % Summing span of control surface strips
            CS_c_dim = (c_dim(cs_strips)); % Summing chords lengths of control Surface Strips
            CF_dif = diff(CF); % Finding difference between init and ending chord fraction
            eta_dif = diff(LS.Controls(i).EtaFrac);

            %             CS_Area(i) = CS_db*CS_c_dim*CF_dif; % Calculating Control Surface Area
            CS_Area(i) = sum(CS_db.*(0.5*(CS_c_dim(1:end-1)+CS_c_dim(2:end))))*CF_dif;
            LS.Controls(i).PlanformArea = CS_Area(i);

            FAR(i) = sum(CS_db.*(0.5*(CS_c_dim(1:end-1)+CS_c_dim(2:end))));

            %% Calculate control surface mean chord
            CS_c_av(i) = (1/CS_Area(i))*sum(CS_db.*(0.5*CF_dif*(CS_c_dim(1:end-1)+CS_c_dim(2:end))).^2);


        end
        LS.CS_Area = [CS_Area]; % Saving Area Values to Structure
        LS.CS_c_av = [CS_c_av]; % Saving Control Surface Mean Chord Values to Structure
        LS.FAR = [FAR];
    end
end






%% Process propellers
if LS.GenUniformSpacing~=1

    if numProps > 0

        for i = 1:numProps

            % identify strip indices that are associated with this prop
            prop_strips = find(StripDef.eta>=etaPropIbdEdges(i) & StripDef.eta<=etaPropObdEdges(i));

            % log the information for this propeller under "LS" struct
            PropName = strcat('Prop',num2str(i));

            LS.(PropName).eta = etaPropCenters(i);
            LS.(PropName).etaIbd = etaPropIbdEdges(i);
            LS.(PropName).etaObd = etaPropObdEdges(i);
            LS.(PropName).Diameter = LS.PropDiams(i);
            LS.(PropName).StripIndices = prop_strips';

            % and hike the "PROP" counter in the strip table by 1
            StripDef.PROP(prop_strips) = StripDef.PROP(prop_strips) + 1;
        end


    end
end


LS.StripDef = struct2table(StripDef);

LS.StripDef = movevars(LS.StripDef, 'eta_exp', 'After','eta');

if Settings.FlightStream ~= 1
    % % script to account for control surfaces
    [AFPts] = ModForControlSurfaces(LS);
end



[LS.Vertices,LS.Surfaces] = prepac3d(AFPts);




%% modify main geometry vertices to account for control surfaces
    function [out] = ModForControlSurfaces(LS)

        cseta_coll = [];
        for m = 1:LS.numControls
            cseta_coll = [cseta_coll, LS.Controls(m).EtaFrac];
        end

        afpts = LS.AFPts;
        modafpts = afpts;
        % note: Mar 25, 2022: written for T/E control surfaces

        % loop over each control
        for ii = 1:LS.numControls

            Cont = LS.Controls(ii);

            % find the chord fraction at which this control begins
            startCF = Cont.ChordFrac(1);

            % find the station indices over which this control extends
            CSStnIndices = Cont.StnIndices;

            % are the inboard and outboard edges shared with any other
            % control surfaces?
            IbdEdgeShared = sum(cseta_coll==Cont.EtaFrac(1))>1;
            ObdEdgeShared = sum(cseta_coll==Cont.EtaFrac(2))>1;

            IbdAtRoot = Cont.EtaFrac(1) == 0;
            ObdAtTip = Cont.EtaFrac(2) == 1;

            IbdHold = [];
            ObdHold = [];

            % catch the main surface airfoil points at the inboard and
            % outboard edge of this control surface

            if ~IbdEdgeShared && ~IbdAtRoot
                IbdHold = afpts(afpts(:,4)==CSStnIndices(1),:);
                IbdHold(:,4) = IbdHold(:,4) - 0.5;
            end

            if ~ObdEdgeShared && ~ObdAtTip
                ObdHold = afpts(afpts(:,4)==CSStnIndices(end),:);
                ObdHold(:,4) = ObdHold(:,4) + 0.5;
            end

            modafpts = [modafpts; IbdHold;  ObdHold];

            for jj = 1:length(CSStnIndices)

                slice = afpts(afpts(:,4)==CSStnIndices(jj),:);

                findinds = find(slice(:,5)==0);

                % top of the airfoil surface
                topsurf = flipud(slice(1:findinds,:));

                % interpolate to find top surface point at start of this
                % control surface
                topsurf_TE = interp1(topsurf(:,5), topsurf(:,1:3),startCF,'linear');

                % bottom of the airfoil surface
                botsurf = slice(findinds+1:end,:);

                % interpolate to find bottom surface point at start of this
                % control surface
                botsurf_TE = interp1(botsurf(:,5), botsurf(:,1:3),startCF,'linear');

                TE_thickness = norm(topsurf_TE - botsurf_TE);

                ClosingPt_TE = (topsurf_TE + botsurf_TE)/2;
                ClosingPt_TE(1) = ClosingPt_TE(1) + TE_thickness/2;

                % set coordinates of top surface points aft of the start of
                % this control surface to the topsurf_TE coordinates
                topsurf_inds = find(topsurf(:,5)>=startCF);

                topsurf(topsurf_inds(1),1:3) = topsurf_TE;
                %topsurf(topsurf_inds(2:end),1:3) = repmat(ClosingPt_TE,length(topsurf_inds)-1,1);
                topsurf(topsurf_inds(2:end),1) = repmat(ClosingPt_TE(1),length(topsurf_inds)-1,1);

                % set coordinates of bottom surface points aft of the start of
                % this control surface to the botsurf_TE coordinates
                botsurf_inds = find(botsurf(:,5)>=startCF);

                botsurf(botsurf_inds(1),1:3) = botsurf_TE;
                %                botsurf(botsurf_inds(2:end),1:3) = repmat(ClosingPt_TE,length(botsurf_inds)-1,1);
                botsurf(botsurf_inds(2:end),1) = repmat(ClosingPt_TE(1),length(botsurf_inds)-1,1);

                % reconstruct the slice
                slicemod = [flipud(topsurf);botsurf];

                % commit to afpts
                modafpts(modafpts(:,4)==CSStnIndices(jj),:) = slicemod;

            end

        end


        % sort the wing vertices in increasing order of spanwise index
        modafpts = sortrows(modafpts,4);

        % new number of slices
        nslices = height(modafpts)/nafp;

        newindices = reshape(repmat([1:nslices],[nafp,1]),[nafp*nslices,1]);

        % assign new spanwise indices
        modafpts(:,4) = newindices;

        out = modafpts;


    end




%% Create data for AC3D export
    function [Vertices,Surfaces] = prepac3d(AFPts)
        % create running variable i (running along stations) and j (running along
        % airfoils from TE to LE).

        npp = AFPts(end,4) - AFPts(1,4) + 1;    % number of spanwise slices
        indfirstslice = AFPts(1,4);
        AFSlice = AFPts(AFPts(:,4)==indfirstslice,:);         % pick first slice

        % measure number of points on first slice
        [nafpt,~] = size(AFSlice);
        ii = repmat((1:npp-1),[nafpt-1,1]); ii = reshape(ii,[],1);
        jj = repmat((1:nafpt-1)',[npp-1,1]);
  

        % Create index numbers for the quad surfaces
        P1 = jj +     (ii-1)*nafpt;
        P2 = jj + 1 + (ii-1)*nafpt;
        P3 = jj + 1 + (ii)*nafpt;
        P4 = jj +     (ii)*nafpt;

        % Create closing surfaces at the root and tip airfoil locations
%         P1c = (1:1:nafpt)';
%         P2c = (2:1:nafpt+1)';
%         P4c = (nafpt:-1:1)';
%         P3c = (nafpt-1:-1:0)';

nlim = ceil(nafpt/2);

        P1c = (1:1:nlim)';
        P2c = (2:1:nlim+1)';
        P4c = (nlim:-1:1)';
        P3c = (nlim-1:-1:0)';

        AA = [P1c,P2c,P3c,P4c];

        SelfIntersection = find(AA(:,2)>=AA(:,3));
        ind_SelfIntersection = SelfIntersection(1);

        % A = [P4c,P3c,P2c,P1c];
        % Indices of root closing surface

        RootClose1 = AA(1:ind_SelfIntersection,:);

        

        P1c = (nafpt:-1:nlim)';
        P2c = (nlim:1:nafpt)';
        P4c = (nafpt-1:-1:nlim-1)';
        P3c = (nlim+1:1:nafpt+1)';

        BB = [P1c,P2c,P3c,P4c];

        SelfIntersection = find(BB(:,1)<=BB(:,2));
        ind_SelfIntersection = SelfIntersection(1);

        RootClose2 = BB(1:ind_SelfIntersection,:);

        RootClose = [RootClose1;RootClose2];

%         RootClose = AA(1:ceil(nafpt/2)-1,:);

        if mod(nafpt,2)
            RootClose(end,3) = RootClose(end,3) + 1;
        end

        % Indices of tip closing surface
        TipClose = nafpt*(npp-1) + RootClose;

        % Gather vertices of the lifting surface
        Vertices = AFPts(:,1:3);

        % Based on the directionality of the lifting surface, arrange the indices
        % so as to avoid getting inverted normals

        % if lifting surface extends "to the right"
        if LS.Directionality == 1
            %     LS.Surfaces = [P4,P3,P2,P1; RootClose; TipClose];
            Surfaces = [P1,P2,P3,P4; RootClose; TipClose];
        end

        % if lifting surface extends "to the left"
        if LS.Directionality == -1
            %     LS.Surfaces = [P1,P2,P3,P4; RootClose; TipClose];
            Surfaces = [P4,P3,P2,P1; RootClose; TipClose];
        end

          stoppffer = 1;


    end


%% embedded functions
    function [R_BS,R_CS,PTS] = CalcR_BS(di,tw,ri,directionality)
        % [B] -- aircraft body fixed basis
        % [C] -- component reference basis
        % [S] -- strip basis
        % Rotation sequence:
        % [B] -- [root incidence] -- [C] -- [dihedral] -- [twist] -- [S]

        % [C] -- R_CS -- [S]
        % R_CS = R1(di) * R2(tw)

        r11_CS = cosd(tw);
        r12_CS = 0*r11_CS;
        r13_CS = sind(tw);
        r21_CS = sind(di).*sind(tw);
        r22_CS = cosd(di);
        r23_CS = -sind(di).*cosd(tw);
        r31_CS = -cosd(di).*sind(tw);
        r32_CS = sind(di);
        r33_CS = cosd(di).*cosd(tw);

        R_CS = [r11_CS, r12_CS, r13_CS, r21_CS, r22_CS, r23_CS, r31_CS, r32_CS, r33_CS];

        % unit vector 1 transformation
        r11_BS = r11_CS * cosd(ri) + r31_CS * sind(ri);
        r21_BS = r21_CS;
        r31_BS = -r11_CS * sind(ri) + r31_CS * cosd(ri);
        % unit vector 2 transformation
        r12_BS = r12_CS * cosd(ri) + r32_CS * sind(ri);
        r22_BS = r22_CS;
        r32_BS = -r12_CS * sind(ri) + r32_CS * cosd(ri);
        % unit vector 3 transformation
        r13_BS = r13_CS * cosd(ri) + r33_CS * sind(ri);
        r23_BS = r23_CS;
        r33_BS = -r13_CS * sind(ri) + r33_CS * cosd(ri);

        R_BS = [r11_BS, r12_BS, r13_BS, r21_BS, r22_BS, r23_BS, r31_BS, r32_BS, r33_BS];

        % backing out euler angles
        % 3-2-1 transformation
        % R31 = -sin(th)
        theta = asind(-r31_BS);
        % R33 = cos(ph) * cos(th)
        phi = acosd(r33_BS./cosd(theta))*(-directionality);
        % R21 = cos(th) * sin(ps)
        psi = asind(r21_BS./cosd(theta));

        PTS = [phi,theta,psi];

    end


    function [R_out] = TransposeVectorizedDCM(R_in)

        % this is the order of elements
        % [r11, r12, r13, r21, r22, r23, r31, r32, r33];

        % 1. out element (1,1) - grab in element (1,1)
        % grab in matrix col 1

        % 2. out element (1,2) - grab in element (2,1)
        % grab in matrix col 4

        % 3. out element (1,3) - grab in element (3,1)
        % grab in matrix col 7

        % 4. out element (2,1) - grab in element (1,2)
        % grab in matrix col 2

        % 5. out element (2,2) - grab in element (2,2)
        % grab in matrix col 5

        % 6. out element (2,3) - grab in element (3,2)
        % grab in matrix col 8

        % 7. out element (3,1) - grab in element (1,3)
        % grab in matrix col 3

        % 8. out element (3,2) - grab in element (2,3)
        % grab in matrix col 6

        % 9. out element (3,3) - grab in element (3,3)
        % grab in matrix col 9

        CaptureOrder = [1, 4, 7, 2, 5, 8, 3, 6, 9];

        R_out = R_in(:, CaptureOrder);

    end


%%
stopper = 1;

end