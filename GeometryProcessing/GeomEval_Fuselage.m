function [Fus] =  GeomEval_Fuselage(Fus)
global Settings
% count number of fuselage cross sections defined
[~,ncs] = size(Fus.CrossSections);
% Fus.AziPtsPerSide = 32;
FS_significant = [];
CS_indref = [];


Lseg = 0;

fs_coll = [];


% loop over each defined cross section
for i = 1:ncs
    % load up the cross-section definitions
    CSDef(i) = load(Fus.CrossSections(i).Defn);
    
    % log the FS fractions for each cross section
    FS_significant = [Fus.CrossSections(i).FS];
    
    % add length of segment
    Lseg = Lseg + Fus.CrossSections(i).Length;
    
    % number of intermediate cross-sections
    ns(i) = max(5,ceil(Fus.CrossSections(i).Length/Fus.MaxDistBetweenStations));
    
    % collect fuselage stations at which to perform calculations
    new_fs = linspace(FS_significant(1),FS_significant(2),ns(i));
    fs_coll = [fs_coll, new_fs];
    
    % log the index of the cross section
    CS_indref = [CS_indref, i*ones(1,length(new_fs))];
    
    
end


Fus.Length = Lseg;



% find the number of intermediate fuselage stations needed to satisfy the
% max distance between stations constraint
% ns = 1 + ceil(diff(Fus.Length * FS_significant)/Fus.MaxDistBetweenStations);


% add additional fuselage stations as required
% for i= 2:length(FS_significant)
%     % provided the cross section shape is not changing...
%     if(CS_indref(i)==CS_indref(i-1))
%         fs_add = linspace(FS_significant(i-1),FS_significant(i),ns(i-1));
%         fs_coll = [fs_coll,fs_add];
%     end
% end

% create final number of fuselage stations for downstream calculations
fs = (unique(fs_coll))';
nfs = length(fs);


CS_ind = CS_indref';

% load fuselage line definitions
FUS = [];
load(Fus.LineDef);
% this puts a variable called FUS into the function workspace

% column 1: normalized x
% column 2: coordinates of "top" line (side view)
% column 3: coordinates of "bottom" line (side view)
% column 4: coordinates of "side" (top view)
xRef = FUS(:,1);
TopRef = FUS(:,2);
BottomRef = FUS(:,3);
SideRef = FUS(:,4);

if~isempty(Fus.FinenessRatio)
    
    ScaleFactor = (1/Fus.FinenessRatio)/(max(TopRef-BottomRef));
    TopRef = TopRef * ScaleFactor;
    BottomRef = BottomRef * ScaleFactor;
    SideRef = SideRef * ScaleFactor;
    
    HeightToWidthRatio = (TopRef - BottomRef)./(2*SideRef);
    
    MaxHeightToWidth = max(HeightToWidthRatio);
    
    if MaxHeightToWidth > 1
        Fus.MaxHeight = Fus.Length/Fus.FinenessRatio;
        Fus.MaxWidth = Fus.MaxHeight / MaxHeightToWidth;
    end
    
    if MaxHeightToWidth <=1
        Fus.MaxWidth = Fus.Length/Fus.FinenessRatio;
        Fus.MaxHeight = Fus.MaxWidth * MaxHeightToWidth;
    end
    
    
end

Lrunning = 0;
fs_last = 0;
x_m = [];

for i = 1:ncs
    
    CSLength = Fus.CrossSections(i).Length;
    
    CS_fs = fs(CS_ind == i);
    
    xtemp = (CS_fs - fs_last) * CSLength ./ ((CS_fs(end)-fs_last)) + Lrunning;
    
    x_m = [x_m; xtemp];
    
    Lrunning = x_m(end);
    
    fs_last = CS_fs(end);
    
end


% Correct for the fact that the FD x-axis is positive forward
% x = -((fs - Fus.RefPt_FS) * Fus.Length); % + Fus.RefPtLocation(1);

x = - x_m + Fus.RefPt_FS * Fus.Length; % + Fus.RefPtLocation(1);

% Correct for the fact that the FD z-axis is positive down
ZScaleFactor = (Fus.MaxHeight) / max(TopRef - BottomRef);
z_top = -(interp1(xRef,TopRef,fs,'linear','extrap') * ZScaleFactor)';
% Correct for the fact that the FD z-axis is positive down
z_bot = -(interp1(xRef,BottomRef,fs,'linear','extrap')  * ZScaleFactor)';
% Establish the centerline passing through centers of all cross-sections
z_ctr = 0.5*(z_top + z_bot);

% Find the maximum semi-width of all fuselage cross-sections
YScaleFactor = (Fus.MaxWidth/2) / max(SideRef);
y_side = (interp1(xRef,SideRef,fs,'linear','extrap')  * YScaleFactor)';

% Find the width of each cross-section
w = 2 * y_side;
% Find the height of each cross section
h = (z_bot - z_top);

ColorVec = Fus.ColorVec;

% Check if a user has specified specified thetas
% if true, use specified thetas
% else use linspace


th_000 = 1;
th_090 = floor(Fus.AziPtsPerSide/2);
th_180 = Fus.AziPtsPerSide;
th_270 = Fus.AziPtsPerSide + floor(Fus.AziPtsPerSide/2);

CSPts = [];

Area = zeros(nfs,1);
Xc = zeros(nfs,1);
Yc = zeros(nfs,1);
Zc = zeros(nfs,1);

kxx = zeros(nfs,1);
kyy = zeros(nfs,1);
kzz = zeros(nfs,1);
kxz = zeros(nfs,1);

Pts_zmax = zeros(nfs,3);
Pts_zmin = zeros(nfs,3);
Pts_ymax = zeros(nfs,3);

% added_thickness = zeros(length(CSDef(1).CS(:,2)),1);
% if strcmp(Fus.Type,'Nacelle')
%     diam_lin_beg = linspace(0,Fus.Max_Diam,ceil(length(CSDef.CS(:,2))));
%     diam_lin_end = linspace(Fus.Max_Diam,0,ceil(length(CSDef.CS(:,2)))-1);
%     added_thickness = [diam_lin_beg';diam_lin_end'];
% end


if isempty(Fus.Spectheta)
    th = (linspace(0,180,Fus.AziPtsPerSide))';
end

if~isempty(Fus.Spectheta)
    th = Fus.Spectheta';
end


for i = 1:nfs
    
    CS = CSDef(CS_ind(i)).CS;
    
    % normalized width: 1st column; calculate stretch factor
    y_norm = CS(:,1);
    y_stretchfactor = y_side(i)/max(y_norm);
    
    % normalized height: 2nd column; calculate stretch factor
    z_norm = CS(:,2);
    z_stretchfactor = Fus.ZStretchFactor * h(i)./(max(z_norm)-min(z_norm));
    
    % calculate dimensional y and z coordinates for this fuselage station
    y_dim = y_norm * y_stretchfactor;
    z_dim = z_norm * z_stretchfactor;
    
    % convert to polar coordinates
    q = 1;
    %     if i > 1 && i < length(added_thickness)
    %         tr = (added_thickness(i-1)./sqrt(y_dim.^2 + z_dim.^2));
    %         q = max(1,tr+1);
    %     end
    r_ref  = sqrt(y_dim.^2 + z_dim.^2).* q;
    
    
    th_ref = (atan2d(y_dim,z_dim))';
    
    if~isempty(Fus.Spectheta)
        th = th_ref';
    end
    
    r = (interp1(th_ref,r_ref,th,'linear','extrap'));
    
    
    y1 = r.*sind(th);
    z1 = -r.*cosd(th);
    
    ycs = [y1; -flipud(y1(1:end-1))];
    zcs = z_ctr(i) +[z1; flipud(z1(1:end-1))];
    xcs = x(i)*ones(size(ycs));
    
    sumPeri_dim(i) = sum(sqrt((ycs(1:end-1) - ycs(2:end)).^2 + ((zcs(1:end-1) - zcs(2:end))).^2));
    
    % cross-sectional area
    Area(i,1) = polyarea(ycs, zcs);
    
    
    % calculate sectional properties
    % Polygon = PolygonMoments (xy,mn,PlotFlag)
    Polygon = PolygonMoments([ycs,-zcs],[2,2],0);
    % note axis transformations:
    % function +x --> body axes +y
    % function +y --> body axes -z
    % function +z --> body axes -x
    % Function creates a variable "Polygon" in calling function's workspace
    kxx(i,1) = Polygon.kzz;
    kyy(i,1) = Polygon.kxx;
    kzz(i,1) = Polygon.kyy;
    ACy = Polygon.ACy;
    kxz(i,1) =  Polygon.kxy;  %1;
    
    
    R_BC = Fus.R_BC;
    xrp = Fus.RefPtLocation(1);
    yrp = Fus.RefPtLocation(2);
    zrp  = Fus.RefPtLocation(3);
    
    % centroid of cross-section
    Xc0 = x(i) + xrp;
    Yc0 = yrp;
    Zc0 =  -ACy  + zrp;
    
    Xc(i,1) = xrp + R_BC(1,1) * (Xc0-xrp) + R_BC(1,2) * (Yc0-yrp) + R_BC(1,3) * (Zc0-zrp);
    Yc(i,1) = yrp + R_BC(1,4) * (Xc0-xrp) + R_BC(1,5) * (Yc0-yrp) + R_BC(1,6) * (Zc0-zrp);
    Zc(i,1) = zrp + R_BC(1,7) * (Xc0-xrp) + R_BC(1,8) * (Yc0-yrp) + R_BC(1,9) * (Zc0-zrp);
    
    
    xcs0 = xcs + xrp;
    ycs0 = ycs + yrp;
    zcs0 = zcs + zrp;
    
    xcs = xrp + R_BC(1,1) * (xcs0-xrp) + R_BC(1,2) * (ycs0-yrp) + R_BC(1,3) * (zcs0-zrp);
    ycs = yrp + R_BC(1,4) * (xcs0-xrp) + R_BC(1,5) * (ycs0-yrp) + R_BC(1,6) * (zcs0-zrp);
    zcs = zrp + R_BC(1,7) * (xcs0-xrp) + R_BC(1,8) * (ycs0-yrp) + R_BC(1,9) * (zcs0-zrp);
    
    CSPts = [CSPts; xcs, ycs, zcs, i*ones(size(xcs))];
    
    
    if Settings.plotFLAG == 1
        plot3(xcs,ycs,zcs,ColorVec(CS_ind(i)))
    else
    end
    
    [~,indzmax] = max(zcs);
    [~,indzmin] = min(zcs);
    [~,indymax] = max(ycs);
    
    Pts_000(i,:) = [xcs(th_000),ycs(th_000), zcs(th_000)];
    Pts_090(i,:) = [xcs(th_090),ycs(th_090), zcs(th_090)];
    Pts_180(i,:) = [xcs(th_180),ycs(th_180), zcs(th_180)];
    Pts_270(i,:) = [xcs(th_270),ycs(th_270), zcs(th_270)];
end
%% Calculating Wetted Area
Peri_av = .5*(sumPeri_dim(1:end-1) + sumPeri_dim(2:end));
dXc = diff(Xc);
dA_Wet = abs(Peri_av'.*diff(Xc));

Stn.i = (1:nfs)';
Stn.FS = x_m/x_m(end);
Stn.x_m = x_m;
Stn.CS = CS_ind;
Stn.CS_Area = Area;
Stn.Xc = Xc;
Stn.Yc = Yc;
Stn.Zc = Zc;


% length between successive cross sections
Stn.dFS = -[0; diff(Stn.Xc)];

% height and width
Stn.Height = h';
Stn.Width = w';

% Wetted Area
Stn.dA_Wet = [0;dA_Wet];
Stn.sdA_Wet = cumsum(Stn.dA_Wet);

% volume
Stn.dV = [0;abs(0.5.*(Area(1:end-1) + Area(2:end)).*diff(Xc))*Fus.eff_V];

% cumulative volume
Stn.sdV = cumsum(Stn.dV);

% distribute fuselage mass in the ratio of segment volumes
Stn.dm = Fus.Mass.*Stn.dV/sum(Stn.dV);

% calculate CG fuselage station (FS) based on this mass distribution
CG_FS = sum(Stn.dm.*Stn.FS)/Fus.Mass;

% CASE 1: No CG location or CG range provided
% CASE 2: CG location provided as an anticipated range
% CASE 3: CG location provided as a single point value


if ~isempty(Fus.CG_FS)
    % means either CASE 2 or CASE 3
    
    MassRedistReqd = 0;
    
    % CASE 2: check if calcuated CG FS falls within anticipated range
    if length(Fus.CG_FS)==2 % meaning a range has been provided
        CG_FS_calc = CG_FS;
        
        % is calculated CG too far forward?
        CalcCGTooFarFwd = CG_FS_calc < Fus.CG_FS(1);
        
        % is calculated CG too far aft?
        CalcCGTooFarAft = CG_FS_calc > Fus.CG_FS(2);
        
        % is mass redistribution required?
        MassRedistReqd = CalcCGTooFarFwd + CalcCGTooFarAft;
        
        % make sure propagating CG FS lies within the anticipated interval
        CG_FS = max(Fus.CG_FS(1),min(Fus.CG_FS(2),CG_FS_calc));
        
    end
    
    % CASE 3:
    if length(Fus.CG_FS)==1 % meaning a single value has been provided
        CG_FS = Fus.CG_FS; % capture the specified value
        MassRedistReqd = 1;
    end
    
    
    % If mass redistribution is required...
    if MassRedistReqd
        
        ConvFlag = 0;
        iter = 0;
        
        while(ConvFlag==0&&iter<100)
            
            iter = iter + 1;
            % new mass distribution: dm' = dm (a + b.fs)
            % condition 1: sum(dm') = Mass
            % condition 2: sum(dm' fs) = Mass x cg_fs
            
            A = [Fus.Mass, sum(Stn.FS.*Stn.dm); sum(Stn.FS.*Stn.dm), sum(Stn.dm.*Stn.FS.^2)];
            B = [Fus.Mass; Fus.Mass * CG_FS];
            
            X = A\B;
            
            a = X(1); b = X(2);
            
            Stn.dm = Stn.dm.*(a + b.*Stn.FS);
            
            if(min(Stn.dm(2:end))>=0)
                ConvFlag = 1;
            else
                Stn.dm(2:end) = Stn.dm(2:end) - min(Stn.dm(2:end));
            end
            
        end
        
    end
end



% Recalculate CG fuselage station
CG_FS = sum(Stn.dm.*Stn.FS)/Fus.Mass;
Fus.CG_FS = CG_FS;

% calculate the CG location based on this mass distribution
Xcg = sum(Stn.dm.*Stn.Xc)/Fus.Mass;
Ycg = sum(Stn.dm.*Stn.Yc)/Fus.Mass;
Zcg = sum(Stn.dm.*Stn.Zc)/Fus.Mass;
Fus.CG = [Xcg,Ycg,Zcg];











% plot the CG location
% scatter3(Xcg,Ycg,Zcg,20,'r','filled')










% moment of inertia calculations
dx = [Stn.dFS];
kxx_av = [0;(kxx(1:end-1) + kxx(2:end))/2];
kyy_av = [0;(kxx(1:end-1) + kxx(2:end))/2];
kzz_av = [0;(kxx(1:end-1) + kxx(2:end))/2];
kxz_av = [0;(kxx(1:end-1) + kxx(2:end))/2];

Ixx0 = Stn.dm.*(kxx_av.^2 + dx.^2/12);
Iyy0 = Stn.dm.*(kyy_av.^2);
Izz0 = Stn.dm.*(kzz_av.^2 + dx.^2/12);

Ixy0 = 0 * Ixx0;        % are they zero?
Iyz0 = 0 * Ixx0;       % are they zero?
Ixz0 = Stn.dm.*(kxz.^2); % is this legit?



% apply parallel axis theorem
R = R_BC;
% Ixx,Iyy,Izz,Ixy,Iyz,Ixz
Ica = [Ixx0,Iyy0,Izz0,Ixy0,Iyz0,Ixz0];

CG = [Xc,Yc,Zc];

[Ixx,Iyy,Izz,Ixy,Iyz,Izx] = ApplyParallelAxisTheorem(Stn.dm,CG,Ica,R);

Stn.Ixx = Ixx;
Stn.Iyy = Iyy;
Stn.Izz = Izz;
Stn.Ixy = Ixy;
Stn.Iyz = Iyz;
Stn.Izx = Izx;



Fus.Ixx = sum(Ixx);
Fus.Iyy = sum(Iyy);
Fus.Izz = sum(Izz);
Fus.Ixy = sum(Ixy);
Fus.Iyz = sum(Iyz);
Fus.Izx = sum(Izx);








Stn = struct2table(Stn);
Fus.Stn = Stn;

% plot top, bottom, and side lines
if Settings.plotFLAG == 1
    plot3(Pts_000(:,1),Pts_000(:,2),Pts_000(:,3),'k')
    plot3(Pts_090(:,1),Pts_090(:,2),Pts_090(:,3),'k')
    plot3(Pts_180(:,1),Pts_180(:,2),Pts_180(:,3),'k')
    plot3(Pts_270(:,1),Pts_270(:,2),Pts_270(:,3),'k')
else
end






%% AC3D geometry export
% number of points per fuselage cross section
ncsp = length(xcs);

% front closing face
FrontFace = CSPts(CSPts(:,4)==1,:);
FrontClosePoint = repmat(mean(FrontFace),[ncsp,1]);

% aft closing face
AftFace = CSPts(CSPts(:,4)==nfs,:);
AftClosePoint = repmat(mean(AftFace),[ncsp,1]);

% number of fuselage cross-sections including front and aft closing faces
np = nfs + 2;

i = repmat((1:np-1),[ncsp-1,1]); i = reshape(i,[],1);
j = repmat((1:ncsp-1)',[np-1,1]);

% Create index numbers for the quad surfaces
P1 = j +     (i-1)*ncsp;
P2 = j + 1 + (i-1)*ncsp;
P3 = j + 1 + (i)*ncsp;
P4 = j +     (i)*ncsp;

Fus.Vertices = [FrontClosePoint(:,1:3);CSPts(:,1:3);AftClosePoint(:,1:3)];

% order set to give outward normals: Oct 20, 2021
% Fus.Surfaces = [P4,P3,P2,P1];
Fus.Surfaces = [P1,P2,P3,P4];
% get fuselage centerline and height at ref point
Fus.height = interp1(Fus.Stn.FS, Fus.Stn.Height,Fus.RefPt_FS,'linear','extrap');
Fus.CirZ = interp1(Fus.Stn.FS, Fus.Stn.Zc,Fus.RefPt_FS,'linear','extrap');

stopper = 1;

end