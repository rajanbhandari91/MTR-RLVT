function [Geom] = GenerateRotors(RotorLoc,phi,theta,dirn,Setup,AFName,matr,RGB,tex,MCSFlag)



nrad = 10;  % number of radial points along the span of each blade
nc = 20;     % number of chordwise points along along upper and lower surfaces (each)





%%%% COMPUTE GROWTH FACTOR
growth_rate = 1.1;
ipts = nc;
    
growth_factor = 0;
for i = 1:ipts-1
        growth_factor = growth_factor + growth_rate^(i-1);
end


sizer = 0;
for i = 1:ipts
        
        if(i == 1)
            uc(i) = 0;
        end
        
        if(i >= 2)
            uc(i) = sizer + 1*(growth_rate^(i-2))/growth_factor;
        end
       
        if(i == ipts)
            uc(i) = 1;
        end
        
%         ulocal(i) = uc;
                    
        if(i >= 2)
            sizer = sizer + 1*(growth_rate^(i-2))/growth_factor;
        end
end



loadFileName = AFName;
load(loadFileName);

dpsi = 360/Setup.nBlades;


azi = 0;

psi = (azi:dpsi:359+azi);

% generate one blade

cref = [Setup.Chord(1),Setup.Chord,0];
rref0 = [-.001,Setup.REdges,Setup.REdges(end)+0.001]; %Setup.RMids(1),


rref = [linspace(0,rref0(end-1),nrad-1),rref0(end)];

chord = interp1(rref0,cref,rref,'linear');

thref = [Setup.Pitch(1),Setup.Pitch,Setup.Pitch(end)];
twist = interp1(rref0,thref,rref,'linear')*pi/180;

% capture number of stations
nsta = length(rref);

















% cquery = linspace(0,1,nc)';

cquery = uc';

US = [cquery,interp1(WAF_US(:,1),WAF_US(:,2),cquery,'linear')];
LS = [cquery,interp1(WAF_LS(:,1),WAF_LS(:,2),cquery,'linear')];

% approximate mean camber line as the average between the upper surface
% and the lower surface. 
MCL = [cquery,0.5*(US(:,2) + LS(:,2))];



if MCSFlag == 0
NAF = [US;flipud(LS(1:end-1,:))];
end


if MCSFlag == 1
NAF = MCL;
end


% check number of airfoil points
[nafp,~] = size(NAF);

[nmcs,~] = size(MCL);

















BladeVertices = [];
BladeSurfaces = [];
startindex = 0;
mcs_startindex = 0;

MCSVertices = [];
MCSSurfaces = [];

AF = zeros(nafp,3,nsta);

for i = 1:1:nsta
    
   xAF = zeros(nafp,1);
   yAF = ((NAF(:,1) - 0.5));
   zAF = NAF(:,2);
    
  
   AF(:,:,i) = chord(i)*[xAF,yAF,-zAF];
    
    % mean camber surface
    MCS(:,:,i) = chord(i)*[zeros(height(MCL),1),MCL(:,1)-0.5,MCL(:,2)];


    % formulate rotation tensors
    Rtwist= [1,0,0;0,cos(twist(i)),-sin(twist(i)); 0, sin(twist(i)), cos(twist(i))];
    %Rtheta = [cos(theta(i)),0, sin(theta(i)); 0, 1, 0; -sin(theta(i)), 0, cos(theta(i))];
    %Rpsi = [cos(psi(i)),-sin(psi(i)),0; sin(psi(i)),cos(psi(i)),0; 0,0,1];
    Rnet = Rtwist;

    
    % rotate airfoil coordinates
    AF(:,:,i) = (Rnet*(AF(:,:,i)'))';
 
    % rotate MCS coordinates
    MCS(:,:,i) = (Rnet*(MCS(:,:,i)'))';



    % translate by coordinates of r0
    AF(:,:,i) = AF(:,:,i) - [dirn*rref(i),0,0];
    
    
        
   

end



% for i = 1:nsta-1
%     BladeVertices = [BladeVertices;AF(:,:,i);AF(:,:,i+1)];
%     SurfTemp(:,1) = startindex+(1:1:nafp)';
%     SurfTemp(:,2) = startindex+[(2:1:nafp)';1];
%     SurfTemp(:,3) = startindex+[(nafp+2:1:2*nafp)';nafp+1];
%     SurfTemp(:,4) = startindex+(nafp+1:1:2*nafp)';
%     startindex = max(max(SurfTemp));
%     BladeSurfaces = [BladeSurfaces;SurfTemp];
%     
% end





for i = 1:1:nsta
    
   BladeVertices = [BladeVertices;AF(:,:,i)];
   MCSVertices = [MCSVertices; MCS(:,:,i)];
   
end

for j = 1:nsta-1
   
    SurfTemp = zeros(nafp-1,4);
    
    
    SurfTemp(:,1) = startindex + (1:1:nafp-1)';
    SurfTemp(:,2) = startindex + (2:1:nafp)';
    SurfTemp(:,3) = startindex + (nafp+2:1:2*nafp)';
    SurfTemp(:,4) = startindex + (nafp+1:1:2*nafp-1)';
    
    startindex = startindex + nafp;
    
    BladeSurfaces = [BladeSurfaces;SurfTemp]; 



    MCSSurfTemp = zeros(nmcs-1,4);
    
    
    MCSSurfTemp(:,1) = mcs_startindex + (1:1:nmcs-1)';
    MCSSurfTemp(:,2) = mcs_startindex + (2:1:nmcs)';
    MCSSurfTemp(:,3) = mcs_startindex + (nmcs+2:1:2*nmcs)';
    MCSSurfTemp(:,4) = mcs_startindex + (nmcs+1:1:2*nmcs-1)';
    
    mcs_startindex = mcs_startindex + nmcs;
    
    MCSSurfaces = [MCSSurfaces;MCSSurfTemp]; 






end














Vertices = [];
Surfaces = [];
surfincr = max(max(BladeSurfaces));

for j = 1:1:Setup.nBlades
   
    Rpsi = [cosd(psi(j)),-sind(psi(j)),0; sind(psi(j)),cosd(psi(j)),0; 0,0,1];
    VertTemp = (Rpsi'*(BladeVertices'))';
    Vertices = [Vertices;VertTemp];
    Surfaces = [Surfaces;BladeSurfaces + (j-1)*surfincr];
    
end

HubRad = 0.4*Setup.REdges(1);
AxialOffset = 0.00;

HubPts = [...
0,0,min(Vertices(:,3))-AxialOffset-0.0001,0    
0,0,min(Vertices(:,3))-AxialOffset,1.0*HubRad;
           0,0,max(Vertices(:,3))+AxialOffset,1.0*HubRad;
           0,0,max(Vertices(:,3))+AxialOffset+0.0001,0];
       
       
GeomTemp = ExtrudeFcn(HubPts);

Vertices = [Vertices;GeomTemp.Vertices];
Surfaces = [Surfaces;GeomTemp.Surfaces+Setup.nBlades*surfincr];

Geom.Surfaces = Surfaces;



Rphi = [1,0,0;0,cosd(phi),-sind(phi); 0, sind(phi), cosd(phi)];
Rtheta = [cosd(theta),0, sind(theta); 0, 1, 0; -sind(theta), 0, cosd(theta)];
    %Rpsi = [cos(psi(i)),-sin(psi(i)),0; sin(psi(i)),cos(psi(i)),0; 0,0,1];

Rtot = Rtheta*Rphi;

Geom.Vertices = RotorLoc + (Rtot*(Vertices'))';
Geom.Material = matr;
Geom.RGB = RGB;
Geom.Texture = tex;
Geom.CG = RotorLoc;
Geom.Angles = [phi,theta,0];
Geom.nBlades = Setup.nBlades;
Geom.Radius = Setup.RotorRadius;