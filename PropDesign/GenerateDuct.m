function [Duct] = GenerateDuct(Duct)

matr = 1;           % not important, but do not remove
RGB = [1,0,0];      % not important, but do not remove


res_psi = 10;       % azimuthal offset between successive airfoils during revolution (deg)
nc = 40;

DAF = Duct.Airfoil;                             % duct airfoil coordinates, note: negative airfoil y-coordinates go on the "inside" of the duct
RotorRadius_m = Duct.RotorRadius_m;             % outer radius of the rotor that the duct goes around
RotorPosnOnDuctChord = Duct.RotorPosnOnDuctChord;    % Location of rotor tip on duct chord [0-1]
TipClearanceRatio = Duct.TipClearanceRatio;     % Tip clearance / Rotor outer radius ratio
IncidenceAngle = Duct.IncidenceAngle;           % Incidence angle of duct w.r.t. rotor axis (deg)
DuctAspectRatio = Duct.AspectRatio;                % Duct chord / Rotor outer radius ratio
XC = Duct.RotorLoc;                             % location of center of rotor (m)
theta = Duct.PitchAngle;                        % pitch inclination of duct axis (deg)
phi = Duct.BankAngle;                           % roll inclination of duct axis (deg)







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














Rtheta = [cosd(theta),0,sind(theta); 0, 1, 0; -sind(theta),0,cosd(theta)];
Rphi = [1,0,0;0,cosd(phi),-sind(phi);0,sind(phi),cosd(phi)];
Rnet = Rtheta*Rphi;


DuctDiameter = 2*RotorRadius_m*(1+TipClearanceRatio);
DuctChord = DuctDiameter/DuctAspectRatio;
DuctOffset = RotorPosnOnDuctChord * DuctChord;






cquery = uc';

US = [cquery,interp1(DAF.US(:,1),DAF.US(:,2),cquery,'linear')];
LS = [cquery,interp1(DAF.LS(:,1),DAF.LS(:,2),cquery,'linear')];

NAF = [US;flipud(LS(1:end-1,:))];


DAF_dim = NAF * DuctChord;

zg0 = DAF_dim(:,1) - DuctOffset;
xg0 = DAF_dim(:,2);
yg = zeros(size(xg0));



zg =  zg0*cosd(IncidenceAngle) - xg0*sind(IncidenceAngle);
xg = -zg0*sind(IncidenceAngle) + xg0*cosd(IncidenceAngle);


[n,~] = size(zg);

DuctInnerCoordinateAtRotorLocn = interp1(zg([floor(n/2)+5:end]),xg([floor(n/2)+5:end]),0,'linear','extrap');



R = RotorRadius_m*(1+TipClearanceRatio) - DuctInnerCoordinateAtRotorLocn;




psi = [0:res_psi:360]';

x = R*cosd(psi);
y = R*sind(psi);
z = zeros(size(x));


Vertices = [];
Surfaces = [];
nang = length(xg);

startindex = 0;

npsi = length(psi);

% for i = 1:1:npsi-1
%     
%    R1 = [cosd(psi(i)),-sind(psi(i)),0; sind(psi(i)),cosd(psi(i)),0;0,0,1];
%    R2 = [cosd(psi(i+1)),-sind(psi(i+1)),0; sind(psi(i+1)),cosd(psi(i+1)),0;0,0,1];
%    
%    VerticesTemp1 = [x(i),y(i),z(i)] + (R1*[xg,yg,zg]')';
%    VerticesTemp2 = [x(i+1),y(i+1),z(i+1)] + (R2*[xg,yg,zg]')';
%   % plot3(VerticesTemp(:,1),VerticesTemp(:,2),VerticesTemp(:,3)); 
%    Vertices = [Vertices;VerticesTemp1;VerticesTemp2];
%    
% 
%     
%     SurfTemp = zeros(nang,4);
%     
%     
%     SurfTemp(:,1) = startindex+(1:1:nang)';
%     SurfTemp(:,2) = startindex+[(2:1:nang)';1];
%     SurfTemp(:,3) = startindex+[(nang+2:1:2*nang)';nang+1];
%     SurfTemp(:,4) = startindex+(nang+1:1:2*nang)';
%     
%     startindex = max(max(SurfTemp));
%     
%     
%     Surfaces = [Surfaces;SurfTemp];
%    
% end


for i = 1:1:npsi
    
   R1 = [cosd(psi(i)),-sind(psi(i)),0; sind(psi(i)),cosd(psi(i)),0;0,0,1];
   %R2 = [cosd(psi(i+1)),-sind(psi(i+1)),0; sind(psi(i+1)),cosd(psi(i+1)),0;0,0,1];
   
   VerticesTemp1 = [x(i),y(i),z(i)] + (R1*[xg,yg,zg]')';
   %VerticesTemp2 = [x(i+1),y(i+1),z(i+1)] + (R2*[xg,yg,zg]')';
   Vertices = [Vertices;VerticesTemp1];%;VerticesTemp2];
   
   
end

for j = 1:npsi-1
   
    SurfTemp = zeros(nang-1,4);
    
    
    SurfTemp(:,1) = startindex + (1:1:nang-1)';
    SurfTemp(:,2) = startindex + (2:1:nang)';
    SurfTemp(:,3) = startindex + (nang+2:1:2*nang)';
    SurfTemp(:,4) = startindex + (nang+1:1:2*nang-1)';
    
    startindex = startindex + nang;
    
    Surfaces = [Surfaces;SurfTemp]; 
end



% Rotations
Vertices = XC+(Rnet*(Vertices'))';

%parameters.RGB = [1,0,0]

Duct.Vertices = Vertices;
Duct.Surfaces = Surfaces;
Duct.Material = matr;
Duct.RGB = RGB;
Duct.CG = XC;
Duct.Angles = [phi,theta,0];
Duct.Radius = R;


% [nsurf,ncol] = size(Surfaces);
%     for i = 1:1:nsurf
%         pt = Vertices(Surfaces(i,:),:);
%         fill3(pt(:,1),pt(:,2),pt(:,3),parameters.RGB)
%     end