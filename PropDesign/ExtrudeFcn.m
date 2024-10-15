function [Geom] = ExtrudeFcn(inp,varargin)

if nargin>1
    angres = varargin{1};
else
    angres = 30;
end


angres = 2;

Geom = [];

x = inp(:,1);
y = inp(:,2);
z = inp(:,3);
r = inp(:,4);

n = length(x);


th = [0:angres:360]';

cnorm = [zeros(size(th)),cosd(th),sind(th)];
[nang,~] = size(cnorm);

Vertices = [];
Surfaces = [];



psd = atan2(diff(y),diff(x));
psd = [psd(1);psd];

dd = sqrt(diff(x).^2+diff(y).^2+diff(z).^2);
thd = asin(diff(z)./dd);
thd = [thd(1);thd];


startindex = 0;

for i = 1:1:n-1
    
    x0 = x(i);
    x1 = x(i+1);
    
    y0 = y(i);
    y1 = y(i+1);
    
    z0 = z(i);
    z1 = z(i+1);
    
    r0 = r(i);
    r1 = r(i+1);
    
    d = sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2);
    

    
    % find psi rotation
    ps = atan2(y1-y0,x1-x0);
    
    % find theta rotation
    th = asin((z1-z0)/d);

    
    
    Rps = [cos(ps),-sin(ps),0; sin(ps),cos(ps), 0; 0,0,1];
    Rth = [cos(th), 0, -sin(th); 0, 1, 0; sin(th), 0, cos(th)];
    
    R = Rps*Rth;
    
    
    
    
    c0 = [x0,y0,z0] + r0*(R*cnorm')';
    c1 = [x1,y1,z1] + r1*(R*cnorm')';
    
%     plot3(c0(:,1),c0(:,2),c0(:,3))
%     plot3(c1(:,1),c1(:,2),c1(:,3))
    
    Vertices = [Vertices;c0;c1];
    
    SurfTemp = zeros(nang,4);
    
    
    SurfTemp(:,1) = startindex+(1:1:nang)';
    SurfTemp(:,2) = startindex+[(2:1:nang)';1];
    SurfTemp(:,3) = startindex+[(nang+2:1:2*nang)';nang+1];
    SurfTemp(:,4) = startindex+(nang+1:1:2*nang)';
    
    startindex = max(max(SurfTemp));
    
    
    Surfaces = [Surfaces;SurfTemp];
    
    
end

Geom.Vertices = Vertices;
Geom.Surfaces = Surfaces;
