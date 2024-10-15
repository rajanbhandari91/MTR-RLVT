function Fan = GeomEval_DuctedFan(Fan)
global Settings
xunit = linspace(0,Fan.Length,Fan.n_eta); % Discretized Length
if xunit(end) == 0
    eta = 0;
else
    eta  = xunit./xunit(end);
end

% straight taper
if length(Fan.TaperDefn)==1
    Fan.TaperDefn = linspace(1,Fan.TaperDefn,length(xunit));
end

radius = Fan.TaperDefn*Fan.Diam./2;

th = 0:pi/Fan.AziPtsPerSide:2*pi;
% Finding the y and z for every angle
for i = 1:length(xunit)
    xi(:,i) = radius(i) * cos(th);
    yi(:,i) = radius(i) * sin(th);
    
end



% making a circle along each eta
yId = ones(length(yi(:,1)),1);
%% Finding moments of inertia
Fan.Ixx_ca = 0.5*Fan.Mass*radius(1).^2; 
Fan.Iyy_ca = 0.5*Fan.Mass*radius(1).^2; 
Fan.Izz_ca = Fan.Mass*radius(1).^2; % rotation axis
Fan.Ixy_ca = 0;
Fan.Iyz_ca = 0;
Fan.Izx_ca = 0;



mu_i = Fan.Mass/length(xunit);
mu = mu_i*ones(length(xunit),1);
% dm = (mu/2).*ones(length(xunit),1);

% Xcg = sum(dm.*xcg)/Fan.Mass;
% Ycg = sum(dm.*ycg)/Fan.Mass;
% Zcg = sum(dm.*zcg)/Fan.Mass;

Fan.CG = [Fan.RefPtLocation(1) ,Fan.RefPtLocation(2),Fan.RefPtLocation(3)];

% scatter3(Fan.CG(1),Fan.CG(2),Fan.CG(3),20,'r','filled')
%% Wetted Area Calculation
Peri = pi.*radius.*2; % Calculating perimeter of normalized airfoil (C = pi*D)

dA_Wet = [0,0.5*(Peri(2:end) + Peri(1:end-1)).*diff(xunit)];
if isempty(dA_Wet)
    dA_Wet = 0;
end
%% Volume Calculation
dV = xunit.*(pi.*radius.^2);
%% Trying to rotate the fans
psi = Fan.Psi;
th = Fan.Theta;
phi = Fan.Phi;




R_ps = [cosd(psi), - sind(psi), 0; sind(psi), cosd(psi), 0; 0,0,1];
R_th = [cosd(th), 0, sind(th); 0, 1, 0; -sind(th), 0, cosd(th)];
R_ph = [1, 0, 0; 0, cosd(phi), -sind(phi); 0, sind(phi), cosd(phi)];


R = R_ps * R_th * R_ph;




R_BC(1,1) = R(1,1);
R_BC(1,2) = R(1,2);
R_BC(1,3) = R(1,3);
R_BC(1,4) = R(2,1);
R_BC(1,5) = R(2,2);
R_BC(1,6) = R(2,3);
R_BC(1,7) = R(3,1);
R_BC(1,8) = R(3,2);
R_BC(1,9) = R(3,3);



Fan.R_BC = R_BC;

for i = 1:length(xunit)
    
    
    xrp = Fan.RefPtLocation(1);
    yrp = Fan.RefPtLocation(2);
    zrp  = Fan.RefPtLocation(3);

    xcs0 = (xi(:,i) + xrp);
    ycs0 = (yi(:,i) + yrp);
    zcs0 = ( + zrp);

    
    xcs(:,i) = xrp + R_BC(1,1) .* (xcs0-xrp) + R_BC(1,2) .* (ycs0-yrp) + R_BC(1,3) .* (zcs0-zrp);
    ycs(:,i) = yrp + R_BC(1,4) .* (xcs0-xrp) + R_BC(1,5) .* (ycs0-yrp) + R_BC(1,6) .* (zcs0-zrp);
    zcs(:,i) = zrp + R_BC(1,7) .* (xcs0-xrp) + R_BC(1,8) .* (ycs0-yrp) + R_BC(1,9) .* (zcs0-zrp);
end


[Ixx_bf_rp,Iyy_bf_rp,Izz_bf_rp,Ixy_bf_rp,Iyz_bf_rp,Izx_bf_rp] = ApplyParallelAxisTheorem(Fan.Mass,Fan.CG,[Fan.Ixx_ca,Fan.Iyy_ca,Fan.Izz_ca,Fan.Ixy_ca,Fan.Iyz_ca,Fan.Izx_ca],R_BC);

Fan.Ixx = sum(Ixx_bf_rp);
Fan.Iyy = sum(Iyy_bf_rp);
Fan.Izz = sum(Izz_bf_rp);
Fan.Ixy = 0;
Fan.Iyz = 0;
Fan.Izx = sum(Izx_bf_rp);


if Fan.SpinDir == 1
    PlotColor = 'r';
end
if Fan.SpinDir == -1
    PlotColor = 'g';
end

if Fan.Type == 1 && Settings.plotFLAG == 1
    fill3(xcs,ycs,zcs,PlotColor)
    alpha(0.1)
elseif Fan.Type == 2 && Settings.plotFLAG == 1
    fill3(xcs,ycs,zcs,PlotColor)
    alpha(0.3)
else
end
Stn = table();

% Stn.i = (1:length(xunit))';
% Stn.eta = eta';
% Stn.radius = radius';
% Stn.tr = Fan.TaperDefn';
% Stn.xcg = xcg' + Fan.RefPtLocation(1);
% Stn.ycg = ycg' + Fan.RefPtLocation(2);
% Stn.zcg = zcg' + Fan.RefPtLocation(3);
% Stn.Ixx = Ixx';
% Stn.Iyy = Iyy';
% Stn.Izz = Izz';
% Stn.Ixy = Ixy*ones(length(xunit),1);
% Stn.Iyz = Iyz*ones(length(xunit),1);
% Stn.Izx = Izx*ones(length(xunit),1);
% Stn.dA = pi.*Stn.radius.^2;
% Stn.sdA = cumsum(Stn.dA);
% Stn.dA_Wet = dA_Wet';
% Stn.sdA_Wet = cumsum(Stn.dA_Wet);
% Stn.dV = dV';
% Stn.sdV = cumsum(Stn.dV);
% Stn.dm = mu_i*ones(length(xunit),1);
% Stn.ph = ones(length(xunit),1);
% Stn.th = ones(length(xunit),1);
% Stn.ps = ones(length(xunit),1);
% Stn.R11 = ones(length(xunit),1);
% Stn.R12 = ones(length(xunit),1);
% Stn.R13 = ones(length(xunit),1);
% Stn.R21 = ones(length(xunit),1);
% Stn.R22 = ones(length(xunit),1);
% Stn.R23 = ones(length(xunit),1);
% Stn.R31 = ones(length(xunit),1);
% Stn.R32 = ones(length(xunit),1);
% Stn.R33 = ones(length(xunit),1);
% 
% Fan.Stn = Stn;

end