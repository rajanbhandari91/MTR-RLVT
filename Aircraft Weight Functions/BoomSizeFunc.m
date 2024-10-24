function [M_L,FOS, tsel, defl] = BoomSizeFunc(M,T,L,h,b,t,FOSTarget,flag)


%% CONSTANTS

%Material Properties - Aluminum 6160
% E = 69e9;         %Young's Modulus (Pa)
E = 183e6 ;
% v = 0.3;          %Poisson Ratio
% sigma_y = 240e6;    %Yield Tensile Strength (Pa)
sigma_y = 827.37e6; 
% tau_y = 207e6;    %Shear Strength (Pa)
rho = 1550;         %kg/m^3

M_max = M;
tw = t;

%% MOI SELECTION

% Bending Stress = M*C/I
% M = bending moment
% C = height from top to principal axis
% I = moment of inertia

if flag == 1
    % x_section = 'hollow rect';
    Ixx = (1./12.)*(b.*h.^3 - (b-2.*tw).*(h-2.*tw).^3); %(un^4)
%     Iyy = 1./12.*(h.*b.^3 - (h-2.*tw).*(b-2.*tw).^3); %(un^4)
%     Q_NA = 2.*((1./2.*h-tw).*tw).*(y_bar - tw) + b.*tw.*(y_bar - tw./2); %(un^3)
    A = b.*h - (b-2.*tw).*(h-2.*tw); %(un^2)
%     A = h*b - (h-2.*t).*(b-2.*t);
elseif flag == 2   
    % x_section = 'I-beam';
    tf = tw;
    A = 2.*b.*tf + (h - 2.*tw).*tw; %(un^2)
%     x_c = b./2;
    Ixx = b.*h.^3./12 - 1./12.*(b - tw).*(h - 2.*tf).^3; %(un^4)
%     Iyy = (h - 2.*tf).^3 + 2./12.*tf.*b.^3; %(un^4)
%     Q_NA = (h./2.*tw).*(y_bar + h./2) + (flange.*tw).*(y_bar - tw./2); %(un^3)
else
    error('Please select a cross-section!')
end

%% CALCULATIONS
y_bar = h/2;
sigma = -M_max.*y_bar./Ixx; %-M*y/I
% tau = V_max.*Q_NA./(Ixx.*tw); %shear loading (if desired, but V_max is required)

%% OUTPUTS
FOSRef = abs(sigma_y./sigma);
M_LRef = A*rho;

M_Lquery= interp1(FOSRef,M_LRef,FOSTarget,'linear'); M_L = M_Lquery;
tsel = interp1(FOSRef,tw,FOSTarget,'linear');

M_L(isnan(M_Lquery) && FOSRef(1) > FOSTarget) = M_LRef(1);
tsel(isnan(M_Lquery) && FOSRef(1) > FOSTarget) = tw(1);

M_L(isnan(M_Lquery) && FOSRef(end) < FOSTarget) = M_LRef(end);
tsel(isnan(M_Lquery) && FOSRef(end) < FOSTarget) = tw(end);


FOS = interp1(tw,FOSRef,tsel,'linear');
Ixx_sel = interp1(tw,Ixx,tsel,'linear');
% deflection
defl = T * L^3 / (3 * E * Ixx_sel);

end