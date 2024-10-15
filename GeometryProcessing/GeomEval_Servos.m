function [Gen] = GeomEval_Servos(varargin)
Gen.Name = [];
if nargin>0
    Gen.Name = {varargin{1}};
    NumComp = varargin{2}; % number of components
end


Gen.Type = 'Generic Component';
Gen.Shape = 'Cylinder';
Gen.RefPtLocation = zeros(NumComp,3);
Gen.Mass = ones(1,NumComp).*26e-3;
Gen.CG = zeros(NumComp,3);

Gen.Ixx_ca = zeros(1,NumComp);
Gen.Iyy_ca = zeros(1,NumComp);
Gen.Izz_ca = zeros(1,NumComp);
Gen.Ixy_ca = zeros(1,NumComp);
Gen.Iyz_ca = zeros(1,NumComp);
Gen.Izx_ca = zeros(1,NumComp);

Gen.xDim_m = zeros(1,NumComp);
Gen.yDim_m = zeros(1,NumComp);
Gen.zDim_m = zeros(1,NumComp);

Gen.Ixx = zeros(1,NumComp);
Gen.Iyy = zeros(1,NumComp);
Gen.Izz = zeros(1,NumComp);
Gen.Ixy = zeros(1,NumComp);
Gen.Iyz = zeros(1,NumComp);
Gen.Izx = zeros(1,NumComp);
Gen.ph = zeros(1,NumComp);
Gen.th = zeros(1,NumComp);
Gen.psi = zeros(1,NumComp);
% Gen.Architecture = {'0'};
end