function [Fan] = GeomEval_InitDuctedFan(varargin)

Fan.Name = [];
if nargin>0
    Fan.Name = {varargin{1}};
end
Fan.Type = 2; % 1 - ducted fan; 2 - open rotor
Fan.Length = 1;
Fan.Diam = .25;
Fan.RefPt_FS = 0.50;

Fan.Directionality = 1;
Fan.TaperDefn = [];

Fan.Psi = 0; % About z-axis
Fan.Theta = 0; % About y-axis
Fan.Phi = 0; % About x-axis
Fan.RefPtLocation = [0,0,0];

Fan.CG = [0 0 0];
Fan.Ixx_ca = 0;
Fan.Iyy_ca = 0;
Fan.Izz_ca = 0;
Fan.Ixy_ca = 0;
Fan.Iyz_ca = 0;
Fan.Izx_ca = 0;
Fan.Ixx = 0;
Fan.Iyy = 0;
Fan.Izz = 0;
Fan.Ixy = 0;
Fan.Iyz = 0;
Fan.Izx = 0;
Fan.Architecture = {'-'};
Fan.Mass = 1e-7;
Fan.MaxDistBetweenStations = 0.05;
Fan.AziPtsPerSide = 50;
Fan.n_eta = 1;
Fan.SpinDir = 1;