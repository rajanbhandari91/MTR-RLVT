function [Fus] = GeomEval_InitBooms(varargin)


Fus.Name = [];
if nargin>0
    Fus.Name = {varargin{1}};
end



% Fus.Length = 10;
Fus.MaxHeight = 1.5;
Fus.MaxWidth = 1.5;
Fus.eff_V = 1;
Fus.RefPt_FS = 0.50;

Fus.CrossSections(1).Length = 10;

Fus.RefPtLocation = [0,0,0];

Fus.LineDef = 'Generic_FuselageLines';
Fus.Type = 'Fuselage';
Fus.CrossSections(1).Name = 'CS1';
Fus.CrossSections(1).Defn = 'CS_circ';
Fus.CrossSections(1).FS = [0, 1];
Fus.Mass = 0.000001;
Fus.CG_FS = [];

Fus.YStretchFactor = 1;
Fus.ZStretchFactor = 1;
Fus.FinenessRatio = [];

Fus.R_BC = [1,0,0,0,1,0,0,0,1];
% Fus.R_BC = [0,1,0,-1,0,0,0,0,1];


Fus.CG = [0 0 0];
Fus.Ixx = 0;
Fus.Iyy = 0;
Fus.Izz = 0;
Fus.Ixy = 0;
Fus.Iyz = 0;
Fus.Izx = 0;

Fus.ColorVec = 'kkkkkkkkkkkkkkkkkkkkkkkkk';
Fus.Architecture = {'-'};
Fus.AziPtsPerSide = 32;
Fus.Spectheta = [];
Fus.MaxDistBetweenStations = 0.07;