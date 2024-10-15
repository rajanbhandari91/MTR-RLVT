function varargout = GeomEval_InitDEPNacelle(varargin)
i = 1;
while i < nargin
    varargout{i}.Name = {varargin{i}};
    varargout{i}.Type = varargin{i+1};
    if strcmp(varargout{i}.Type,'Nacelle')

        varargout{i}.Shape = 1; % 1 - Flow through; 2 - Enclosed
        varargout{i}.Length = 1;
        varargout{i}.Max_Diam = 1;
        varargout{i}.eff_V = 1;
        varargout{i}.RefPt_FS = .5;
        
        varargout{i}.Mass = 0.0000000000000000001;
        varargout{i}.AirfoilName = 'symmetricAF';
        varargout{i}.RootIncidence = 0;
        varargout{i}.Twist = 0;
        varargout{i}.RootDihedral = 0;
        varargout{i}.RefPtLocation = [];
        varargout{i}.FinenessRatio = [];
        varargout{i}.LineDef = 'Generic_Nacelle_Lines';
        varargout{i}.CrossSections(1).Name = 'CS1';
        varargout{i}.CrossSections(1).Defn = 'CS_circ';
        varargout{i}.CrossSections(1).FS = [0, 1];

        varargout{i}.CG_FS = [];

        varargout{i}.R_BC = [1,0,0,0,1,0,0,0,1];

        varargout{i}.Directionality = 1;

        varargout{i}.CG = [0 0 0];
        varargout{i}.Ixx = 0;
        varargout{i}.Iyy = 0;
        varargout{i}.Izz = 0;
        varargout{i}.Ixy = 0;
        varargout{i}.Iyz = 0;
        varargout{i}.Izx = 0;
        varargout{i}.ColorVec = 'r';
        varargout{i}.AziPtsPerSide = 20;
        varargout{i}.MaxDistBetweenStations = 0.01;
        varargout{i}.Spectheta = [];
        varargout{i}.Architecture = {'-'};
        
        
        
        varargout{i}.YStretchFactor = 1;
        varargout{i}.ZStretchFactor = 1;
        
        
    else strcmp(varargout{i}.Type,'Pylon')

        varargout{i}.PlanformArea = 1;
        varargout{i}.AspectRatio = 4;

        varargout{i}.Directionality = 1;

        varargout{i}.TaperDefn = [];
        varargout{i}.SweepDefn = [];
        varargout{i}.Dihedral = 0;
        varargout{i}.Twist = 0;
        varargout{i}.RootDihedral = 0;
        varargout{i}.RootIncidence = 0;

        varargout{i}.t_min = [];

        varargout{i}.RefPtLocation = [0;0;0];
        varargout{i}.RefPtChordFrac = 0;

        varargout{i}.ExposedEtas = [0,1];
        varargout{i}.SpecEtas = [];
        varargout{i}.FinenessRatio = [];

        varargout{i}.AirfoilName = 'symmetricAF';

        varargout{i}.Controls = [];

        varargout{i}.PropEtas = [];
        varargout{i}.PropDiams = 0;

        varargout{i}.Span =[];
        varargout{i}.Mass = 0.0000000000000000001;
        varargout{i}.EtaMounting = 0;                 % spanwise mounting of another lifting surface onto this one. Default = 0

        varargout{i}.CG = [0 0 0];
        varargout{i}.Ixx = 0;
        varargout{i}.Iyy = 0;
        varargout{i}.Izz = 0;
        varargout{i}.Ixy = 0;
        varargout{i}.Iyz = 0;
        varargout{i}.Izx = 0;

        varargout{i}.StripMaxWidth = 0.3;
        varargout{i}.GenUniformSpacing = 0;
        varargout{i}.Architecture = {'-'};
        varargout{i}.Stn = [];
        
        varargout{i}.YStretchFactor = 1;
        varargout{i}.ZStretchFactor = 1;
        
    end
    i = i+2;

end
for i = 3:nargin-1
    varargout{i-1} = varargout{i};
end
end
