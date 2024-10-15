function varargout =  GeomEval_DEPNacelle(varargin)
i = 1;
while i <= nargin
    varargout{i} = varargin{i};
    if strcmp(varargin{i}.Type,'Pylon') && nargin == 1

        [varargout{i}] =  GeomEval_LiftingSurface(varargin{i});

    elseif strcmp(varargin{i}.Type,'Nacelle') && nargin == 1
        toe = varargin{i}.Twist * varargin{i}.Directionality;
        cant = varargin{i}.RootDihedral;

        R_ps = [cosd(toe), -sind(toe), 0; sind(toe), cosd(toe), 0; 0,0,1];
        R_th = [cosd(cant), 0, sind(cant); 0, 1, 0; -sind(cant), 0, cosd(cant)];
        R = R_ps * R_th;

        R_BC(1,1) = R(1,1);
        R_BC(1,2) = R(1,2);
        R_BC(1,3) = R(1,3);
        R_BC(1,4) = R(2,1);
        R_BC(1,5) = R(2,2);
        R_BC(1,6) = R(2,3);
        R_BC(1,7) = R(3,1);
        R_BC(1,8) = R(3,2);
        R_BC(1,9)= R(3,3);
        varargin{i}.K_ng = 1;
        varargin{i}.R_BC = R_BC;

        [varargout{i}] =  GeomEval_Fuselage(varargin{i});

    elseif strcmp(varargin{i}.Type,'Nacelle') && strcmp(varargin{i+1}.Type,'Pylon')

        [varargout{i+1}] =  GeomEval_LiftingSurface(varargin{i+1});

        varargin{i}.RefPtLocation = [varargout{i+1}.Stn.xcg(end);varargout{i+1}.Stn.ycg(end);varargout{i+1}.Stn.zcg(end)];

        toe = varargin{i}.Twist * varargin{i}.Directionality;
        cant = varargin{i}.RootDihedral;

        R_ps = [cosd(toe), -sind(toe), 0; sind(toe), cosd(toe), 0; 0,0,1];
        R_th = [cosd(cant), 0, sind(cant); 0, 1, 0; -sind(cant), 0, cosd(cant)];
        R = R_ps * R_th;

        R_BC(1,1) = R(1,1);
        R_BC(1,2) = R(1,2);
        R_BC(1,3) = R(1,3);
        R_BC(1,4) = R(2,1);
        R_BC(1,5) = R(2,2);
        R_BC(1,6) = R(2,3);
        R_BC(1,7) = R(3,1);
        R_BC(1,8) = R(3,2);
        R_BC(1,9)= R(3,3);
        varargin{i}.K_ng = 1.017;
        varargin{i}.R_BC = R_BC;
        


        [varargout{i}] =  GeomEval_Fuselage(varargin{i});
    else strcmp(varargin{i}.Type,'Pylon') && strcmp(varargin{i+1}.Type,'Nacelle')

        [varargout{i}] =  GeomEval_LiftingSurface(varargin{i});

        varargin{i+1}.RefPtLocation = [varargout{i}.Stn.xcg(end);varargout{i}.Stn.ycg(end);varargout{i}.Stn.zcg(end)];

        toe = varargin{i}.Twist * varargin{i}.Directionality;
        cant = varargin{i}.RootDihedral;

        R_ps = [cosd(toe), -sind(toe), 0; sind(toe), cosd(toe), 0; 0,0,1];
        R_th = [cosd(cant), 0, sind(cant); 0, 1, 0; -sind(cant), 0, cosd(cant)];
        R = R_ps * R_th;

        R_BC(1,1) = R(1,1);
        R_BC(1,2) = R(1,2);
        R_BC(1,3) = R(1,3);
        R_BC(1,4) = R(2,1);
        R_BC(1,5) = R(2,2);
        R_BC(1,6) = R(2,3);
        R_BC(1,7) = R(3,1);
        R_BC(1,8) = R(3,2);
        R_BC(1,9)= R(3,3);
        varargin{i+1}.K_ng = 1.017;
        varargin{i}.R_BC = R_BC;
        


        [varargout{i+1}] =  GeomEval_Fuselage(varargin{i+1});
    end
    if i+1 >= nargin
        break
    else
        i = i+1;
        continue
    end
end
end