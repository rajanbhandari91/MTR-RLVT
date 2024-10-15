function Wing = getParamLiftingSurface(GeomNodes,degenGeom,Name)

conv_sqft_sqm = 0.092903;
conv_ft_m = 0.30488;

GeomName = 'WingGeom';
for i = 0:2:GeomNodes.getLength()
    try
        GeomNode = GeomNodes.item(i);
        WingName = GeomNode.getElementsByTagName('Name').item(0);
        if strcmp(char(WingName.getTextContent),Name)
            % Find the type of component
            TypeNodesWing = GeomNode.getElementsByTagName('GeomBase').item(0);
            TypeNodeWing = TypeNodesWing.getElementsByTagName('TypeName').item(0);
            TypeWing = char(TypeNodeWing.getTextContent());

            if strcmp(TypeWing,'Wing') ~= 1
                break
            end

            WingNodes = GeomNode.getElementsByTagName(GeomName);
            WingNode = WingNodes.item(1);

            % Find all XSecSurf elements within wing
            xsecSurfNodesWing = WingNode.getElementsByTagName('XSecSurf');
            xsecSurfNodeWing = xsecSurfNodesWing.item(0);

            % Find XForm for root incidence angle
            xFormNodesWing = GeomNode.getElementsByTagName('XForm');
            xFormNodeWing = xFormNodesWing.item(0);

            % Find Sym to find the planar and axial symmetry of lifting
            % surfaces
            SymmNodesWing = GeomNode.getElementsByTagName('Sym').item(0);
            SymPlanarNode = SymmNodesWing.getElementsByTagName('Sym_Planar_Flag').item(0);
            SymmFlag = str2double(SymPlanarNode.getAttribute('Value'));

            % Find one more Geom node to get control surfaces definition
            GeomControlNode = GeomNode.getElementsByTagName('Geom').item(0);
        end
    catch
    end
end

% Find all XSec elements within XSecSurf
% The first three XSec nodes are not useful to get the correct lifting surface
% geometry parameters
% So, we need to use those after the third XSec nodes. In matlab the node
% count starts from 0
% The last Xsec node also cannot be used. So, if there are 6 sections of
% XSec, we need to extract from index 3 or 4.
% Similarly, if there are more than one sections in a lifting surface, geom parameters
% can be extracted starting from index 3 to end-1 at every third interval
% from 3. This is because 3 and 4, 6 and 7, or 9 and 10 contains same
% information for each sections respectively.
%% Wing
if strcmp(TypeWing,'Wing') == 1
    xsecNodes = xsecSurfNodeWing.getElementsByTagName('XSec');
    numXSecs = xsecNodes.getLength();

    % Number of lifting surface sections modeled in openVSP can be determined
    % by numXSecs
    Sec = (numXSecs-3)/3;
    Span = zeros(length(Sec),1);
    PlanformArea = zeros(length(Sec),1);
    AR = zeros(length(Sec),1);
    Sweep = zeros(length(Sec),1);
    SweepChordLoc = zeros(length(Sec),1);
    Dihedral = zeros(length(Sec),1);
    Twist = zeros(length(Sec),1);
    InLESweep = zeros(length(Sec),1);
    OutLESweep = zeros(length(Sec),1);
    InLEDihedral = zeros(length(Sec),1);
    OutLEDihedral = zeros(length(Sec),1);
    TwistLocChord = zeros(length(Sec),1);
    RootChord = zeros(length(Sec),1);
    TipChord = zeros(length(Sec),1);
    %
    j = 1;
    for i = 3:3:numXSecs
        if i < numXSecs-1
            xsecNode = xsecNodes.item(i);

            % Extract Span
            spanNode = xsecNode.getElementsByTagName('Span').item(0);
            Span(j) = str2double(spanNode.getAttribute('Value'))*conv_ft_m;

            % Extract Projected Span
            ProjspanNodeHStab = xsecNode.getElementsByTagName('ProjectedSpan').item(0);
            ProjSpan(j) = str2double(ProjspanNodeHStab.getAttribute('Value'))*conv_ft_m;

            % Extract Planform Area
            areaNode = xsecNode.getElementsByTagName('Area').item(0);
            PlanformArea(j) = str2double(areaNode.getAttribute('Value'))*conv_sqft_sqm;

            % Extract Aspect Ratio
            ARNode = xsecNode.getElementsByTagName('Aspect').item(0);
            AR(j) = str2double(ARNode.getAttribute('Value'));

            % Extract Taper Ratio
            % TRNode = xsecNode.getElementsByTagName('Taper').item(0);
            % TR(j) = str2double(TRNode.getAttribute('Value'));

            % Extract Root Chord
            RCNode = xsecNode.getElementsByTagName('Root_Chord').item(0);
            RootChord(j) = str2double(RCNode.getAttribute('Value'))*conv_ft_m;

            % Extract Root Chord
            TCNode = xsecNode.getElementsByTagName('Tip_Chord').item(0);
            TipChord(j) = str2double(TCNode.getAttribute('Value'))*conv_ft_m;

            % Extract Sweep
            SweepNode = xsecNode.getElementsByTagName('Sweep').item(0);
            Sweep(j) = str2double(SweepNode.getAttribute('Value'));

            % Extract InboardLE Sweep
            InLESweepNode = xsecNode.getElementsByTagName('InLESweep').item(0);
            InLESweep(j) = str2double(InLESweepNode.getAttribute('Value'));

            % Extract OutboardLE Sweep
            OutLESweepNode = xsecNode.getElementsByTagName('OutLESweep').item(0);
            OutLESweep(j) = str2double(OutLESweepNode.getAttribute('Value'));

            % Extract Location along chord
            SweepChordLocNode = xsecNode.getElementsByTagName('Sweep_Location').item(0);
            SweepChordLoc(j) = str2double(SweepChordLocNode.getAttribute('Value'));

            % Extract Dihedral
            DihedralNode = xsecNode.getElementsByTagName('Dihedral').item(0);
            Dihedral(j) = str2double(DihedralNode.getAttribute('Value'));

            % Extract InboardLEDihedral
            InLEDihedralNode = xsecNode.getElementsByTagName('InLEDihedral').item(0);
            InLEDihedral(j) = -str2double(InLEDihedralNode.getAttribute('Value'));

            % Extract InboardLEDihedral
            OutLEDihedralNode = xsecNode.getElementsByTagName('OutLEDihedral').item(0);
            OutLEDihedral(j) = str2double(OutLEDihedralNode.getAttribute('Value'));

            % Extract Twist
            TwistNode = xsecNode.getElementsByTagName('Twist').item(0);
            Twist(j) = str2double(TwistNode.getAttribute('Value'));

            % Extract TwistLoc Along chord
            TwistLocChordNode = xsecNode.getElementsByTagName('Twist_Location').item(0);
            TwistLocChord(j) = str2double(TwistLocChordNode.getAttribute('Value'));

            j = j+1;

        end
    end
    k = 1;
    for i = 2:3:numXSecs
        % Get the airfoil name
        AFxsecNode = xsecNodes.item(i);
        AFxsecCurve = AFxsecNode.getElementsByTagName('XSecCurve').item(0);
        AFnode = AFxsecCurve.getElementsByTagName('FileAirfoil').item(0);
        AFNamenode = AFnode.getElementsByTagName('AirfoilName').item(0);
        AFName{k} = char(AFNamenode.getTextContent());
        k = k +1;
    end

    % To get the twist at the root, we need to query the frist xsecNode of wing
    % Extract Twist
    xsecNoderoot = xsecNodes.item(0);
    TwistNode = xsecNoderoot.getElementsByTagName('Twist').item(0);
    Twist = [str2double(TwistNode.getAttribute('Value')) Twist];

    % Get the eta based on Span
    TotalSpan = sum(Span);   % Total Span is the acutal semi-span calculated after adding all numbers
    %span array
    eta = cumsum(Span)/TotalSpan;

    % To get root incidence angle XForm should be queried
    xformNodesWingYRot = xFormNodeWing.getElementsByTagName('Y_Rotation').item(0);
    RootIncidence = str2double(xformNodesWingYRot.getAttribute('Value'));

    % The dihedral can be obtained using two ways, one directly giving diheral
    % input in VSP or by rotating the wing component by 90 deg along x-axis
    xformNodesWingXRot = xFormNodeWing.getElementsByTagName('X_Rotation').item(0);
    XRot = str2double(xformNodesWingXRot.getAttribute('Value'));
    Dihedral = Dihedral + XRot;

    % Capture rotation along z-axis as well
    xformNodesWingZRot = xFormNodeWing.getElementsByTagName('Z_Rotation').item(0);
    ZRot = str2double(xformNodesWingZRot.getAttribute('Value'));

    % rotation along y-axis
    xformNodesWingYRot = xFormNodeWing.getElementsByTagName('Y_Rotation').item(0);
    YRot = str2double(xformNodesWingYRot.getAttribute('Value'));

    %Location
    XLocNodesWing = xFormNodeWing.getElementsByTagName('X_Location').item(0);
    XLocWing = -str2double(XLocNodesWing.getAttribute('Value'))*conv_ft_m;

    YLocNodesWing = xFormNodeWing.getElementsByTagName('Y_Location').item(0);
    YLocWing = str2double(YLocNodesWing.getAttribute('Value'))*conv_ft_m;

    ZLocNodesWing = xFormNodeWing.getElementsByTagName('Z_Location').item(0);
    ZLocWing = -str2double(ZLocNodesWing.getAttribute('Value'))*conv_ft_m;

    % The directionality of the lifting surface should be evaluated based on
    % z-rotation
    % The default directionality of the lifting surface in VSP is 1 since y
    Wing.Directionality = 1;
    Wing.RefPtChordFrac = 0;
    Wing.Directionality(ZRot == 180 || ZRot == -180) = -1;
    Wing.RefPtChordFrac(Wing.Directionality == -1) = 1;
    Dihedral = Wing.Directionality.*Dihedral;
    Sweep = Wing.Directionality.*Sweep;


    % Calculate the taper ratio based on root chord and tip chord
    TR = TipChord / RootChord(1);

    % Get Control Surfaces Information
    SubSurfacesNode = GeomControlNode.getElementsByTagName('SubSurfaces').item(0);
    SubSurfaceNode = SubSurfacesNode.getElementsByTagName('SubSurface');
    if ~isempty(SubSurfaceNode.item(0))
        ControlSurfName = strings(0,1);
        j = 1;
        for k = 1:2:SubSurfaceNode.getLength()

            CS = SubSurfaceNode.item(k-1);
            Namenode = CS.getElementsByTagName('Name').item(0);
            CSName = char(Namenode.getTextContent());
            CSetanode = CS.getElementsByTagName('SS_Control').item(0);
            CSetastartnode = CSetanode.getElementsByTagName('EtaStart').item(0);
            CSetaendnode = CSetanode.getElementsByTagName('EtaEnd').item(0);
            Chordfracnode = CSetanode.getElementsByTagName('Length_C_Start').item(0);
            CSetastart = str2double(CSetastartnode.getAttribute('Value'));
            CSetaend = str2double(CSetaendnode.getAttribute('Value'));
            CSchordfrac = str2double(Chordfracnode.getAttribute('Value'));
            EtaStart(j) = CSetastart;
            EtaEnd(j) = CSetaend;
            ChordFrac(j) = CSchordfrac;
            ControlSurfName(j) = CSName;
            j = j + 1;
        end
        Wing.CS.eta = [EtaStart;EtaEnd];
        Wing.CS.Chordfrac = ChordFrac;
        Wing.CS.Name = ControlSurfName;
    end


    % Concatenate all in new structure called Wing
    Wing.Span = sum(Span);
    Wing.PlanformArea = sum(PlanformArea);
    Wing.AR = sum(Span)^2 / Wing.PlanformArea;
    Wing.Sweep = [Sweep(1) Sweep];
    Wing.SweepChordLoc = [SweepChordLoc(1) SweepChordLoc];
    Wing.Dihedral = [Dihedral(1) Dihedral];
    Wing.Twist = Twist;
    Wing.TR = [1, TR];
    Wing.eta = [0 eta];
    Wing.RootIncidence = RootIncidence;
    Wing.Name = Name ;
    Wing.AFName = AFName;
    Wing.Transform.Location = [XLocWing YLocWing ZLocWing];
    Wing.Transform.Rotation = [XRot YRot ZRot];
    Wing.Transform.Symm = SymmFlag;
    Wing.Type = TypeWing;
end

%% Propeller
if strcmp(TypeWing,'Propeller') == 1
    PropXFormNode = GeomNode.getElementsByTagName('XForm').item(0);
    PropGeomNode = GeomNode.getElementsByTagName('PropellerGeom').item(0);
    XSecSurfNode = PropGeomNode.getElementsByTagName('XSecSurf').item(0);
    XSecNode1 = XSecSurfNode.getElementsByTagName('XSec').item(0);
    XSecRadFraNode = XSecNode1.getElementsByTagName('XSec').item(0);
    RadFracstr = XSecRadFraNode.getElementsByTagName('RadiusFrac').item(0);
    RadFrac = str2double(RadFracstr.getAttribute('Value'));
    
    PropXLoc = PropXFormNode.getElementsByTagName('X_Location').item(0);
    XLoc = -str2double(PropXLoc.getAttribute('Value')) * conv_ft_m;

    PropYLoc = PropXFormNode.getElementsByTagName('Y_Location').item(0);
    YLoc = str2double(PropYLoc.getAttribute('Value')) * conv_ft_m;

    PropZLoc = PropXFormNode.getElementsByTagName('Z_Location').item(0);
    ZLoc = -str2double(PropZLoc.getAttribute('Value')) * conv_ft_m;

    PropXRot = PropXFormNode.getElementsByTagName('X_Rotation').item(0);
    XRot = str2double(PropXRot.getAttribute('Value'));
    PropXRelRot = PropXFormNode.getElementsByTagName('X_Rel_Rotation').item(0);
    XRelRot = str2double(PropXRelRot.getAttribute('Value'));

    PropYRot = PropXFormNode.getElementsByTagName('Y_Rotation').item(0);
    YRot = str2double(PropYRot.getAttribute('Value'));
    PropYRelRot = PropXFormNode.getElementsByTagName('Y_Rel_Rotation').item(0);
    YRelRot = str2double(PropYRelRot.getAttribute('Value'));

    PropZRot = PropXFormNode.getElementsByTagName('Z_Rotation').item(0);
    ZRot = str2double(PropZRot.getAttribute('Value'));
    PropZRelRot = PropXFormNode.getElementsByTagName('Z_Rel_Rotation').item(0);
    ZRelRot = str2double(PropZRelRot.getAttribute('Value'));

    PropLoc = [XLoc YLoc ZLoc];
    PropRot = [XRot YRot ZRot];
    PropRelRot = [XRelRot YRelRot ZRelRot];

    PropDesignNode = GeomNode.getElementsByTagName('Design').item(0);
    PropDiaNode = PropDesignNode.getElementsByTagName('Diameter').item(0);
    PropDia = str2double(PropDiaNode.getAttribute('Value')) * conv_ft_m;

    PropNBladesNode = PropDesignNode.getElementsByTagName('NumBlade').item(0);
    PropNBlades = str2double(PropNBladesNode.getAttribute('Value'));

    theta = YRot - 90;
    phi = 0; %XRot;

    Wing.Type = 'Propeller';
    Prop.Transform.Location = PropLoc;
    Prop.Transform.Rotation = PropRot;
    Prop.Diameter = PropDia;
    Prop.NBlades = PropNBlades;
    Prop.phi = phi;
    Prop.theta = theta;
    Prop.Name = Name;
    Wing.Prop = Prop;
end




%% Wheel
if strcmp(TypeWing,'BodyOfRevolution') == 1
    WP.Transform = [];
    % WP.Name = WheelPant;
    wheel.Name = Name;
    wheel.Transform = [];

    index = find(arrayfun(@(s) strcmp(s.name, Name), degenGeom));

    % Extract the coordinates of the points in x, y, and z axis.
    x = degenGeom(index(1)).surf.x ;
    y = degenGeom(index(1)).surf.y ;
    z = degenGeom(index(1)).surf.z ;

    for i = 0:1:GeomNodes.getLength()
        try
            GeomNode = GeomNodes.item(i);

            FusName = GeomNode.getElementsByTagName('Name').item(0);
            if strcmp(char(FusName.getTextContent),Name)
                % DesignNodes = GeomNode.getElementsByTagName('Design').item(0);
                XFormNodes = GeomNode.getElementsByTagName('XForm').item(0);
                SymmNodes = GeomNode.getElementsByTagName('Sym').item(0);
                % XSecCurveNodes = GeomNode.getElementsByTagName('XSecCurve').item(0);
                % XSecCurve1 = XSecCurveNodes.getElementsByTagName('XSecCurve').item(0);
                % XSecCurve2 = XSecCurveNodes.getElementsByTagName('XSecCurve').item(1);

            end
        catch
        end
    end

    try

        % Grab the location of Wheel
        XLocNode = XFormNodes.getElementsByTagName('X_Location').item(0);
        XLoc = -str2double(XLocNode.getAttribute('Value'))*conv_ft_m;

        YLocNode = XFormNodes.getElementsByTagName('Y_Location').item(0);
        YLoc = str2double(YLocNode.getAttribute('Value'))*conv_ft_m;

        ZLocNode = XFormNodes.getElementsByTagName('Z_Location').item(0);
        ZLoc = -str2double(ZLocNode.getAttribute('Value'))*conv_ft_m;

        % Grab the rotation along the axes
        XRotNode = XFormNodes.getElementsByTagName('X_Rotation').item(0);
        XRot = str2double(XRotNode.getAttribute('Value'));

        YRotNode = XFormNodes.getElementsByTagName('Y_Rotation').item(0);
        YRot = str2double(YRotNode.getAttribute('Value'));

        ZRotNode = XFormNodes.getElementsByTagName('Z_Rotation').item(0);
        ZRot = str2double(ZRotNode.getAttribute('Value'));

        SymmNode = SymmNodes.getElementsByTagName('Sym_Planar_Flag').item(0);
        Symm = str2double(SymmNode.getAttribute('Value'));

        wheel.Transform.Location = [XLoc YLoc ZLoc];
        wheel.Transform.Rotation = [XRot YRot ZRot];
        wheel.Transform.Symm = Symm;
        wheel.Diameter = (max(max(x)) - min(min(x))) * conv_ft_m;

        % % Use getFuselage lines to get the information of wheel
        % FusLines = getFuselageLines(filenameext,Wheel);
        % wheel.FusLines = FusLines;
        wheel.Span = (max(max(y)) - min(min(y))) * conv_ft_m;
        wheel.Chord = wheel.Diameter;
    catch
    end


    if isempty(wheel.Transform)
        wheel.HasWheel = 0;
    else
        wheel.HasWheel = 1;
    end

    if isempty(WP.Transform)
        WP.HasWheelPant = 0;
    else
        WP.HasWheelPant = 1;
    end

    Wing.Wheel = wheel;
    Wing.Name = Name;
    Wing.WP = WP;
    Wing.Type = 'Wheel';

end

