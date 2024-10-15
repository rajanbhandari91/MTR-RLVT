function setParamLiftingSurface(GeomNodes,Name,ParamOut)

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

            % Total Span and Planform Area
            WingGeomNodes = GeomNode.getElementsByTagName('WingGeom');
            WingGeom = WingGeomNodes.item(0);

            % Find one more Geom node to get control surfaces definition
            GeomControlNode = GeomNode.getElementsByTagName('Geom').item(0);

            isLS = @(s) isfield(s, 'Name') && strcmp(s.Name, Name);
            matchingFields = structfun(isLS, ParamOut.LS);
            LSFields = fieldnames(ParamOut.LS);
            matchingLSFields = LSFields(matchingFields);

        end
    catch
    end
end

if strcmp(TypeWing,'Wing') == 1
    xsecNodes = xsecSurfNodeWing.getElementsByTagName('XSec');
    numXSecs = xsecNodes.getLength();

    j = 1;
    for i = 3:3:numXSecs
        if i < numXSecs-1
            xsecNode = xsecNodes.item(i);
            
            % Area
            AreaNode = xsecNode.getElementsByTagName('Area').item(0);
            AreaOut = ParamOut.LS.(matchingLSFields{1}).SectArea;
            AreaNode.setAttribute('Value',num2str(AreaOut(j)));

            % Aspect Ratio
            ARNode = xsecNode.getElementsByTagName('Aspect').item(0);
            AROut = ParamOut.LS.(matchingLSFields{1}).SectAR;
            ARNode.setAttribute('Value',num2str(AROut(j)));

            % Span
            spanNode = xsecNode.getElementsByTagName('Span').item(0);
            spanOut = ParamOut.LS.(matchingLSFields{1}).SectSpan;
            spanNode.setAttribute('Value',num2str(spanOut(j)));

            % Taper Ratio
            TRNode = xsecNode.getElementsByTagName('Taper').item(0);
            TROut = ParamOut.LS.(matchingLSFields{1}).TR;
            TRNode.setAttribute('Value',num2str(TROut(j)));

            % Root Chord
            RCNode = xsecNode.getElementsByTagName('Root_Chord').item(0);
            RCOut = ParamOut.LS.(matchingLSFields{1}).Chord;
            RCNode.setAttribute('Value',num2str(RCOut(j)));

            % Tip Chord
            TCNode = xsecNode.getElementsByTagName('Tip_Chord').item(0);
            TCOut = (ParamOut.LS.(matchingLSFields{1}).Chord(j) + ParamOut.LS.(matchingLSFields{1}).Chord(j+1)) * 0.5;
            TCNode.setAttribute('Value',num2str(TCOut));

            % Average Chord
            AvgCNode = xsecNode.getElementsByTagName('Avg_Chord').item(0);
            AvgCOut = ParamOut.LS.(matchingLSFields{1}).Chord;
            AvgCNode.setAttribute('Value',num2str(AvgCOut(j+1)));

            % Sweep
            SweepNode = xsecNode.getElementsByTagName('Sweep').item(0);
            SweepOut = ParamOut.LS.(matchingLSFields{1}).Sweep;
            SweepNode.setAttribute('Value',num2str(SweepOut(j+1)));

            % Extract Location along chord
            SweepChordLocNode = xsecNode.getElementsByTagName('Sweep_Location').item(0);
            SweepChordOut = ParamOut.LS.(matchingLSFields{1}).SweepChordLoc;
            SweepChordLocNode.setAttribute('Value',num2str(SweepChordOut(j+1)));

            % Dihedral
            DihedralNode = xsecNode.getElementsByTagName('Dihedral').item(0);
            DihedralOut = ParamOut.LS.(matchingLSFields{1}).Dihedral;
            DihedralNode.setAttribute('Value',num2str(DihedralOut(j+1)));

            % Twist
            TwistNode = xsecNode.getElementsByTagName('Twist').item(0);
            TwistOut = ParamOut.LS.(matchingLSFields{1}).Twist;
            TwistNode.setAttribute('Value',num2str(TwistOut(j+1)));

            j = j+1;

        end
    end

    % To get the twist at the root, we need to query the first xsecNode of wing
    % Extract Twist
    xsecNoderoot = xsecNodes.item(0);
    TwistNode = xsecNoderoot.getElementsByTagName('Twist').item(0);
    TwistOut = ParamOut.LS.(matchingLSFields{1}).Twist;
    TwistNode.setAttribute('Value',num2str(TwistOut(1)));

    % Span, Area, and AR
    SpanNode = WingGeom.getElementsByTagName('TotalSpan').item(0);
    if ParamOut.LS.(matchingLSFields{1}).Transform.Symm == 2
        SpanOut = ParamOut.LS.(matchingLSFields{1}).Span*2;
    else
        SpanOut = ParamOut.LS.(matchingLSFields{1}).Span;
    end
    SpanNode.setAttribute('Value',num2str(SpanOut));

    AreaNode = WingGeom.getElementsByTagName('TotalArea').item(0);
    if ParamOut.LS.(matchingLSFields{1}).Transform.Symm == 2
        AreaOut = ParamOut.LS.(matchingLSFields{1}).PlanformArea*2;
    else
        AreaOut = ParamOut.LS.(matchingLSFields{1}).PlanformArea;
    end
    AreaNode.setAttribute('Value',num2str(AreaOut));
    %
    ARNode = WingGeom.getElementsByTagName('TotalAR').item(0);
    if ParamOut.LS.(matchingLSFields{1}).Transform.Symm == 2
        AROut = ParamOut.LS.(matchingLSFields{1}).AR*2;
    else
        AROut = ParamOut.LS.(matchingLSFields{1}).AR;
    end
    ARNode.setAttribute('Value',num2str(AROut));


    % To get root incidence angle XForm should be queried
    xformNodesWingYRot = xFormNodeWing.getElementsByTagName('Y_Rotation').item(0);
    RotOut = ParamOut.LS.(matchingLSFields{1}).Transform.Rotation;
    xformNodesWingYRot.setAttribute('Value',num2str(RotOut(2)));

    % The dihedral can be obtained using two ways, one directly giving diheral
    % input in VSP or by rotating the wing component by 90 deg along x-axis
    xformNodesWingXRot = xFormNodeWing.getElementsByTagName('X_Rotation').item(0);
    xformNodesWingXRot.setAttribute('Value',num2str(RotOut(1)));

    % Capture rotation along z-axis as well
    xformNodesWingZRot = xFormNodeWing.getElementsByTagName('Z_Rotation').item(0);
    xformNodesWingZRot.setAttribute('Value',num2str(RotOut(3)));

    %Location
    AbsRelNode = xFormNodeWing.getElementsByTagName('Abs_Or_Relitive_flag').item(0);
    AbsFlag = 0 ;                                                           % 0 for absolute and 1 for relative
    AbsRelNode.setAttribute('Value',num2str(AbsFlag))

    XLocNodesWing = xFormNodeWing.getElementsByTagName('X_Location').item(0);
    LocOut = ParamOut.LS.(matchingLSFields{1}).Transform.Location;
    XLocNodesWing.setAttribute('Value',num2str(LocOut(1)));

    YLocNodesWing = xFormNodeWing.getElementsByTagName('Y_Location').item(0);
    YLocNodesWing.setAttribute('Value',num2str(LocOut(2)));

    ZLocNodesWing = xFormNodeWing.getElementsByTagName('Z_Location').item(0);
    ZLocNodesWing.setAttribute('Value',num2str(LocOut(3)));
end

if strcmp(TypeWing,'BodyOfRevolution') == 1
    isLS = @(s) isfield(s, 'Type') && strcmp(s.Type, 'Wheel');
    matchingFields = structfun(isLS, ParamOut.LS);
    LSFields = fieldnames(ParamOut.LS);
    matchingLSFields = LSFields(matchingFields);

    for k = 1: length(matchingLSFields)
        if strcmp(ParamOut.LS.(matchingLSFields{k}).Name,Name)
            %% Wheel
            for i = 0:1:GeomNodes.getLength()
                try
                    GeomNode = GeomNodes.item(i);

                    FusName = GeomNode.getElementsByTagName('Name').item(0);
                    if strcmp(char(FusName.getTextContent),Name)
                        DesignNodes = GeomNode.getElementsByTagName('Design').item(0);
                        XFormNodes = GeomNode.getElementsByTagName('XForm').item(0);
                        XSecCurveNodes = GeomNode.getElementsByTagName('XSecCurve').item(0);
                        XSecCurve1 = XSecCurveNodes.getElementsByTagName('XSecCurve').item(0);
                        XSecCurve2 = XSecCurveNodes.getElementsByTagName('XSecCurve').item(1);

                    end
                catch
                end
            end

            try
                Diameter1Node = DesignNodes.getElementsByTagName('Diameter').item(0);
                Diam1 = ParamOut.LS.(matchingLSFields{k}).Diameter - ParamOut.LS.(matchingLSFields{k}).Span ;
                Diam2 = ParamOut.LS.(matchingLSFields{k}).Span ;
                Diameter1Node.setAttribute('Value',num2str(Diam1(1)));

                Diameter2Node = XSecCurve1.getElementsByTagName('Circle_Diameter').item(0);
                Diameter2Node.setAttribute('Value',num2str(Diam2(1)));

                % Grab the location of Wheel
                XLocNode = XFormNodes.getElementsByTagName('X_Location').item(0);
                LocOut = ParamOut.LS.(matchingLSFields{k}).Transform.Location;
                XLocNode.setAttribute('Value',num2str(LocOut(1)));

                YLocNode = XFormNodes.getElementsByTagName('Y_Location').item(0);
                YLocNode.setAttribute('Value',num2str(LocOut(2)));

                ZLocNode = XFormNodes.getElementsByTagName('Z_Location').item(0);
                ZLocNode.setAttribute('Value',num2str(LocOut(3)));

                % Grab the rotation along the axes
                XRotNode = XFormNodes.getElementsByTagName('X_Rotation').item(0);
                RotOut = ParamOut.LS.(matchingLSFields{k}).Transform.Rotation;
                XRotNode.setAttribute('Value',num2str(RotOut(1)));

                YRotNode = XFormNodes.getElementsByTagName('Y_Rotation').item(0);
                YRotNode.setAttribute('Value',num2str(RotOut(2)));

                ZRotNode = XFormNodes.getElementsByTagName('Z_Rotation').item(0);
                ZRotNode.setAttribute('Value',num2str(RotOut(3)));
            catch
            end
        end
    end

end

