function setParamFus(Nodes,Name,ParamOut)
GeomNodes = Nodes;
conv_ft_m = 0.30488;

for i = 0:1:GeomNodes.getLength()
    try
        GeomNode = GeomNodes.item(i);

        FusName = GeomNode.getElementsByTagName('Name').item(0);
        if strcmp(char(FusName.getTextContent),Name)
            FuselageNodes = GeomNode.getElementsByTagName('FuselageGeom');
            FuselageNode = FuselageNodes.item(0);
            FusXForm = GeomNode.getElementsByTagName('XForm');
            FusXFormNodes = FusXForm.item(0);

            % Find all XSecSurf elements within Fuselage
            xsecSurfNodesFuse = FuselageNode.getElementsByTagName('XSecSurf');
            xsecSurfNodeFuse = xsecSurfNodesFuse.item(0);
            xsecNodesFuse = xsecSurfNodeFuse.getElementsByTagName('XSec');

            % Find the Design to extract length of Fuselage
            LenNodesFus = GeomNode.getElementsByTagName('Design');
            LenNodeFus = LenNodesFus.item(0);
            LenFus = LenNodeFus.getElementsByTagName('Length').item(0);
            isFuselage = @(s) isfield(s, 'Name') && strcmp(s.Name, Name);
            matchingFields = structfun(isFuselage, ParamOut.Fus);
            fusFields = fieldnames(ParamOut.Fus);
            matchingFusFields = fusFields(matchingFields);
            LenOut = ParamOut.Fus.(matchingFusFields{1}).Length;
            LenFus.setAttribute('Value', num2str(LenOut));

        end
    catch
    end
end

%% Fuselage
AbsRelNode = FusXFormNodes.getElementsByTagName('Abs_Or_Relitive_flag').item(0);
AbsFlag = 0 ;                                                           % 0 for absolute and 1 for relative
AbsRelNode.setAttribute('Value',num2str(AbsFlag))

FusXlocNode = FusXFormNodes.getElementsByTagName('X_Location').item(0);
FuslocOut = ParamOut.Fus.(matchingFusFields{1}).Transform.Location;
FusXlocNode.setAttribute('Value',num2str(FuslocOut(1)));

FusYlocNode = FusXFormNodes.getElementsByTagName('Y_Location').item(0);
FusYlocNode.setAttribute('Value',num2str(FuslocOut(2)));

FusZlocNode = FusXFormNodes.getElementsByTagName('Z_Location').item(0);
FusZlocNode.setAttribute('Value',num2str(FuslocOut(3)));

FusXRotNode = FusXFormNodes.getElementsByTagName('X_Rotation').item(0);
FusRotOut = ParamOut.Fus.(matchingFusFields{1}).Transform.Location;

FusYRotNode = FusXFormNodes.getElementsByTagName('Y_Rotation').item(0);
FusYRot = str2double(FusYRotNode.getAttribute('Value'));

FusZRotNode = FusXFormNodes.getElementsByTagName('Z_Rotation').item(0);
FusZRot = -str2double(FusZRotNode.getAttribute('Value'));
FusRot = [FusXRot FusYRot FusZRot];



j = 1;
try
    for i = 3:3:xsecNodesFuse.getLength()-1
        % Extract XLocPercent as xsecNodesFus
        xsecNodesFus = xsecNodesFuse.item(i);
        SectionFS = xsecNodesFus.getElementsByTagName('XLocPercent').item(0);
        FS(j) = str2double(SectionFS.getAttribute('Value'));

        % Find out the type of cross-section in each section
        xsecNodesFusCross = xsecNodesFuse.item(i+2);
        xsecCurve = xsecNodesFusCross.getElementsByTagName('XSecCurve');
        xsecCurveType = xsecCurve.item(2);
        CrossSection = xsecCurveType.getElementsByTagName('Type').item(0);
        CrossSecType(j) = str2double(CrossSection.getTextContent());

        % Extract Width and Height as xsecNodesFus2. We can extract height and
        % width if the cross-section type is not a circle.
        xsecNodesFus2 = xsecNodesFuse.item(i+2);
        xseccurve = xsecNodesFus2.getElementsByTagName('XSecCurve').item(0);

        if CrossSecType(j) ~= 1
            Heightparam = xseccurve.getElementsByTagName('Height').item(0);
            Widthparam = xseccurve.getElementsByTagName('Width').item(0);
            HtOut = ParamOut.Fus.(matchingFusFields{1}).Height(j);
            WidthOut = ParamOut.Fus.(matchingFusFields{1}).Width(j);
            Heightparam.setAttribute('Value', num2str(HtOut));
            Widthparam.setAttribute('Value', num2str(WidthOut));
        else
            Diaparam = xseccurve.getElementsByTagName('Circle_Diameter').item(0);
            HtOut = ParamOut.Fus.(matchingFusFields{1}).Height(j);
            Diaparam.setAttribute('Value', num2str(HtOut));
        end

        j = j + 1;
    end
catch
end



