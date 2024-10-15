function Fus = getParamFus(Nodes,degenGeom,Name)
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

            %Symmetric Nodes
            FusSymmNodes = GeomNode.getElementsByTagName('Sym').item(0);
            FusSymPlanar = FusSymmNodes.getElementsByTagName('Sym_Planar_Flag').item(0);

            % Find all XSecSurf elements within Fuselage
            xsecSurfNodesFuse = FuselageNode.getElementsByTagName('XSecSurf');
            xsecSurfNodeFuse = xsecSurfNodesFuse.item(0);
            xsecNodesFuse = xsecSurfNodeFuse.getElementsByTagName('XSec');

            % Find the Design to extract length of Fuselage
            LenNodesFus = GeomNode.getElementsByTagName('Design');
            LenNodeFus = LenNodesFus.item(0);
            LenFus = LenNodeFus.getElementsByTagName('Length').item(0);
            LengthFus = str2double(LenFus.getAttribute('Value'));

        end
    catch
    end
end

%% Fuselage
% Extract Xloc, Yloc, and Zloc. The fuselage location in VSP is for the
% aftmost point of nosecone
FusXlocNode = FusXFormNodes.getElementsByTagName('X_Location').item(0);
FusXloc = -str2double(FusXlocNode.getAttribute('Value'))*conv_ft_m;

FusYlocNode = FusXFormNodes.getElementsByTagName('Y_Location').item(0);
FusYloc = str2double(FusYlocNode.getAttribute('Value'))*conv_ft_m;

FusZlocNode = FusXFormNodes.getElementsByTagName('Z_Location').item(0);
FusZloc = -str2double(FusZlocNode.getAttribute('Value'))*conv_ft_m;
Fusloc = [FusXloc FusYloc FusZloc];

FusXRotNode = FusXFormNodes.getElementsByTagName('X_Rotation').item(0);
FusXRot = -str2double(FusXRotNode.getAttribute('Value'));

FusYRotNode = FusXFormNodes.getElementsByTagName('Y_Rotation').item(0);
FusYRot = str2double(FusYRotNode.getAttribute('Value'));

FusZRotNode = FusXFormNodes.getElementsByTagName('Z_Rotation').item(0);
FusZRot = -str2double(FusZRotNode.getAttribute('Value'));

FusSym = str2double(FusSymPlanar.getAttribute('Value'));
FusRot = [FusXRot FusYRot FusZRot];


j = 1;
try
    for i = 3:3:xsecNodesFuse.getLength()-1
        % Extract XLocPercent as xsecNodesFus
        xsecNodesFus = xsecNodesFuse.item(i);
        SectionFS = xsecNodesFus.getElementsByTagName('XLocPercent').item(0);
        FS(j) = str2double(SectionFS.getAttribute('Value'));
        % Length(j) = LengthFus*FS(j)*conv_ft_m;
        
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
            if isempty(Heightparam)
                Heightparam = xseccurve.getElementsByTagName('Ellipse_Height').item(0);
                Widthparam = xseccurve.getElementsByTagName('Ellipse_Width').item(0);
            end
            if isempty(Heightparam)
                Heightparam = xseccurve.getElementsByTagName('Super_Height').item(0);
                Widthparam = xseccurve.getElementsByTagName('Super_Width').item(0);
            end
            if isempty(Heightparam)
                Heightparam = xseccurve.getElementsByTagName('RoundedRect_Height').item(0);
                Widthparam = xseccurve.getElementsByTagName('RoundedRect_Width').item(0);
            end
            
            Height(j) = str2double(Heightparam.getAttribute('Value'))*conv_ft_m;
            Width(j) = str2double(Widthparam.getAttribute('Value'))*conv_ft_m;
        else
            Diaparam = xseccurve.getElementsByTagName('Circle_Diameter').item(0);
            Diameter = str2double(Diaparam.getAttribute('Value'))*conv_ft_m;
            Height(j) = Diameter;
            Width(j) = Diameter;
        end

        j = j + 1;
    end
catch
end

% Extract the information of angle to get the number of cross-sections
xsecNodesSkin = xsecNodesFuse.item(0) ;
RSectionRAngle = xsecNodesSkin.getElementsByTagName('RightRAngle').item(0);
Skin.RightRAngle(j) = str2double(RSectionRAngle.getAttribute('Value'));

% Recompute FS if there are any 0 or 1 at second section or end-1 section
% respectively
if FS(1) == 0
    FS(1) = 0.01;
end
if FS(end-1) == 1
    FS(end-1) = 0.99;
end

Length = LengthFus*FS*conv_ft_m;


% Get fuselage lines and crosssection lines
[FusLines, CS] = getFuselageLines(degenGeom,Name,FusRot,FS);

% Concatenate every information into a structure called Fus
Fus.NumSec = length(Skin.RightRAngle);
Fus.FUS = FusLines ;
Fus.FS = FS;
Fus.CS = CS;
Fus.SectionLength = Length;
Fus.Length = Length(end);
Fus.Location = Fusloc;
Fus.Rotation = FusRot;
Fus.Symmetry = FusSym;
Fus.MaxHeight = max(Height);
Fus.MaxWidth = max(Width);
Fus.Name = Name ;
Fus.CrossSecType = CrossSecType;
