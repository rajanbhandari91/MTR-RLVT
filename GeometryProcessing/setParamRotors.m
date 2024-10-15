function setParamRotors(Nodes,Name,ParamOut)

GeomNodes = Nodes;

% We will change the Name to DiscName and HubName to get information from
% both disc and hub
DiscName = Name;
HubName = strrep(Name, 'disk', 'Hub');
BladeName = strrep(Name,'disk','blades');

isRotor = @(s) isfield(s, 'Name') && strcmp(s.Name, Name);
matchingFields = structfun(isRotor, ParamOut.Disk);
RotorFields = fieldnames(ParamOut.Disk);
matchingLSFields = RotorFields(matchingFields);

for i = 0:1:GeomNodes.getLength()
    try
        GeomNode = GeomNodes.item(i);

        RotorName = GeomNode.getElementsByTagName('Name').item(0);
        if strcmp(char(RotorName.getTextContent),DiscName)
            RotorXForm = GeomNode.getElementsByTagName('XForm');
            RotorXFormNodes = RotorXForm.item(0);

            % Find the Design to extract length of Fuselage
            DiaNodesRotor = GeomNode.getElementsByTagName('Design');
            DiaNodeRotor = DiaNodesRotor.item(0);
            DiaRotor = DiaNodeRotor.getElementsByTagName('Diameter').item(0);
            DiaOut = ParamOut.Disk.(matchingLSFields{1}).Diameter;
            DiaRotor.setAttribute('Value',num2str(DiaOut));
        end

        if strcmp(char(RotorName.getTextContent),HubName)
            HubDesignNodes = GeomNode.getElementsByTagName('Design');
            HubDesignNode = HubDesignNodes.item(0);
            LengthHub = HubDesignNode.getElementsByTagName('Length').item(0);
            LengthHub = str2double(LengthHub.getAttribute('Value'))*conv_ft_m;

            HubNodes = GeomNode.getElementsByTagName('FuselageGeom');
            HubNode = HubNodes.item(0);
            xsecSurfNodesHub = HubNode.getElementsByTagName('XSecSurf');
            xsecSurfNodeHub = xsecSurfNodesHub.item(0);
            xsecNodesHub = xsecSurfNodeHub.getElementsByTagName('XSec');

            j = 1;
            for k = 3:3:xsecNodesHub.getLength()-1
                xsecNodesHub2 = xsecNodesHub.item(k+2);
                xseccurve = xsecNodesHub2.getElementsByTagName('XSecCurve').item(0);

                xsecNodesHubCross = xsecNodesHub.item(k+2);
                xsecCurve = xsecNodesHubCross.getElementsByTagName('XSecCurve');
                xsecCurveType = xsecCurve.item(2);
                CrossSection = xsecCurveType.getElementsByTagName('Type').item(0);
                CrossSecType(j) = str2double(CrossSection.getTextContent());

                if CrossSecType(j) ~= 1
                    Heightparam = xseccurve.getElementsByTagName('Height').item(0);
                    Widthparam = xseccurve.getElementsByTagName('Width').item(0);
                    Height(j) = str2double(Heightparam.getAttribute('Value'))*conv_ft_m;
                    Width(j) = str2double(Widthparam.getAttribute('Value'))*conv_ft_m;
                else
                    Diaparam = xseccurve.getElementsByTagName('Circle_Diameter').item(0);
                    Diameter = str2double(Diaparam.getAttribute('Value'))*conv_ft_m;
                    Height(j) = Diameter;
                    Width(j) = Diameter;
                end
                j = j+1;
            end
        end

        if strcmp(char(RotorName.getTextContent),BladeName)
            BladeDesignNodes = GeomNode.getElementsByTagName('Design');
            BladeDesignNode = BladeDesignNodes.item(0);
            BladeXFormNodes = GeomNode.getElementsByTagName('XForm').item(0);
            XLocationNode = BladeXFormNodes.getElementsByTagName('X_Location').item(0);
            Location = ParamOut.Disk.(matchingLSFields{1}).Transform.Location;
            XLocationNode.setAttribute('Value',num2str(Location(1)));
            YLocationNode = BladeXFormNodes.getElementsByTagName('Y_Location').item(0);
            YLocationNode.setAttribute('Value',num2str(Location(2)));
            ZLocationNode = BladeXFormNodes.getElementsByTagName('Z_Location').item(0);
            ZLocationNode.setAttribute('Value',num2str(Location(3)));
            
            NBlades = BladeDesignNode.getElementsByTagName('NumBlade').item(0);
            NBladesOut = ParamOut.Disk.(matchingLSFields{1}).NBlades;
            NBlades.setAttribute('Value',num2str(NBladesOut));

            Diam = BladeDesignNode.getElementsByTagName('Diameter').item(0);
            DiamOut = ParamOut.Disk.(matchingLSFields{1}).Diameter;
            Diam.setAttribute('Value',num2str(DiamOut));

            
        end
    catch
    end
end

% Extract Xloc, Yloc, and Zloc.
RotorXlocNode = RotorXFormNodes.getElementsByTagName('X_Location').item(0);
RotorlocOut = ParamOut.Disk.(matchingLSFields{1}).Transform.Location;
RotorXlocNode.setAttribute('Value',num2str(RotorlocOut(1)));

RotorYlocNode = RotorXFormNodes.getElementsByTagName('Y_Location').item(0);
RotorYlocNode.setAttribute('Value',num2str(RotorlocOut(2)));

RotorZlocNode = RotorXFormNodes.getElementsByTagName('Z_Location').item(0);
RotorZlocNode.setAttribute('Value',num2str(RotorlocOut(3)));

% Extract XRot, YRot, and ZRot
RotorXRotNode = RotorXFormNodes.getElementsByTagName('X_Rotation').item(0);
RotorRotXOut = ParamOut.Disk.(matchingLSFields{1}).phi;
RotorXRotNode.setAttribute('Value',num2str(RotorRotXOut));

RotorYRotNode = RotorXFormNodes.getElementsByTagName('Y_Rotation').item(0);
RotorRotYOut = ParamOut.Disk.(matchingLSFields{1}).theta;
RotorYRotNode.setAttribute('Value',num2str(RotorRotYOut));


