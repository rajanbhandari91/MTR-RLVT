function Rotor = getParamRotors(Nodes,filename,Name)

conv_sqft_sqm = 0.092903;
conv_ft_m = 0.30488;

GeomNodes = Nodes;
filenameext = [filename,'_DegenGeom.m'];

% We will change the Name to DiscName and HubName to get information from
% both disc and hub
DiscName = Name;
HubName = strrep(Name, 'disk', 'Hub');
BladeName = strrep(Name,'disk','blades');

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
            DiaRotor = str2double(DiaRotor.getAttribute('Value'))*conv_ft_m;
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
            NBlades = BladeDesignNode.getElementsByTagName('NumBlade').item(0);
            NBlades = str2double(NBlades.getAttribute('Value'));
        end
    catch
    end
end

% Extract Xloc, Yloc, and Zloc.
RotorXlocNode = RotorXFormNodes.getElementsByTagName('X_Location').item(0);
RotorXloc = -str2double(RotorXlocNode.getAttribute('Value'))*conv_ft_m;

RotorYlocNode = RotorXFormNodes.getElementsByTagName('Y_Location').item(0);
RotorYloc = str2double(RotorYlocNode.getAttribute('Value'))*conv_ft_m;

RotorZlocNode = RotorXFormNodes.getElementsByTagName('Z_Location').item(0);
RotorZloc = -str2double(RotorZlocNode.getAttribute('Value'))*conv_ft_m;
Rotorloc = [RotorXloc RotorYloc RotorZloc];

% Extract XRot, YRot, and ZRot
RotorXRotNode = RotorXFormNodes.getElementsByTagName('X_Rotation').item(0);
RotorXRot = -str2double(RotorXRotNode.getAttribute('Value'));

RotorYRotNode = RotorXFormNodes.getElementsByTagName('Y_Rotation').item(0);
RotorYRot = str2double(RotorYRotNode.getAttribute('Value'));

RotorZRotNode = RotorXFormNodes.getElementsByTagName('Z_Rotation').item(0);
RotorZRot = -str2double(RotorZRotNode.getAttribute('Value'));
RotorRot = [RotorXRot RotorYRot RotorZRot];

% Set the theta and phi of rotors based on their rotation along the axis
theta = 90 - RotorYRot;
phi = RotorXRot;

% Caputure the blade profile
BladeProfile = getBladeProfile(filenameext,BladeName);

% Concatenate the extracted information into a structure named Rotor
Rotor.Diameter = DiaRotor;
Rotor.Location = Rotorloc;
% Rotor.HubLength = LengthHub;
Rotor.Name = DiscName;
Rotor.theta = theta;
Rotor.phi = phi;
Rotor.NBlades = NBlades;
% Rotor.HubDia = max(max(Height,Width));
Rotor.BladeProfile = BladeProfile;
