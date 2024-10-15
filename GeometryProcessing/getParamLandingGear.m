function [LDG,WP,wheel] = getParamLandingGear(GeomNodes,filename, LDG,Wheel,WheelPant)

conv_sqft_sqm = 0.092903;
conv_ft_m = 0.30488;

LDG =getParamLiftingSurface(GeomNodes,LDG);
WP.Transform = [];
WP.Name = WheelPant;
wheel.Name = Wheel;
wheel.Transform = [];

filenameext = [filename,'_DegenGeom.m'];

%% Wheel
for i = 0:1:GeomNodes.getLength()
    try
        GeomNode = GeomNodes.item(i);

        FusName = GeomNode.getElementsByTagName('Name').item(0);
        if strcmp(char(FusName.getTextContent),Wheel)
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
    Diameter1 = str2double(Diameter1Node.getAttribute('Value'))*conv_ft_m;

    %Type of cross-sections
    CSNode = XSecCurve2.getElementsByTagName('Type').item(0);
    CSType = str2double(CSNode.getTextContent());

    Diameter2Node = XSecCurve1.getElementsByTagName('Circle_Diameter').item(0);
    Diameter2 = str2double(Diameter2Node.getAttribute('Value'))*conv_ft_m;

    Diameter = Diameter1 + Diameter2;

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

    wheel.Transform.Location = [XLoc YLoc ZLoc];
    wheel.Transform.Rotation = [XRot YRot ZRot];
    wheel.Diameter = Diameter;

    % % Use getFuselage lines to get the information of wheel
    % FusLines = getFuselageLines(filenameext,Wheel);
    % wheel.FusLines = FusLines;
    wheel.Span = Diameter2;
    wheel.Chord = Diameter;
catch
end


if isempty(wheel.Transform)
    LDG.HasWheel = 0;
else
    LDG.HasWheel = 1;
end

if isempty(WP.Transform)
    LDG.HasWheelPant = 0;
else
    LDG.HasWheelPant = 1;
end




end
