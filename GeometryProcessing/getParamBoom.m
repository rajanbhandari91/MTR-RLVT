function Boom = getParamBoom(Nodes,filename,Name)
GeomNodes = Nodes;
filenameext = [filename,'_DegenGeom.m'];

for i = 0:1:GeomNodes.getLength()
    try
        GeomNode = GeomNodes.item(i);

        FusName = GeomNode.getElementsByTagName('Name').item(0);
        if strcmp(char(FusName.getTextContent),'Rotor_1_Support')
            BoomNodes = GeomNode.getElementsByTagName('FuselageGeom');
            BoomNode = BoomNodes.item(0);
            BoomXForm = GeomNode.getElementsByTagName('XForm');
            BoomXFormNodes = BoomXForm.item(0);

            % Find the symmetry of the boom
            BoomSymm = GeomNode.getElementsByTagName('Sym');
            BoomSymmNodes = BoomSymm.item(0);

            % Find all XSecSurf elements within Boom
            xsecSurfNodesBoom = BoomNode.getElementsByTagName('XSecSurf');
            xsecSurfNodeBoom = xsecSurfNodesBoom.item(0);
            xsecNodesBoom = xsecSurfNodeBoom.getElementsByTagName('XSec');

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
BoomXlocNode = BoomXFormNodes.getElementsByTagName('X_Location').item(0);
BoomXloc = -str2double(BoomXlocNode.getAttribute('Value'));

BoomYlocNode = BoomXFormNodes.getElementsByTagName('Y_Location').item(0);
BoomYloc = str2double(BoomYlocNode.getAttribute('Value'));

BoomZlocNode = BoomXFormNodes.getElementsByTagName('Z_Location').item(0);
BoomZloc = -str2double(BoomZlocNode.getAttribute('Value'));
Boomloc = [BoomXloc BoomYloc BoomZloc];

SymmNode = BoomSymmNodes.getElementsByTagName('Sym_Planar_Flag').item(0);
Symm = str2double(SymmNode.getAttribute('Value'));

j = 1;
try
for i = 3:3:xsecNodesBoom.getLength()-1
    % Extract Width and Height as xsecNodesFus2
    xsecNodesBoom2 = xsecNodesBoom.item(i);
    xdelta = xsecNodesBoom2.getElementsByTagName('XLocPercent').item(0);
    XLocPercent(j) = str2double(xdelta.getAttribute('Value'));
    
    % Find out the type of cross-section in each section
    xsecNodesFusCross = xsecNodesBoom.item(i+2);
    xsecCurve = xsecNodesFusCross.getElementsByTagName('XSecCurve');
    xsecCurveType = xsecCurve.item(2);
    CrossSection = xsecCurveType.getElementsByTagName('Type').item(0);
    CrossSecType(j) = str2double(CrossSection.getTextContent());

    j = j + 1;
end
catch 
end

%Evaluate the length based on XLocPercent

% To get the number of stations for boom we can use array of Length and
% evaluate it
% FS = ;

% Extract the information of angle to get the number of cross-sections
xsecNodesSkin = xsecNodesBoom.item(0) ;
RSectionRAngle = xsecNodesSkin.getElementsByTagName('RightRAngle').item(0);
Skin.RightRAngle(j) = str2double(RSectionRAngle.getAttribute('Value'));

% Send the targeName to create Boomlines
targetName = 'Boom';

% Using the information of skinning make fuselage lines
FusLines = getFuselageLines(filenameext,targetName);

% Evaluate the maximum height and width of the boom using FusLines
Boom.MaxHeight = max(FusLines(:,2)*sum(Length) - FusLines(:,3)*sum(Length));
Boom.MaxWidth = 2 * max(FusLines(:,4)*sum(Length));

Boom.NumSec = length(Skin.RightRAngle);
Boom.FUS = FusLines ;
Boom.Location = Boomloc;
Boom.Symm = Symm;
Boom.FS = FS;
Boom.SectionLength = Length;
Boom.Length = sum(Length);
Boom.Location = Boomloc;

