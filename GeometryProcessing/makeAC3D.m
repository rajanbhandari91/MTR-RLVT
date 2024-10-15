function [VertColl,VertColl_FSBLWL] = makeAC3D(geom, filename, mats,PlotMode,TrackPts,ac3dFLAG)
VertColl = [];
PlotMode = 0;

if PlotMode==1

    axis equal
    hold on
    box on
    %     xlabel('FUSELAGE STATION (inch)')
    %     ylabel('BUTTOCK LINE (inch)')
    %     zlabel('WATER LINE (inch)')

    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    set(gca,'ZDir','reverse')
    set(gca,'YDir','reverse')
    %view([-1,0,0])
    view([-0.3,-0.3,0.3])
    %view([0,1,0])
end


Objects = fieldnames(geom);
objCount = length(Objects);



[~,numMats] = size(mats);
fh = fopen(filename, 'w');
fprintf(fh, 'AC3Db');
% for n=1:numMats
%     fprintf(fh,'\nMATERIAL "%s" rgb %f %f %f  amb %f %f %f  emis %f %f %f  spec %f %f %f  shi %0.0f  trans %0.0f', mats(n).Name, mats(n).RGB, mats(n).Amb, mats(n).Emis,mats(n).Spec,mats(n).Shi,mats(n).Trans);
% end

fprintf(fh, '\nOBJECT WORLD\nkids %d',objCount);

for n=1:objCount

    VertColl = [VertColl;geom.(Objects{n}).Vertices];
end

UnitConv = 39.3701; % meter to inch
% =1 for plotting in meters. Else, = appropriate conversion factor
% from meters to unit chosen


% find the forward-most point
FwdMostPt = max(VertColl(:,1));

VertColl_FSBLWL(:,1) = - UnitConv*(VertColl(:,1)-FwdMostPt);
VertColl_FSBLWL(:,2) = UnitConv*VertColl(:,2);
VertColl_FSBLWL(:,3) = - UnitConv*VertColl(:,3);

%TrackPts = [geom.Seat.SRP];
if ~isempty(TrackPts)
    TrackPts_FSBLWL(:,1) = - UnitConv*(TrackPts(:,1)-FwdMostPt);
    TrackPts_FSBLWL(:,2) = UnitConv*TrackPts(:,2);
    TrackPts_FSBLWL(:,3) = - UnitConv*TrackPts(:,3);

end


for n=1:objCount


    Verts = geom.(Objects{n}).Vertices;

    xvert = - UnitConv*(Verts(:,1)-FwdMostPt);
    yvert = UnitConv*Verts(:,2);
    zvert = - UnitConv*Verts(:,3);

    geom.(Objects{n}).Vertices_FSBLWL = [xvert,yvert,zvert];

end

for n=1:objCount
    ObjectName = Objects{n};


    plotObjectGroup(fh,geom.(Objects{n}), 1,ObjectName,PlotMode,ac3dFLAG);
    %VertColl = [VertColl;geom.(Objects{n}).Vertices];
end
fclose(fh);

if ~isempty(TrackPts)
    scatter3(TrackPts_FSBLWL(:,1),TrackPts_FSBLWL(:,2),TrackPts_FSBLWL(:,3),'filled');
end
end




function plotObjectGroup(fh,parameters, mat,ObjectName,PlotMode,ac3dFLAG)
%[stnCount,~,~] = size(parameters)
%fprintf(fh, '\nOBJECT group\nkids %d',1);


Vertices = parameters.Vertices;
Vertices_FSBLWL = parameters.Vertices_FSBLWL;
[numvert,~] = size(Vertices);



fprintf(fh, '\nOBJECT poly');
fprintf(fh, '\nname "%s"',ObjectName);
if isfield(parameters,'Texture')
    fprintf(fh, '\ntexture "%s"',parameters.Texture);
    fprintf(fh, '\ntexrep 1.0 1.0');
end
fprintf(fh,'\ncrease 91.0');
fprintf(fh, '\nnumvert %d',numvert);
for m=1:numvert
    % put a negative sign in front of the x-coordinate and y-coordinate
    % then, swap the y and z coordinates
    if ac3dFLAG == 1
        x = -Vertices(m,1);
        y = -Vertices(m,3);
        z = -Vertices(m,2);
    elseif ac3dFLAG == 2
        x = -Vertices(m,1);
        y = -Vertices(m,2);
        z = -Vertices(m,3);
    else
    end
    % was all minus
    fprintf(fh,'\n%f %f %f', x, y, z);
end


%plotCap(fh,0:2:(pntCount*2-1), mat);




if ~isfield(parameters,'Material')
    Material = 0;
else
    Material = parameters.Material;
end

if ~isfield(parameters,'RGB')
    parameters.RGB = [1 1 1];
end


Surfaces = parameters.Surfaces;
[nsurf,ncol] = size(Surfaces);

fprintf(fh,'\nnumsurf %d',nsurf);
for n=1:nsurf

    fprintf(fh,'\nSURF 0x20\nmat %d\nrefs %d',Material,ncol);

    for j = 1:ncol
        fprintf(fh, '\n%d 0 0',Surfaces(n,j)-1);
    end
end

fprintf(fh,'\nkids 0');

%mat.RGB = [1,0,0];
if PlotMode==1
    %     fprintf('\n Now plotting...')
    for i = 1:1:nsurf
        pt = Vertices(Surfaces(i,:),:);
        %rgb = mat.RGB;
        rgb = [1 1 1];
        if isempty(rgb)
            rgb = [1 1 1];
        end

        fill3(pt(:,1),pt(:,2),pt(:,3),rgb);
        %alpha(0.4)
    end


end

end




%
% function plotObject(fh,parameters, mat)
% [pntCount,~] = size(parameters)
%
%
% Vertices = parameters.Vertices;
% [numvert,~] = size(Vertices)
%
% % if PlotMode==1
% %     fill3(Vertices(:,1),Vertices(:,2),Vertices(:,3),parameters.Material)
% % end
%
% fprintf(fh, '\nOBJECT poly\nnumvert %d',numvert);
% for m=1:numvert
%     fprintf(fh,'\n%f %f %f', -Vertices(m,1), Vertices(m,2), Vertices(m,3));
%
% end
%
%
% %plotCap(fh,0:2:(pntCount*2-1), mat);
% Material = parameters.Material;
% Surfaces = parameters.Surfaces;
% [nsurf,ncol] = size(Surfaces);
%
% fprintf(fh,'\nnumsurf %d',nsurf);
% for n=1:nsurf
%
%     fprintf(fh,'\nSURF 0x20\nmat %d\nrefs %d',Material,ncol);
%
%     for j = 1:ncol
%         fprintf(fh, '\n%d 0 0',Surfaces(n,j)-1);
%     end
% end
%
% fprintf(fh,'\nkids 0');
% end
%
%




% %Plots surfaces to close open surfaces
% function plotCap(fh,parameters, mat)
% [~,rows] = size(parameters);
% fprintf(fh,'\nSURF 0x20\nmat %d\nrefs %d',mat,rows);
% for n=parameters
%     fprintf(fh, '\n%d 0 0',n);
% end
% end
%
% function plotSurf(fh, parameters, mat)
% [~,rows] = size(parameters);
% fprintf(fh,'\nSURF 0x20\nmat %d\nrefs %d',mat,rows);
% for n=parameters
%     fprintf(fh, '\n%d 0 0',n);
% end
% end

%function [aero_struct] = structure (parameters)
%
%    aero_struct.fuselage = fuselage;
%    aero_struct.
%
%end