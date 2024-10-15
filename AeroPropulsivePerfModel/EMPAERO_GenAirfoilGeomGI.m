
function [Vehicle] = EMPAERO_GenAirfoilGeomGI(Vehicle)
% generate GIs for the airfoil coordinates
% [kunwar, 07.25.2023]
FieldNames = fieldnames(Vehicle.Geom);
for i = 1: length(FieldNames)
    if strcmp(Vehicle.Geom.(FieldNames{i}).Type,'liftingsurface')
        Vehicle.Geom.(FieldNames{i}).AirfoilGeom = EMPAERO_GenAirfoilGeom_embedded(Vehicle.Geom.(FieldNames{i}).AirfoilName{1}.name);
    end
end
% % Vehicle.Geom.RVS.AirfoilGeom =  EMPAERO_GenAirfoilGeom_embedded(Vehicle.Geom.RVS.AirfoilName{1}.name);
% Vehicle.Geom.TG.AirfoilGeom =  EMPAERO_GenAirfoilGeom_embedded(Vehicle.Geom.TG.AirfoilName{1}.name);
% Vehicle.Geom.LWinglet.AirfoilGeom = EMPAERO_GenAirfoilGeom_embedded(Vehicle.Geom.LWinglet.AirfoilName{1}.name);
% Vehicle.Geom.RWinglet.AirfoilGeom = EMPAERO_GenAirfoilGeom_embedded(Vehicle.Geom.RWinglet.AirfoilName{1}.name);

    function[AirfoilGeom] = EMPAERO_GenAirfoilGeom_embedded(airfoil_name)
        Name = [pwd,'\',airfoil_name,'.dat'];
        % Open the file
        fid = fopen(Name, 'r');

        % Skip the first line (text)
        fgetl(fid);

        % Read the remaining data as numeric values
        data = textscan(fid, '%f %f', 'CollectOutput', true);

        % Close the file
        fclose(fid);

        % Store the numeric data in the variable 'AF'
        AF = data{1};
        AF(:,3) = zeros(length(AF(:,1)),1);

        boundary = find(AF(:,1)==0);
        AF_xy_top = AF(1:boundary,:);
        AF_xy_bot = AF(boundary:end,:);

        % sort the rows of the data in ascending order based on  x coordinate
        AF_xy_top = sortrows(AF_xy_top);
        % bottom half is already sorted

        % GI is used control surface modeling in CalcSpanLoading.m
        AirfoilGeom.GI_AF_xy_top = griddedInterpolant(AF_xy_top(:,1),AF_xy_top(:,2));
        AirfoilGeom.GI_AF_xy_bot = griddedInterpolant(AF_xy_bot(:,1),AF_xy_bot(:,2));

        x = 0:0.1:1;
        y_top = AirfoilGeom.GI_AF_xy_top(x);
        y_bot = AirfoilGeom.GI_AF_xy_bot(x);
        h = y_top-y_bot;
        AirfoilGeom.maxthicknessloc = x(max(h) == h);
    end
end