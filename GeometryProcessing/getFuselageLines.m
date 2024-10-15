function [FUS_final, Cross] = getFuselageLines(degenGeom,targetName,Rotation,FusSt)

%load the .mat file generated after exporting from openVSP using degenGEOM
% run(filename);
index = find(arrayfun(@(s) strcmp(s.name, targetName), degenGeom));

% Extract the coordinates of the points in x, y, and z axis.
x = degenGeom(index(1)).surf.x ;
y = degenGeom(index(1)).surf.y ;
z = degenGeom(index(1)).surf.z ;

x_vec = x(:);
y_vec = y(:);
z_vec = z(:);

coords = [x_vec';y_vec';z_vec'];

phi = -Rotation(1);
theta = Rotation(2);
psi = -Rotation(3);

% Evaluate the rotation tensor
R1 = [1 0 0;...
    0 cosd(phi) sind(phi);...
    0 -sind(phi) cosd(phi)];

R2 = [cosd(theta) 0 -sind(theta);...
    0 1 0;...
    sind(theta) 0 cosd(theta)];

R3 = [cosd(psi) sind(psi) 0;...
    -sind(psi) cosd(psi) 0;...
    0 0 1];

R = R1*R2*R3;

new_coords = R * coords;

x = reshape(new_coords(1, :),size(x));
y = reshape(new_coords(2, :), size(y));
z = reshape(new_coords(3, :), size(z));


% Every row for X has repeated values. So, we can grab one element from
% each row. Grab them and offset the values so that X starts from 0.
X = x(:,1);
X = X - X(1);

%Check if there are any repeated x-coordinates like in circle
[unique_X, unique_indices] = unique(X, 'stable');
X = unique_X;

% From y axis grab the maximum value from each row to get the fuselage
% lines of the starboard side 
Y = (max(y, [], 2) - min(y, [], 2)) ./ 2;
Y = Y(unique_indices);

% From z axis we need to grab the top fuselage lines and the bottom
% fuselage lines
% Before doing this we need to offset the z-coordinates to start it from 0
Zmax = max(z, [], 2);
Zmax = Zmax(unique_indices);
Zmin = min(z, [], 2);
Zmin = Zmin(unique_indices);
Zref = mean([Zmax(1),Zmin(1)]);
Zmax = Zmax - Zref;
Zmin = Zmin - Zref ;
Ztop = Zmax;
Zbottom = Zmin;

% Replace all these variables to make fuselage lines to our standard
% fuselage lines terms
FS = X ;
Top = Ztop;
Bottom = Zbottom;
Side = Y ;

FS_norm = FS ./ FS(end);
Top_norm = Top ./ FS(end);
Bottom_norm = Bottom ./ FS(end);
Side_norm = Side ./ FS(end);

FUS = [FS_norm Top_norm Bottom_norm Side_norm];

% Append the FUS first and last row since they cannot be zero
% Modify the first row
FUS(1, 2) = FUS(1, 2) + (FUS(1, 2) <  0.00001) * 0.001;
FUS(1, 3) = FUS(1, 3) - (FUS(1, 3) <  0.00001) * 0.001;
FUS(1, 4) = FUS(1, 4) + (FUS(1, 4) <  0.00001) * 0.001;

% Modify the last row
FUS(end, 2) = FUS(end, 2) + (FUS(end, 2) <  0.00001) * 0.001;
FUS(end, 3) = FUS(end, 3) - (FUS(end, 3) <  0.00001) * 0.001;
FUS(end, 4) = FUS(end, 4) + (FUS(end, 4) <  0.00001) * 0.001;

% interpolate the fuselage lines for a standard FS_norm
FS_normstd = linspace(0,1,100);
FUS_std = interp1(FUS(:,1), [FUS(:,2),FUS(:,3),FUS(:,4)], FS_normstd,'linear');

FUS_final = [FS_normstd' , FUS_std];

% Get crosssection of the fuselage
Cross = getCrossSection(x_vec,y_vec,z_vec,Rotation,FusSt);

% % Find indices of rows with 1 in the first column
% indices_first_column = find(CS(:, 1) == 1);
% 
% % Find indices of rows with 1 in the first column and 0 in the second
% indices_first_column_second_column_zero = find(CS(:, 1) == 1 & CS(:, 2) == 0);
% 
% % If there's a row with 1 in the first column and 0 in the second, keep it and remove the others
% if ~isempty(indices_first_column_second_column_zero)
%     indices_to_remove = setdiff(indices_first_column, indices_first_column_second_column_zero);
%     CS(indices_to_remove, :) = [];
% else
%     % If no row with 1 in the first column and 0 in the second, remove the first occurrence
%     CS(indices_first_column(1), :) = [];
% end
% 
% % Find indices of rows with the duplicate value in the first column
% indices = find(CS(:, 1) == 1);
% 
% % If there are multiple occurrences, remove the first one
% if length(indices) > 1
%     CS(indices(1), :) = [];
% end


end








