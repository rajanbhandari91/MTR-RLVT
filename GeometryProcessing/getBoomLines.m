function FUS_final = getBoomLines(filename,targetName)

%load the .mat file generated after exporting from openVSP using degenGEOM
run(filename);
index = find(arrayfun(@(s) strcmp(s.name, targetName), degenGeom));

% Extract the coordinates of the points in x, y, and z axis.
% Extracting information from one index should suffice to create fuselage
% lines
x = degenGeom(index(1)).surf.x ;
y = degenGeom(index(1)).surf.y ;
z = degenGeom(index(1)).surf.z ;

% Every row for X has repeated values. So, we can grab one element from
% each row. Grab them and offset the values so that X starts from 0.
X = x(:,1);
X = X - X(1);

% From y axis grab the maximum value from each row to get the fuselage
% lines of the starboard side 
Y = (max(y, [], 2) - min(y, [], 2)) ./ 2;

% From z axis we need to grab the top fuselage lines and the bottom
% fuselage lines
Ztop = (max(z, [], 2) - min(z, [], 2));
Zbottom = (min(z, [], 2) - max(z, [], 2));

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


% x_normstd = linspace(0,1,25)';
% % Get the number of sections in fuselage
% NumSec = length(Skin.TopLStrength);
% 
% % The number of curves are NumSec -1 . eg. If there are five sections,
% % there should be 4 curves to represent those five sections
% 
% % Form the spline using Bezier Curves
% % The first point is always (0,0), which is also the starting point of
% % fuselage nosecone
% 
% %% Concatenate all the points in one matrix. First we need to do for top
% % view
% x_coords = [0,Length];
% z_coords = [0, 0.5.*Width];
% Top_coords = [x_coords;z_coords]; 
% 
% for i = 1: NumSec-1
%     p1 = [Top_coords(1,i); Top_coords(2,i)];
%     p2 = [Top_coords(1,i+1); Top_coords(2,i+1)];
%     angle1 = Skin.RightRAngle(i);
%     angle2 = Skin.RightLAngle(i+1);
%     strength1 = 1/3; %Skin.RightRStrength(i);
%     strength2 = 1/3; %Skin.RightLStrength(i+1);
% 
%     % Check for valid input (column vectors with 2 elements)
%     if size(p1, 1) ~= 2 || size(p1, 2) ~= 1 || ...
%             size(p2, 1) ~= 2 || size(p2, 2) ~= 1
%         error('p1 and p2 must be 2D column vectors');
%     end
% 
%     % Convert angles to radians
%     angle1 = deg2rad(angle1);
%     angle2 = deg2rad(angle2);
% 
%     % Calculate direction vectors based on angles
%     dir1 = [cos(angle1); sin(angle1)];
%     dir2 = [cos(angle2); sin(angle2)];
% 
%     % Calculate control points based on direction vectors and strengths
%     control1 = p1 + strength1 * dir1;
%     control2 = p2 - strength2 * dir2;
% 
%     % Define the parametric variable (usually denoted by 't')
%     t = linspace(0, 1, 20); % Adjust number of points for desired resolution
% 
%     % Calculate Bezier curve points
%     BezierCurve = (1-t).^3 .* p1 + ...
%         3*(1-t).^2 .* t .* control1 + ...
%         3*(1-t) .* t.^2 .* control2 + ...
%         t.^3 .* p2;
% 
%     % Extract x-y coordinates
%     if i == 1
%         x = BezierCurve(1,:)' ;
%         zs = BezierCurve(2,:)' ;
%     else
%         x = [x ; BezierCurve(1,2:end)'];
%         zs = [zs ; BezierCurve(2,2:end)'];
%     end
% 
% end
% 
% FS = x;
% Side = zs;
% 
% %% Do the same for side view for top surface and bottom surface
% x_coords = [0,Length];
% z_coords = [0, 0.5.*Height];
% Top_coords = [x_coords;z_coords]; 
% 
% for i = 1: NumSec-1
%     p1 = [Top_coords(1,i); Top_coords(2,i)];
%     p2 = [Top_coords(1,i+1); Top_coords(2,i+1)];
%     angle1 = Skin.TopRAngle(i);
%     angle2 = Skin.TopLAngle(i+1);
%     strength1 = Skin.TopRStrength(i);
%     strength2 = Skin.TopLStrength(i+1);
% 
%     % Check for valid input (column vectors with 2 elements)
%     if size(p1, 1) ~= 2 || size(p1, 2) ~= 1 || ...
%             size(p2, 1) ~= 2 || size(p2, 2) ~= 1
%         error('p1 and p2 must be 2D column vectors');
%     end
% 
%     % Convert angles to radians
%     angle1 = deg2rad(angle1);
%     angle2 = deg2rad(angle2);
% 
%     % Calculate direction vectors based on angles
%     dir1 = [cos(angle1); sin(angle1)];
%     dir2 = [cos(angle2); sin(angle2)];
% 
%     % Calculate control points based on direction vectors and strengths
%     control1 = p1 + strength1 * dir1;
%     control2 = p2 - strength2 * dir2;
% 
%     % Define the parametric variable (usually denoted by 't')
%     t = linspace(0, 1, 20); % Adjust number of points for desired resolution
% 
%     % Calculate Bezier curve points
%     BezierCurve = (1-t).^3 .* p1 + ...
%         3*(1-t).^2 .* t .* control1 + ...
%         3*(1-t) .* t.^2 .* control2 + ...
%         t.^3 .* p2;
% 
%     % Extract x-y coordinates
%     if i == 1
%         x = BezierCurve(1,:)' ;
%         zs = BezierCurve(2,:)' ;
%     else
%         x = [x ; BezierCurve(1,2:end)'];
%         zs = [zs ; BezierCurve(2,2:end)'];
%     end
% 
% end
% 
% Top = zs ;
% 
% %% Repeat the same for bottom
% x_coords = [0,Length];
% z_coords = [0, -0.5.*Height];
% Top_coords = [x_coords;z_coords]; 
% 
% for i = 1: NumSec-1
%     p1 = [Top_coords(1,i); Top_coords(2,i)];
%     p2 = [Top_coords(1,i+1); Top_coords(2,i+1)];
%     angle1 = -Skin.BottomRAngle(i);
%     angle2 = -Skin.BottomLAngle(i+1);
%     strength1 = Skin.BottomRStrength(i);
%     strength2 = Skin.BottomLStrength(i+1);
% 
%     % Check for valid input (column vectors with 2 elements)
%     if size(p1, 1) ~= 2 || size(p1, 2) ~= 1 || ...
%             size(p2, 1) ~= 2 || size(p2, 2) ~= 1
%         error('p1 and p2 must be 2D column vectors');
%     end
% 
%     % Convert angles to radians
%     angle1 = deg2rad(angle1);
%     angle2 = deg2rad(angle2);
% 
%     % Calculate direction vectors based on angles
%     dir1 = [cos(angle1); sin(angle1)];
%     dir2 = [cos(angle2); sin(angle2)];
% 
%     % Calculate control points based on direction vectors and strengths
%     control1 = p1 + strength1 * dir1;
%     control2 = p2 - strength2 * dir2;
% 
%     % Define the parametric variable (usually denoted by 't')
%     t = linspace(0, 1, 20); % Adjust number of points for desired resolution
% 
%     % Calculate Bezier curve points
%     BezierCurve = (1-t).^3 .* p1 + ...
%         3*(1-t).^2 .* t .* control1 + ...
%         3*(1-t) .* t.^2 .* control2 + ...
%         t.^3 .* p2;
% 
%     % Extract x-y coordinates
%     if i == 1
%         x = BezierCurve(1,:)' ;
%         zs = BezierCurve(2,:)' ;
%     else
%         x = [x ; BezierCurve(1,2:end)'];
%         zs = [zs ; BezierCurve(2,2:end)'];
%     end
% 
% end
% 
% Bottom = zs ;
% 
% %% Using this information make FUS that is used in PEACE as fuselage definition lines
% 


end








