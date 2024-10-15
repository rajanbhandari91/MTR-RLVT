function Blade = getBladeProfile(filename,targetName)

%load the .mat file generated after exporting from openVSP using degenGEOM
run(filename);
index = find(arrayfun(@(s) strcmp(s.name, targetName), degenGeom));

% Extract the coordinates of the points in x, y, and z axis.
x = degenGeom(index(1)).surf.x ;
y = degenGeom(index(1)).surf.y ;
z = degenGeom(index(1)).surf.z ;

Blade.x = x;
Blade.y = y;
Blade.z = z;

end