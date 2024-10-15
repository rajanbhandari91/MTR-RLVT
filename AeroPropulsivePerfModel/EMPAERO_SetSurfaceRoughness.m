function [Vehicle] = EMPAERO_SetSurfaceRoughness(Vehicle)

% Based on DATCOM (Reader pg. 723)
% Set Surface_roughness_ height in m ( 0 for smooth surface)

LS_surface_roughness_height = 23.37 * 1e-6;               
Fus_surface_roughness_height = 23.37 * 1e-6;
MainGear_surface_roughness_height = 23.37 * 1e-6;

Vehicle.Geom.LWing.surface_roughness_height = LS_surface_roughness_height;
Vehicle.Geom.RWing.surface_roughness_height = LS_surface_roughness_height;
Vehicle.Geom.LHTail.surface_roughness_height = LS_surface_roughness_height;
Vehicle.Geom.RHTail.surface_roughness_height = LS_surface_roughness_height;
Vehicle.Geom.VTail.surface_roughness_height = LS_surface_roughness_height;
Vehicle.Geom.Fuselage.surface_roughness_height = Fus_surface_roughness_height;
% Vehicle.Geom.LMG.surface_roughness_height = MainGear_surface_roughness_height;
% Vehicle.Geom.RMG.surface_roughness_height = MainGear_surface_roughness_height;
end 
