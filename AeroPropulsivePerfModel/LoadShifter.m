function [Fx, Fy, Fz, Mx, My, Mz] = LoadShifter(Fx0, Fy0, Fz0, Mx0, My0, Mz0, xref, yref, zref)

% notes:
% 1. [xref, yref, zref] is the position vector from the origin of the input
% load system (subscript 0) to the origin of the output load system 

% 2. input forces and moments must all be arrays of the same size

% 3. input forces, moments, and moment arms can either all be dimensional or
% non-dimensional quantities

% direct pass through of the forces
Fx = Fx0;
Fy = Fy0;
Fz = Fz0;

% correct the moments
Mx = Mx0 - (          - zref.*Fy + yref.*Fz);
My = My0 - ( zref.*Fx            - xref.*Fz);
Mz = Mz0 - (-yref.*Fx + xref.*Fy           );


