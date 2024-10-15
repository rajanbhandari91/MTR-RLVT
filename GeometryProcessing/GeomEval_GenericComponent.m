function [GEN] = GeomEval_GenericComponent(GEN)

    GEN.CG = GEN.RefPtLocation;
    n = length(GEN.Mass);

    r11 = 1 * ones(n,1);
    r12 = 0 * ones(n,1);
    r13 = 0 * ones(n,1);
    r21 = 0 * ones(n,1);
    r22 = 1 * ones(n,1);
    r23 = 0 * ones(n,1);
    r31 = 0 * ones(n,1);
    r32 = 0 * ones(n,1);
    r33 = 1 * ones(n,1);
    
    ri = 0;

    % unit vector 1 transformation
    R11_BS = r11;
    R21_BS = r21;
    R31_BS = r31;
    % unit vector 2 transformation
    R12_BS = r12;
    R22_BS = r22;
    R32_BS = r32;
    % unit vector 3 transformation
    R13_BS = r13;
    R23_BS = r23;
    R33_BS = r33;

    if(strcmpi(GEN.Shape,'cylinder'))
        GEN.rdim_m = 0.5.*(GEN.yDim_m + GEN.zDim_m);
        GEN.Ixx_ca = 0.5.*GEN.Mass.*GEN.rdim_m.^2;
        GEN.Iyy_ca = (1/12).*GEN.Mass.*(3.*GEN.rdim_m.^2 + GEN.xDim_m.^2);
        GEN.Izz_ca = GEN.Iyy_ca;
    end

    R = [R11_BS,R12_BS,R13_BS,R21_BS,R22_BS,R23_BS,R31_BS,R32_BS,R33_BS];
    % Ixx,Iyy,Izz,Ixy,Iyz,Ixz
    Ica = [GEN.Ixx_ca',GEN.Iyy_ca',GEN.Izz_ca',GEN.Ixy_ca',GEN.Iyz_ca',GEN.Izx_ca'];

    [Ixx,Iyy,Izz,Ixy,Iyz,Izx] = ApplyParallelAxisTheorem(GEN.Mass,GEN.CG,Ica,R);

    GEN.Ixx = Ixx';
    GEN.Iyy = Iyy';
    GEN.Izz = Izz';
    GEN.Ixy = Ixy';
    GEN.Iyz = Iyz';
    GEN.Izx = Izx';

end