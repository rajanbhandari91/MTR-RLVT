function [aero_out] = ROM_Scaling(aero_in,geom)

aero_out = aero_in;

% find the spanwise dimension of the strip data
StripSpan = geom.StripDef.db_obd(end);

% capture the reference length for the original ROM data
ROMSpan = aero_in.RefLengthROM;

% Calculate length scaling factor
LengthScaleFactor = StripSpan/ROMSpan;
aero_out.LengthScaleFactor = LengthScaleFactor;

% ROM columns:
% 1. Y   2. dy   3. C0    4. dCdA
% each needs to be recomputed or scaled

% 1. recompute Y
Y = geom.StripDef.db_mid;

% 2. recompute dY
dY = geom.StripDef.db_obd - geom.StripDef.db_ibd;

% 3. scale C0
C0_Ref = aero_in.ROM_orig(:,3);
C0_scaled = C0_Ref * LengthScaleFactor;

% 4. scale dCdA
dCdA_Ref = aero_in.ROM_orig(:,4);
dCdA_scaled = dCdA_Ref * LengthScaleFactor;


% prepare to interpolate ROM based on eta
eta_ref = aero_in.ROM_orig(:,1)/aero_in.ROM_orig(end,1);
eta_query = Y/Y(end);
eta_query(eta_query<eta_ref(1)) = eta_ref(1);

% assign updated values to ROM table
ROMUpdated(:,1) = Y;
ROMUpdated(:,2) = dY;
ROMUpdated(:,3) = interp1(eta_ref,C0_scaled,eta_query,'linear');
ROMUpdated(:,4) = interp1(eta_ref,dCdA_scaled,eta_query,'linear');
ROMUpdated(:,5) = interp1(eta_ref,aero_in.ROM_orig(:,5),eta_query,'linear');
ROMUpdated(:,6) = interp1(eta_ref,aero_in.ROM_orig(:,5),eta_query,'linear');

aero_out.ROM = ROMUpdated;

% aero_out.ROM(:,1) = Y;
% aero_out.ROM(:,2) = dY;
% aero_out.ROM(:,3) = interp1(eta_ref,C0_scaled,eta_query,'linear');
% aero_out.ROM(:,4) = interp1(eta_ref,dCdA_scaled,eta_query,'linear');




