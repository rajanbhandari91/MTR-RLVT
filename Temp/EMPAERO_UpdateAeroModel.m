function [Vehicle] = EMPAERO_UpdateAeroModel(Vehicle)

% capture all the components of the Vehicle.Geom structure
Components = fieldnames(Vehicle.Geom);

% number of components
NumComp = length(Components);

running_count = 0;

StripDef = [];

%%% COLLECT ALL THE STRIPS
% stepping through all the components
for i = 1:NumComp


    % is there a field called "StripDef"
    if isfield(Vehicle.Geom.(Components{i}),'StripDef')

        % is it classified as a "lifting surface"
        if strcmpi(Vehicle.Geom.(Components{i}).Type,'liftingsurface')

            % is the strip field, if present, non-empty
            if ~isempty(Vehicle.Geom.(Components{i}).StripDef)
                % concatenate it
                StripDef = [StripDef; Vehicle.Geom.(Components{i}).StripDef];

                % how many strips?
                h = height(Vehicle.Geom.(Components{i}).StripDef);

                % if they have strips, log the strip indices
                Vehicle.Aero.Indices.(Components{i}) = running_count + [1:h];
                running_count = height(StripDef);


                % if they have control surfaces

                if ~isempty(Vehicle.Geom.(Components{i}).Controls)
                    [~,ncs] = size(Vehicle.Geom.(Components{i}).Controls);

                    % for each control, log the strips
                    for j = 1:ncs

                        CompName = Components{i};
                        CSname = strcat(Vehicle.Geom.(Components{i}).Controls(j).Name,'_on_',CompName);
                        Vehicle.Aero.Indices.(CSname) = Vehicle.Geom.(Components{i}).Controls(j).StripIndices;
                        Vehicle.Aero.ControlChordFracs.(CSname) = abs(diff(Vehicle.Geom.(Components{i}).Controls(j).ChordFrac));
                    end

                end

            end

            %   StripPlanformArea for scale factor computation
            Vehicle.Aero.(Components{i}).StripPlanformArea = sum(Vehicle.Geom.(Components{i}).StripDef.Area);
        end

    end

end

nstrips = height(StripDef);
if nstrips>0
    IndexCol = table();
    IndexCol.Index = (1:height(StripDef))';
    StripDef = [IndexCol,StripDef];
end


Vehicle.Aero.StripDef = StripDef;                                % tabular form
Vehicle.Aero.StripDefn = table2struct(StripDef,'ToScalar',true); % array form


% changes made for EM
% left wing
Vehicle.Aero.LWing = EMPAERO_CalcSpanLoading(Vehicle.Aero.LWing, Vehicle.Geom.LWing,Vehicle.Aero.OptCondition,Vehicle);

% right wing
Vehicle.Aero.RWing = EMPAERO_CalcSpanLoading(Vehicle.Aero.RWing, Vehicle.Geom.RWing,Vehicle.Aero.OptCondition,Vehicle);

% left HS
Vehicle.Aero.LHS = EMPAERO_CalcSpanLoading(Vehicle.Aero.LHS, Vehicle.Geom.LHS,Vehicle.Aero.OptCondition,Vehicle);

% right HS
Vehicle.Aero.RHS = EMPAERO_CalcSpanLoading(Vehicle.Aero.RHS, Vehicle.Geom.RHS,Vehicle.Aero.OptCondition,Vehicle);

% left VS
Vehicle.Aero.VS = EMPAERO_CalcSpanLoading(Vehicle.Aero.VS, Vehicle.Geom.VS,Vehicle.Aero.OptCondition,Vehicle);

%% Downwash gradient and angle (epsilon) GI

% % Wing on tail
Vehicle.Aero.LWing = EMPAERO_DownwashInterpolant_Raymer(Vehicle.Aero.LWing, Vehicle.Geom.LWing, Vehicle.Geom.LHS, Vehicle);
Vehicle.Aero.RWing = EMPAERO_DownwashInterpolant_Raymer(Vehicle.Aero.RWing, Vehicle.Geom.RWing, Vehicle.Geom.RHS, Vehicle);


%%

% other updates
% % Reference Area and lengths for Lifting Surfaces
% Vehicle.Aero.RefArea = 2*Vehicle.Recalcs.WingExposedAreaEach_m2;
% Vehicle.Aero.RefLengthLat = 2*Vehicle.Recalcs.WingExposedSpanEach_m;
% Vehicle.Aero.RefLengthLong = Vehicle.Geom.LWing.MAC;

Vehicle.Aero.RefArea = 22.3;
% Vehicle.Aero.RefLengthLat  = 2*Vehicle.Recalcs.WingExposedSpanEach_m;
Vehicle.Aero.RefLengthLong = 1.2192;

% % % % Reference Area and lengths for non-strip geometry
% Vehicle.Aero.FuselageRefArea = Vehicle.Geom.Fus.Length*max(Vehicle.Geom.Fus.Stn.Width);   % m2
% Vehicle.Aero.FuselageRefLengthLong = Vehicle.Geom.Fus.Length; % m
% Vehicle.Aero.FuselageRefLengthLat = Vehicle.Geom.Fus.Length; % m

Vehicle.Aero.RefArea_Fus = 22.3;
Vehicle.Aero.RefLength_Fus = 1.2192;


RefPt_NacWMP = [];
for iprop = 1:Vehicle.Propulsion.NProps
    NacName = horzcat('NAC_',num2str(iprop));
    RefPt_NacWMP = vertcat(RefPt_NacWMP, Vehicle.Geom.(NacName).RefPtLocation);
end
Vehicle.Aero.RefPt_NacWMP = RefPt_NacWMP;


%% Propeller setup

% update rotor setup
Vehicle.Propulsion.Setup.RotorRadius =  Vehicle.Propulsion.PropDiam_m/2;

[Vehicle.Propulsion.Setup] = UpdateRotorSetup(Vehicle.Propulsion.Setup);

Vehicle.Propulsion.Setup.nRotor = Vehicle.Propulsion.NProps;
Vehicle.Propulsion.Setup.SpinDir = [-ones(1,Vehicle.Propulsion.NProps/2), ones(1,Vehicle.Propulsion.NProps/2)];
Vehicle.Propulsion.Setup.HealthStatus = ones(1,Vehicle.Propulsion.NProps);
Vehicle.Propulsion.Setup.RotorAxisTheta = Vehicle.Geom.Prop_1.Theta * ones(1,Vehicle.Propulsion.NProps);
Vehicle.Propulsion.Setup.RotorAxisPhi = Vehicle.Geom.Prop_1.Phi * ones(1,Vehicle.Propulsion.NProps);


for iprop = 1:Vehicle.Propulsion.NProps
    PropName = horzcat('Prop_',num2str(iprop));
    Vehicle.Geom.(PropName).SpinDir = Vehicle.Propulsion.Setup.SpinDir(iprop);
end

% % update HLP locations (3 x nprop matrix)
RotorLoc = [];
for iprop = 1:Vehicle.Propulsion.NProps
    PropName = horzcat('Prop_',num2str(iprop));
    RotorLoc = horzcat(RotorLoc, Vehicle.Geom.(PropName).RefPtLocation);
end
Vehicle.Propulsion.Setup.RotorLoc = RotorLoc;

Vehicle.Propulsion.Setup.XBlades_RA = Vehicle.Propulsion.Setup.dXBlades_RA + reshape(Vehicle.Propulsion.Setup.RotorLoc(1,:),[1,1,Vehicle.Propulsion.Setup.nRotor]);
Vehicle.Propulsion.Setup.YBlades_RA = Vehicle.Propulsion.Setup.dYBlades_RA + reshape(Vehicle.Propulsion.Setup.RotorLoc(2,:),[1,1,Vehicle.Propulsion.Setup.nRotor]);
Vehicle.Propulsion.Setup.ZBlades_RA = Vehicle.Propulsion.Setup.dZBlades_RA + + reshape(Vehicle.Propulsion.Setup.RotorLoc(3,:),[1,1,Vehicle.Propulsion.Setup.nRotor]);

% update beta(CQ, J) interpolant
% Vehicle.Propulsion.PropCurves.GI_beta_CQ_J
Vehicle = TestProp_dtCQ(Vehicle);


%%  slipstream modeling parameters
% Compute distance from propellers to midchord of wings they are mounted on
s = abs(Vehicle.Propulsion.Setup.RotorLoc(1,:)) - abs(Vehicle.Geom.RWing.LocMAC(1) - Vehicle.Geom.RWing.MAC/4);

% calculate kd parameters: kd = 1 + s/sqrt(s^2+R^2)
kd = 1 + s./sqrt(s.^2 + Vehicle.Propulsion.Setup.RotorRadius^2);

% calculate r/c (radius of prop divided by MAC)
rc = Vehicle.Propulsion.Setup.RotorRadius./Vehicle.Geom.LWing.MAC;
beta = -0.3612 * rc.^5 + 2.2635 * rc.^4 - 5.0715 * rc.^3 + 4.3912 * rc.^2 - 0.3255 * rc;
beta(rc>=1.9) = 1;

Vehicle.Propulsion.Setup.kd = kd;
Vehicle.Propulsion.Setup.beta = beta;
Vehicle.Propulsion.Setup.kdxbeta = kd.*beta;


%% compute stall speed
% % 
% Vehicle.Aero.alpha_LOF = 8;
% AOA = (Vehicle.Aero.alpha_LOF:1:22)'.*pi/180;
% dcon = 0;
% dt = [0.2,0];
% rho = 1.225;
% Mode = [];
% V = 100 * 0.5144;
% NProps = Vehicle.Propulsion.NProps;
% 
% DOD = 0.02;
% UsedREPerc = Vehicle.SOTA.Battery.GI_usedRE_fcn_DOD(DOD);
% 
% Vehicle.States = [DOD;0;0;UsedREPerc];
% 
% % run AOA sweep to capture CL_TO and CLmax
% for i = 1:length(AOA)
%     [~,~,~,~,~,ADDL] = AeroPropPerf(AOA(i),dcon,dt,rho,V,Mode,Vehicle);
%     CL(i,1) = ADDL(NProps+4);
%     LD(i,1) = ADDL(NProps+6);
% end
% CL_TO = CL(1,1);
% CLmax = max(CL);
% Vehicle.Aero.AOA_stall = AOA(CL == CLmax)*180/pi;
% Vehicle.Aero.TOSpd = sqrt((2*Vehicle.DesignPoint.WL_kgm2*9.81)/(rho*CL_TO));
% Vehicle.Aero.StallSpd = sqrt((2*Vehicle.DesignPoint.WL_kgm2*9.81)/(rho*CLmax));


% Vs1 from the spec-sheet
Vehicle.Aero.alpha_LOF = 6;
Vehicle.Aero.StallSpd = 76*0.5144; 
Vehicle.Aero.TOSpd = 1.1*Vehicle.Aero.StallSpd;


end