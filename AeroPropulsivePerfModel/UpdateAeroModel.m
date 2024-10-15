function [Vehicle] = UpdateAeroModel(Vehicle)



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



Vehicle.Aero.StripDef = StripDef;               % tabular form
Vehicle.Aero.StripDefn = table2struct(StripDef,'ToScalar',true); % array form




% get operating conditions for updating the aerodynamic model
Vehicle = EMPAERO_ObtainOptCond(Vehicle);

% left wing
Vehicle.Aero.LWing = EMPAERO_CalcSpanLoading(Vehicle.Aero.LWing, Vehicle.Geom.LWing,Vehicle.Aero.OptCondition,Vehicle);

% right wing
Vehicle.Aero.RWing = EMPAERO_CalcSpanLoading(Vehicle.Aero.RWing, Vehicle.Geom.RWing,Vehicle.Aero.OptCondition,Vehicle);

% left horizontal tail
Vehicle.Aero.LHTail = EMPAERO_CalcSpanLoading(Vehicle.Aero.LHTail, Vehicle.Geom.LHTail,Vehicle.Aero.OptCondition,Vehicle);

% right horizontal tail
Vehicle.Aero.RHTail = EMPAERO_CalcSpanLoading(Vehicle.Aero.RHTail, Vehicle.Geom.RHTail,Vehicle.Aero.OptCondition,Vehicle);

% VTail
Vehicle.Aero.VTail = EMPAERO_CalcSpanLoading(Vehicle.Aero.VTail, Vehicle.Geom.VTail,Vehicle.Aero.OptCondition,Vehicle);

%% Downwash
Vehicle.Aero.LWing = EMPAERO_DownwashInterpolant_Raymer(Vehicle.Aero.LWing, Vehicle.Geom.LWing,Vehicle.Geom.LHTail,Vehicle);
Vehicle.Aero.RWing = EMPAERO_DownwashInterpolant_Raymer(Vehicle.Aero.RWing, Vehicle.Geom.RWing,Vehicle.Geom.RHTail,Vehicle);

%% Compute Landing Gear Drag
% Evaluate based on Raymer to find the wheel diameter for given MTOM
% Use diamter information to evaluate the landing gear
% Vehicle = EMPAERO_LandingGear(Vehicle);

% Evaluate based on Gudmunddson to use the wheel dimater information 
[Vehicle] = EMPAERO_FixedLandingGear(Vehicle);
% other updates
% % Reference Area and lengths for Lifting Surfaces
Vehicle.Aero.RefArea = 2*Vehicle.Geom.LWing.PlanformArea;
Vehicle.Aero.RefLengthLat = 2*Vehicle.Geom.LWing.Span;
Vehicle.Aero.RefLengthLong = Vehicle.Geom.LWing.MAC;

% % % Reference Area and lengths for non-strip geometry
Vehicle.Aero.FuselageRefArea = Vehicle.Geom.Fuselage.Length*max(Vehicle.Geom.Fuselage.Stn.Width);   % m2
Vehicle.Aero.FuselageRefLengthLong = Vehicle.Geom.Fuselage.Length; % m
Vehicle.Aero.FuselageRefLengthLat = Vehicle.Geom.Fuselage.Length; % m


% boom geometry update, 02.03.2022, Chakraborty
% inboard booms
Vehicle.Aero.RefArea_IBDBoom = Vehicle.Geom.Pylons_2.Length * Vehicle.Geom.Pylons_2.MaxWidth;          % update based on current geometry
Vehicle.Aero.RefLength_IBDBoom = Vehicle.Geom.Pylons_2.Length;                                           % update based on current geometry
Vehicle.Aero.RefPt_R_IBD_Boom = Vehicle.Geom.Pylons_2.RefPtLocation;
Vehicle.Aero.RefPt_L_IBD_Boom = Vehicle.Geom.Pylons_1.RefPtLocation;

end