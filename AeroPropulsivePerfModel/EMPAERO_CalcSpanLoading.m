function [aero_out] = EMPAERO_CalcSpanLoading(aero_in,geom,OptCondition,Vehicle)
CSType = 'Aileron';
% CSType = 'Flap';
% CalcSpanLoading calculates spanwise loading coefficient (G/delta) per radain of angle of attack for
% wing and per radian of flap deflection for flaps or control surfaces
aero_out = aero_in;

%% Capture data to be used later in calculating downwash
% coordinate starting outboard of fuselage for wings
% generalized for every lifting surfces
y = geom.StripDef.db_mid;
dy = diff(y);

stripLen = geom.StripDef.Length;
b_exp = sum(stripLen);
% Beta : Prandtl-Galauert compressibility orrection factor [ B = sqrt(1-M^2), M: Mach]
B = OptCondition.Beta;

%% Major Task 1: Calculate (basic) spanwise loading coefficient per radian (G/delta) for full wing without flaps or control surfaces
% DATCOM pg. 6.1.5.1-4 (Reader pg. 2078)
% Example problem: DATCOM pg. 6.1.7-7 (Reader pg. 2214)

% Extract/Compute the following parameters
% 1. LS etas
% 2. Taper ratio
% 3. Section etas, ni and no
% 4. SweepB
% 5. Param: Beta x AR/k, k = cl_alpha/(2pi) % Kind of , need airfoil cla

% 1. Capture the eta of the lifting surface ( for example, Left wing)
% Note eta and eta_db correspond to the same strip midpoints and can be mapped to one another
eta = geom.StripDef.eta;                                                    % eta : station of the midpoint of the strips w.r.to the tip-to-tip span
eta_exp = geom.StripDef.eta_exp;                                            % eta_db : station of the midpoint of the strips w.r.to the exposed span

% 2. Capture taper ratio of the lifting surface ( for example, Left wing)
% need to update code for multiple taper definition
% disp(geom.Name)
Taper = geom.TaperDefn(2,:);
rootchord = geom.Stn.c(1);
% 3. Treat each section of the wing as seperate parts
% find the loading due to each part
% add the loading

% compute lift curve slope
% each strip might have different cla due to different strip airfoil
% the code assumes a previous knowledge that only wing can have variable
% arifoils.
% update if the canard can also have variable airfoil

TC_OB = geom.StripDef.TC(end);
eta_TC_change = abs(TC_OB-geom.StripDef.TC) > 0.001;

% if any(eta_TC_change==1)
%     Cla(eta_TC_change==1,1) = Vehicle.Aero.StrakeCLa * 57.3;
%     Cla(eta_TC_change==0,1) = Vehicle.Aero.WingCLa * 57.3;
% else
    Cla = aero_in.Cla * 57.3;
% end

% if abs(geom.StripDef.TC - geom.StripDef.TC(1)) < 0.001
%     Cla = aero_in.CLa * 57.3;
% else
%     Cla(geom.StripDef.TC<TC_outbaord) = Vehicle.Aero.GI_TCR_Cla_wing(geom.StripDef.TC)*57.3;
%     TC_outbaord =  0.18003;
%     Cla(geom.StripDef.TC<TC_outbaord) = Vehicle.Aero.StrakeCLa * 57.3;
% end



n_sec = size(geom.TaperDefn,2)-1;
if n_sec == 1
    S_exposed = geom.ExposedPlanformArea;
    b_exposed = sum(geom.StripDef.Length);
    AR_exposed = 2 * b_exposed^2/S_exposed;
    % find the root and tip chord of the "exposed" LS
    chord_rootNtip = interp1(eta_exp,geom.StripDef.Chord,[0;1],'linear','extrap');
    TR_exposed = chord_rootNtip(2)/chord_rootNtip(1);
    sweep = geom.Stn.Sw_25(1);
    sweepB = atand(tand(sweep/B));



    K = Cla/(2*pi);
    Param = B*AR_exposed/K;

    height = ones(size(eta_exp,1),1);
    TR_vec = TR_exposed * height;
    eta_tip_vec = 1 * height;
    sweepB_vec = sweepB * height;
    Param_vec = Param * height;
    alpha_d = -1;

    G_to_alpha_LS = Vehicle.Aero.SpanLoad(eta_exp,TR_vec,eta_tip_vec,sweepB_vec,Param_vec);
    G_to_alpha_LS = -G_to_alpha_LS * alpha_d;
end
if n_sec>1
    TR_new = zeros(n_sec,1);
    AR_new = zeros(n_sec,1);
    for i = 1:n_sec
        eta_ibd = geom.TaperDefn(1,i);
        eta_obd = geom.TaperDefn(1,i+1);
        % make a new wing by extending the LE and TE of the section upto
        % the span of the original wing
        TR_eta = (Taper(i)-Taper(i+1)) /(eta_ibd-eta_obd) * (eta-eta_ibd) + Taper(i);
        % chord_new is the chord at midpoint of the strips of exposed
        % portion
        chord_new = TR_eta.*rootchord;
        chord_rootNtip = interp1(eta_exp,chord_new,[0;1],'linear','extrap');
        % take only the chord that are positive to develop the new wing
        % negative chord results when the extension of the LE and TE line meet
        % before the original wing span
        nng_index = chord_new>0;
        chord_new_sc= chord_new(nng_index);
        stripLen_new = stripLen(nng_index) ;
        S_half_new = sum(chord_new_sc.*stripLen_new);
         % semi_span_new = sum(geom.StripDef.Length);
        semi_span_new = sum(geom.StripDef.Length(nng_index));
        AR_new(i) = 2*semi_span_new^2/(S_half_new);

        if sum(nng_index==0)
            TR_new(i)  = 0;
        else
            TR_new(i) = chord_rootNtip(2)/chord_rootNtip(1);
        end
        TR = TR_new(i);
        % 4. sweep angle at quarter chord for each section
        % sweep angle remains constant for each sections
        % taken at outboard station
        sweep = geom.Stn.Sw_25(geom.Stn.eta==eta_obd);
        % SweepB
        sweepB = atand(tand(sweep)/B);

        % 5. Param (Beta x AR)/k, k = cla/(2pi) % need airfoil lift curve slope
        % cla: lift curve slope

        AR = AR_new(i) ;
        K = Cla./(2*pi);
        Param = B*AR./K;

        % convert eta_cs_ibd and eta_cs_obd to corresponding values on exposed wing
        eta_new = interp1(eta,eta_exp,[eta_ibd;eta_obd],'linear','extrap');

        height = ones(size(eta_exp,1),1);
        TR_vec = TR * height;
        eta_ibd_vec = eta_new(1) * height;
        eta_obd_vec = eta_new(2) * height;
        sweepB_vec = sweepB * height;
        Param_vec = Param .* height;
        alpha_d = -1;                                                                % lift-effectiveness parameter; -1 for wing
        G_to_alpha_ibd = Vehicle.Aero.SpanLoad(eta_exp,TR_vec,eta_ibd_vec,sweepB_vec,Param_vec);
        G_to_alpha_obd = Vehicle.Aero.SpanLoad(eta_exp,TR_vec,eta_obd_vec,sweepB_vec,Param_vec);

        % zero out extrapolated data for which design curve does not exist
        % To do : make a new GI with G/alpha = 0 when eta_ibd = 0
        if eta_ibd < 0.195
            G_to_alpha_ibd = 0;
        end
        G_to_alpha_sec = G_to_alpha_obd - G_to_alpha_ibd;
        % store the span loading due to each sections as columns
        G_to_alpha_crank(:,i) = G_to_alpha_sec;
    end
    G_to_alpha_LS = sum(G_to_alpha_crank,2);
    G_to_alpha_LS = -G_to_alpha_LS*alpha_d ;
end



aero_out.LS.G_to_alpha = G_to_alpha_LS;
aero_out.LS.y = y;
aero_out.LS.dy = dy;
aero_out.LS.Span = b_exp;
aero_out.LS.eta = eta;
aero_out.LS.Name = geom.Name;
aero_out.LS.stripLen = stripLen;

%% Major Task 2: Calculate (incremental) spanwise loading coefficient per radian (G/delta)  control surfaces
% do the following taks for each flaps or control surfaces
% 3. Capture ontrol surface location, eta_ibd and eat_obd
% do for each control surface if there are multiple control surfaces


Cla = aero_in.Cla * 57.3;                                                   % per radian
K = Cla/(2*pi);

% for lifting surface with one section
if n_sec ==1
    TR = TR_exposed;
    AR = AR_exposed;
end

% for lifting surface with multiple sections
% TR and AR of the the section, where the control surface lies, developed by extending LE and TE to the root


% multiple control surface
ncs = 0;
if ~isempty(geom.Controls)
    [~,ncs] = size(geom.Controls);

    % for each control, log the chord fraction and end points
    for j = 1:ncs
        CSname = strcat(geom.Controls(j).Name);
        aero_out.CSname{j} = CSname;
        aero_out.ControlChordFracs.(CSname) = abs(diff(geom.Controls(j).ChordFrac));
        aero_out.ControlEta.(CSname).eta = geom.Controls(j).eta;
    end
else
    aero_out.CSname = [];
end
for j = 1:ncs
    CSname = aero_out.CSname{j};
    % Make a vector of the values to query GI
    eta_cs_ibd = aero_out.ControlEta.(CSname).eta (1);
    eta_cs_obd = aero_out.ControlEta.(CSname).eta (2);

    if n_sec > 1
        for i = 1 :n_sec
            if eta_cs_ibd >= geom.TaperDefn(1,i)&& eta_cs_obd <= geom.TaperDefn(1,i+1)
                TR = TR_new(i);
                AR = AR_new(i);
            end
        end
    end
    % convert eta_cs_ibd and eta_cs_obd to corresponding values on exposed wing
    eta_new = interp1(eta,eta_exp,[eta_cs_ibd;eta_cs_obd],'linear','extrap');

    Param = B*AR/K;
    % Vector for quering the GI
    TR_vec = TR * height;
    eta_cs_ibd_vec = eta_new(1) * height;
    eta_cs_obd_vec = eta_new(2) * height;
    sweepB_vec = sweepB * height;
    Param_vec = Param * height;

    if strcmp(CSType,'Flap')
        % Spanwise loading coefficient G/delta (per rad) for full wing-chord flaps
        G_to_delta_fullChord_eta_cs_ibd = Vehicle.Aero.SpanLoad(eta_exp,TR_vec,eta_cs_ibd_vec,sweepB_vec,Param_vec);
        G_to_delta_fullChord_eta_cs_obd = Vehicle.Aero.SpanLoad(eta_exp,TR_vec,eta_cs_obd_vec,sweepB_vec,Param_vec);
    end

    if strcmp(CSType,'Aileron')
        % Spanwise loading coefficient G/delta (per rad) for full wing-chord
        % aileron
        % span measured from wing tip inboard
        G_to_delta_fullChord_eta_cs_ibd = Vehicle.Aero.G_to_delta_ail(eta_exp,1-eta_cs_ibd_vec,TR_vec,sweepB_vec,Param_vec);
        G_to_delta_fullChord_eta_cs_obd = Vehicle.Aero.G_to_delta_ail(eta_exp,1-eta_cs_obd_vec,TR_vec,sweepB_vec,Param_vec);
    end



    %% Sub Task: Calculate 2-D lift-effectiveness parameter (alpha_d) for flaps or control surfaces
    % DATCOM pg. 6.1.5.1-7 (Reader pg. 2081)

    % 1. Calcuate flap-lift effectiveness (cl_delta)
    % DATCOM pg. 6.1.1.1-8 (Reader pg. 1878)

    % Required Parameters

    % step1
    % Theoritical Lift Effectiveness of Plain TE Flaps
    int1 = 0.05;
    int2 = 0.15;
    int3 = 0.3;
    int4 = 0.5;

    % take value from baseline
    FCR = aero_out.ControlChordFracs.(CSname);                              % FCR : flap-chord-to-airfoil-chord ratio
    % When different control surface sections have airfoils with different
    % thickness ratio
    CSBeg = geom.(CSname).StripIndices(1);
    ATR = geom.StripDef.TC(CSBeg);                                          % ATR: airfoil-chord-to-thickness ratio

    % for stabiliators like surfaces, treat them as lifting surface
    if FCR == 1
        alpha_d_over_Kprime = 1;
    else

        if (FCR>=int1 && FCR<int2)
            GI_F = Vehicle.Aero.TEFGI_cl_delta_theory_1;
        elseif (FCR>=int2 && FCR<int3)
            GI_F = Vehicle.Aero.TEFGI_cl_delta_theory_2;
        elseif (FCR>=int3 && FCR<=int4)
            GI_F = Vehicle.Aero.TEFGI_cl_delta_theory_3;
        end

        cl_delta_theory = GI_F(FCR,ATR);                                        % per rad

        % step2
        % DATCOM (first) Fig. 4.1.1.2-8, pg.  4.1.1.2-8 (Reader pg. 478)
        % x1 = Tan 1/2 phi_TE', x2 = log10(Re)
        % phi_TE' is the total trailing edge angle

        %         extract GI

        GI_AF_xy_top = geom.AirfoilGeom.GI_AF_xy_top;
        GI_AF_xy_bot = geom.AirfoilGeom.GI_AF_xy_bot;
        Y90 = GI_AF_xy_top(0.9)-GI_AF_xy_bot(0.9);
        Y99 = GI_AF_xy_top(0.99)-GI_AF_xy_bot(0.99);

        % The base airfoil may be stretched in y-direction
        % ATR/AF_Thickness_to_chord multiplicative term takes that into account
        AF_x_q = 0:0.1:1;
        y_top_dist= GI_AF_xy_top(AF_x_q);
        y_bot_dist = GI_AF_xy_bot(AF_x_q);

        AF_Thickness_to_chord = max(y_top_dist-y_bot_dist);
        TE_slope = (Y90/2-Y99/2)/(0.09) * ATR/AF_Thickness_to_chord;       % Airfoil TE slope
        Re_pul = OptCondition.Re_pul;                                           % Re: Reynodls number per unit length
        Re = Re_pul*geom.MAC;

        cl_alpha_ratio = Vehicle.Aero.TEFGI_cl_alpha_ratio(TE_slope,log10(Re));
        % step3
        cl_delta_ratio = Vehicle.Aero.TEFGI_cl_delta_to_cl_delta_theory (FCR,cl_alpha_ratio);
        % calculate alpha_d
        alpha_d_over_Kprime = -cl_delta_ratio*cl_delta_theory/Cla;            % this value should be multiplied by K', empirical correction factor at high deflection while computing delta_Cl due to CS
    end
    aero_out.(CSname).alpha_d_over_Kprime = alpha_d_over_Kprime;
    %% Sub Task: Calculate span loading coefficient per radian of flap deflection (G/delta) for partial wing-chord flaps
    % DATCOM pg. 6.1.7.-1 (Reader pg. 2208)
    G_to_delta_eta_cs_ibd = -G_to_delta_fullChord_eta_cs_ibd;
    G_to_delta_eta_cs_obd = -G_to_delta_fullChord_eta_cs_obd;

    if strcmp(CSType,'Flap')
        G_to_delta = G_to_delta_eta_cs_obd-G_to_delta_eta_cs_ibd;                 % per radian of flap deflection ...correct for non-linearity by K' in MainAeroFcnCall
    end

    if strcmp(CSType,'Aileron')
        G_to_delta = -G_to_delta_eta_cs_obd+G_to_delta_eta_cs_ibd;                 % per radian of flap deflection ...correct for non-linearity by K' in MainAeroFcnCall
    end
    aero_out.(CSname).G_to_delta = G_to_delta;

    %% Sub Task: Calculate Span factor for inboard flaps for 3D profile drag calculation
    Kb_fep = Vehicle.Aero.TEFGI_FlapSpanFactor([eta_new(1);eta_new(2)],[TR;TR]);
    aero_out.(CSname).Kb = diff(Kb_fep);

    % Since the control surface chord ratio remains constant until the
    % geometry is updated in the next iteration, make control surface GIs
    % based on deflection angle for given chord ratio

    % cl related GIs
    def_vect_deg = [10    13    15    17    19    20    21    23    25    27    29    30    40    50    60]';
    chordratio_vect = ones(size(def_vect_deg,1),1) * FCR;
    K_prime_vec = Vehicle.Aero.TEFGI_K_prime(def_vect_deg,chordratio_vect);
    aero_out.(CSname).PlainFlap_Kprime = griddedInterpolant(def_vect_deg,K_prime_vec,'linear','nearest');

    LiftEff_vec = Vehicle.Aero.SingleSlottedTEFlapLiftEffParamGI(def_vect_deg,chordratio_vect);
    aero_out.(CSname).SingleSlottedFlap_LiftEffParam = griddedInterpolant(def_vect_deg,LiftEff_vec);

    % drag related GIs
    PlainFlap_delta_Cdp = Vehicle.Aero.PlainTEFlapGI.delta_cdfp(chordratio_vect,def_vect_deg);
    aero_out.(CSname).PlainFlap_delta_Cdp = griddedInterpolant(def_vect_deg, PlainFlap_delta_Cdp);
    SingleSlottedFlap_delta_Cdp = Vehicle.Aero.SingleSlottedTEFlapGI.delta_cdfp(chordratio_vect,def_vect_deg);
    aero_out.(CSname).SingleSlottedFlap_delta_Cdp = griddedInterpolant(def_vect_deg, SingleSlottedFlap_delta_Cdp,'linear','nearest');
end




