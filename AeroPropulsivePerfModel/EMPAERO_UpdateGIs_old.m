function [Vehicle] = EMPAERO_UpdateGIs(Vehicle)
load ('empModel.Mat');
%% Plain TE flap
% Section lift increment due to the deflection of plain TE flaps (updated 02.21.2022, Kunwar)
% Digitization of DATCOM Fig. 6.1.1.1-39a, pg. 6.1.1.1-39 (Reader pg. 1909)
% Theoritical lift efffectiveness of plain TE flaps
% x1 = FCR
% three different interpolants generated due to nature of plot
Vehicle.Aero.TEFGI_cl_delta_theory_1 = empModel.PlainTEFlap2D.cl_delta_theory_1;
Vehicle.Aero.TEFGI_cl_delta_theory_2 = empModel.PlainTEFlap2D.cl_delta_theory_2;
Vehicle.Aero.TEFGI_cl_delta_theory_3 = empModel.PlainTEFlap2D.cl_delta_theory_3;

% NOTE: Cl_delta_theory comes out in units of (1/rad).
% Digitization of DATCOM (first) Fig. 4.1.1.2-8, pg.  4.1.1.2-8 (Reader pg. 478)
% x1 = Tan 1/2 phi_TE', x2 = log10(Re)
Vehicle.Aero.TEFGI_cl_alpha_ratio = empModel.PlainTEFlap2D.cl_alpha_ratio;

% Digitization of DATCOM Fig. 6.1.1.1-39b, pg. 6.1.1.1-39 (Reader pg. 1909)
% Empirical correction for lift effectiveness of plain TE flaps
% x1 = FCR, x2 = cl_alpha_ratio
Vehicle.Aero.TEFGI_cl_delta_to_cl_delta_theory = empModel.PlainTEFlap2D.cld_to_cld_theory;

% Digitization of DATCOM Fig. 6.1.1.1-40, pg. 6.1.1.1-40 (Reader pg. 1910)
% Empirical correction for lift effectiveness of plain TE flaps at high
% flap deflections
% x1 = flap deflection (deg)
Vehicle.Aero.TEFGI_K_prime = empModel.PlainTEFlap2D.K_prime;

%Two-dimensional profile drag increment due to flaps (plain flaps)
% Digitization of DATCOM Fig. 6.1.7-22, pg. 6.1.7-22 (Reader pg. 2229)
% x1 = FCR, x2 = df (deg)
Vehicle.Aero.PlainTEFlapGI.delta_cdfp = empModel.PlainTEFlap2D.delta_cdf;
%% Single-slotted flap
% Section lift-effectiveness parameter of single-slotted flaps (updated 02.21.2022, Kunwar)
% Digitization of Datcom Fig 6.1.1.1-41 (Reader pg. 1911)
% alpha_d = f(df(deg),cf/c);
Vehicle.Aero.SingleSlottedTEFlapLiftEffParamGI = empModel.SingleSlottedTEFlap2D.alpha_d;

% Two-dimensional drag increment due to flap
% Digitization of DATCOM Fig. 6.1.7.23 (Reader pg. 2230)
% delta_cdf = f(cf/c,df(deg));
Vehicle.Aero.SingleSlottedTEFlapGI.delta_cdfp = empModel.SingleSlottedTEFlap2D.delta_cdf;

%% Spanwise loading coefficient (G/delta) FLAP
% Digitization of DATCOM Fig. 6.1.5.1-62b through Fig.6.1.5.1-62d , pg. 66.1.5.1-62 through 6.1.5.1-66  (Reader pg. 2137-2140)
% Spanwise load distribution due to symmetric flap deflection for
% straight-tapered wings w/ FCR = 1.
% x1 = eta [0 1.0], x2 = TR, x3 = IBD_eta_flap, x4 = sweep_Beta, x5 = Beta x A/k_av
Vehicle.Aero.SpanLoad = empModel.spanLoading;

%% Spanwise loading coefficient (G/delta) AILERON
% Recreated based on Theoritical Antisymmetric span loaidng for wings of
% arbitrary plan form at subsonic speeds ( Report 1056)
% x1: fraction of semispan, n
% x2: aileron-tip-span-to-wing-span ratio measured inboard from wing tip.
% x3: taper ratio, TR
% x4: sweep and compressibility parameter, sweepB
% x5: Beta*A/k
Vehicle.Aero.G_to_delta_ail = empModel.G_to_delta_ail;


%% Flap drag
% Digitization of DATCOM Figure 6.1.4.1-15 Span Factor For Inboard Flaps
% (Reader pg. 2041)
% x1 = flap-end point eta, x2 = TR
Vehicle.Aero.TEFGI_FlapSpanFactor = empModel.GI_SpanFactorInboardFlaps;

%% Wing subsonic downwash effect on tail
%% Step1: Effective wing aspect ratio and span
% Digitization of DATCOM Fig. 4.4.1-66 (quadrant II), pg. 4.4.1-66 (Reader pg. 1268)
% x1 = AOA_parameter
% x2 = Sweep angle
Vehicle.Aero.GI_EffectiveWingAR_S1 = empModel.EffectiveWingAR_Step1;

% Digitization of DATCOM Fig. 4.4.1-66 (quadrant I), pg. 4.4.1-66 (Reader pg. 1268)
% x1 = Ouput of Vehicle.Aero.GI_EffectiveWingAR_S1
% x2 = Taper ratio
Vehicle.Aero.GI_EffectiveWingAR_S2 = empModel.EffectiveWingAR_Step2;

% Digitization of DATCOM Fig. 4.4.1-66 (quadrant IV), pg. 4.4.1-66 (Reader pg. 1268) IV
% x1 = AR_eff_to_AR (Output of Vehicle.Aero.GI_EffectiveWingAR_S2)
% x2 = Taper ratio
Vehicle.Aero.GI_EffectiveWingSpan = empModel.EffectiveWingSpan;

%% Step2: Downwash at the plane of symmetry and height of vortex core
% Digitization of DATCOM Fig. 4.4.1-67 (quadrant I), pg. 4.4.1-67 (Reader pg. 1269)
% x1 = l2/(b/2) Tail length in semispans
% x2 = AR_eff
% Output is some intermediate value
Vehicle.Aero.GI_deda_S1 = empModel.d_epsilon_d_alpha_Step1;

% Digitization of DATCOM Fig. 4.4.1-67 (quadrant III), pg. 4.4.1-67 (Reader pg. 1269)
% x1 = AR_eff
% x2 = Sweep
% Output is downwash greadient at infinity
Vehicle.Aero.GI_deda_S2= empModel.d_epsilon_d_alpha_Step2;

% Step3: Vertical position of the vortex core based on dy
% Substep: Identification of the seperation type

% Digitization of DATCOM Fig. 4.4.1-68a pg. 4.4.1-68a (Reader pg. 1270)
% x1	lower_boundary	x2	upper_boundary
data = [...
    0	    1.2615	0	1.7023
    19.937	1.2775	20.147	1.694
    30.186	1.4695	29.91	1.8489
    39.69	1.8079	34.908	1.9999
    44.922	2.0816	39.781	2.1876
    50.509	2.5025	44.886	2.5224
    54.624	2.9956	50.223	3.0165];
x1 = data(:,1);
y1 = data(:,2);
x2 = data(:,3);
y2 = data(:,4);
xv = [x1;flip(x2)];
yv = [y1;flip(y2)];
sweep_max = 60;
dy_max = 3;
poly1 = polyshape(xv,yv);
poly2 = polyshape([0;x1;sweep_max;sweep_max],[0;y1;dy_max;0]);
poly = polyshape([0 0 sweep_max sweep_max],[0 dy_max dy_max 0]);
poly3 = subtract(poly,union(poly1,poly2));

Vehicle.Aero.FlowSepType.Poly1 = poly1;
Vehicle.Aero.FlowSepType.Poly2 = poly2;
Vehicle.Aero.FlowSepType.Poly3 = poly3;


% Step 4: Span of the vertices
%% Step 5: Average downwash gradient acting on the tail (de/da)
% Digitization of DATCOM Fig. 4.4.1-68b pg. 4.4.1-68 (Reader pg. 1270)
Vehicle.Aero.GI_deda_ratio = empModel.deda_ratio;

%% Downwash gradient acting on the aft surface due to forward surface (de/da)
% Digitization of Fig 16.12 of Aircraft Design: A Conceptual Approach (Raymer (2018), Reader pg. 634)
Vehicle.Aero.GI_deda_raymer = empModel.GI_deda_raymer;

%% Upwash angle derivative with respect to wing angle of attack (deu/da)
% Digitization of Fig 16.11 of Aircraft Design: A Conceptual Approach (Raymer (2018), Reader pg. 633)
% X1 = distance ahead of the wing root quarterchord in root chords
% X2 = Aspect ratio of the wing
% x A4 A6 A9 A12
deu_dalpha = [...
    0.25      1.1475       1.215     1.3576     1.4593
    0.3     0.85479     0.96543     1.1113     1.2062
    0.4     0.55708     0.69123     0.8021    0.93029
    0.6     0.32437     0.40748     0.4902    0.55773
    0.8     0.19942     0.28247     0.3476    0.39861
    1     0.14214     0.20012    0.26039    0.30112
    1.2      0.1008     0.15353    0.19856     0.2457
    1.4    0.077653     0.11903    0.15872    0.20051
    1.6    0.061485    0.094631    0.13713    0.17579
    1.8    0.053555      0.0886    0.12864     0.1626
    2    0.052581    0.093253    0.13153    0.15409];

X1 = deu_dalpha(:,1);
X2 = [4;6;9;12];
Vehicle.Aero.UpwashGI_deu_dalpha = griddedInterpolant({X1,X2},deu_dalpha(:,2:5),'linear','nearest');

%% Fuselage
% Digitization of DATCOM Fig. 4.2.1.1-20a (Reader pg. 834)
Vehicle.Aero.GI_ApparentMassFactor = empModel.GI_ApparentMassF;


%  steady-state cross-flow drag coefficient for circular cylinders (2d)
% Digitization of DATCOM Fig. 4.2.1.2-35b (Reader pg. 875)
% x1 = data.Mc;
% y1 = data.Cdc;
Vehicle.Aero.GI_cdc = empModel.GI_cdc;

% ratio of drag on a finite cylinder to the drag of an infinite cylinder
% Digitization of DATCOM Fig. 4.2.1.2-35a (Reader pg. 875)
% x1 = data.Body_Fineness_ratio;
% y1 = data.eta;
Vehicle.Aero.GI_CirCyl_CD_ratio = empModel.GI_CirCyl_CD_ratio;

% Digitization of DATCOM figure 4.1.5.1-27 (Reader pg. 748)
% query  cut-off Re
% x1 = log10(l/k);
% x2 = Mach Number
% log10 of cutoffRe
Vehicle.Aero.GI_cutoffRe_Log10 = empModel.GI_cutoffRe_Log10;

% Digitization of DATCOM figure 4.1.5.1-26 (Reader pg. 747)
% x1 = Re;
% x2 = Mach Number
Vehicle.Aero.GI_skin_fric_coeff = empModel.GI_skin_fric_coeff;

% Speed brake drag  
% Taken from Raymer Table 12, pg. 429 (Reader pg. 459)
Vehicle.Aero.Speedbrake_fus_CDpi = 1; % fuselage mounted speed brake
Vehicle.Aero.Speedbrake_wing_CDpi = 1.6; % wing mounted speed brake 

end 