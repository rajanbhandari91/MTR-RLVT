function [SecData_out] = ApplyViternaEquations(SecData_in,AR)



% REFERENCES
% REF1: 2014 - Selig - Real-Time FLight Simulatin of Highly Maneuverable
% UAVs (Journal of Aircraft, Vol. 51, No. 6, Nov-Dec 2014) 

% SecData_in and SecData_out format
% Col 1: AOA
% Col 2: sectional Cl
% Col 3: sectional Cd
% Col 4: sectional Cm

% subscript g --> "given"
AOA_g = SecData_in(:,1);
Cl_g = SecData_in(:,2);
Cd_g = SecData_in(:,3);
Cm_g = SecData_in(:,4);



%% STEP 1: GENERATE LIFT AND DRAG CURVES IN THE [-180,+180] DEG AOA RANGE
% ASSUMING INFINITE AR, i.e., WITHOUT ACCOUNTING FOR FINITE AR

% A. Zone H1: Apply Viterna equations from max(AOA_given) to 90 deg
Cl_lim_pos = Cl_g(end);
indCl_lim_pos = length(Cl_g);
AOA_lim_pos = AOA_g(indCl_lim_pos);
Cd_lim_pos = Cd_g(indCl_lim_pos);

[AOA_ZoneH1,Cl_ZoneH1, Cd_ZoneH1] = ApplyViterna(AOA_lim_pos, Cl_lim_pos, Cd_lim_pos);

xcp = 0.50 - (1-abs(AOA_ZoneH1)/90).*(0.175 + 0.13.*(1-abs(AOA_ZoneH1)/90));
Cm_ZoneH1 = -(xcp - 0.25).*(Cl_ZoneH1.*cosd(AOA_ZoneH1) + Cd_ZoneH1.*sind(AOA_ZoneH1));
Cm_ZoneH1 = -(0.25-0.175.*(1-abs(AOA_ZoneH1)/90)).*(Cl_ZoneH1.*cosd(AOA_ZoneH1) + Cd_ZoneH1.*sind(AOA_ZoneH1));
Cm_ZoneH1 = 0 * Cm_ZoneH1;  % zeroing out; not satisfactory


% B. Zone L1: Apply Viterna equations from min(AOA_given) to -90 deg
Cl_lim_neg = Cl_g(1);
AOA_lim_neg = AOA_g(1);
Cd_lim_neg = Cd_g(1);

[AOA_ZoneL1,Cl_ZoneL1, Cd_ZoneL1] = ApplyViterna(AOA_lim_neg, Cl_lim_neg, Cd_lim_neg);

xcp = 0.50 - (1-abs(AOA_ZoneL1)/90).*(0.175 + 0.13.*(1-abs(AOA_ZoneL1)/90));
Cm_ZoneL1 = -(xcp - 0.25).*(Cl_ZoneL1.*cosd(AOA_ZoneL1) + Cd_ZoneL1.*sind(AOA_ZoneL1));
Cm_ZoneL1 = 0 * Cm_ZoneL1;  % zeroing out; not satisfactory

% C. Zone H2: Apply flat plate theory for reverse flow region from (90,180] deg
AOA_ZoneH2 = (91:1:180)';
Cd0 = interp1(AOA_g, Cd_g,0,'linear');
[Cl_ZoneH2, Cd_ZoneH2] = FlatPlateTheory(AOA_ZoneH2,Cd0);

Cm_ZoneH2 = -0.5*(Cl_ZoneH2.*cosd(AOA_ZoneH2) + Cd_ZoneH2.*sind(AOA_ZoneH2));
Cm_ZoneH2 = 0 * Cm_ZoneH2;  % zeroing out; not satisfactory


% D. Zone L2: Apply flat plate theory for reverse flow region from [-180,-90) deg
AOA_ZoneL2 = (-91:-1:-180)';
[Cl_ZoneL2, Cd_ZoneL2] = FlatPlateTheory(AOA_ZoneL2,Cd0);

Cm_ZoneL2 = -0.5*(Cl_ZoneL2.*cosd(AOA_ZoneL2) + Cd_ZoneL2.*sind(AOA_ZoneL2));
Cm_ZoneL2 = 0 * Cm_ZoneL2;  % zeroing out; not satisfactory


% Concatenate the four zones to get sectional Cl and Cd vs AOA in the range
% [-180,180] deg
AOA0 = [flipud(AOA_ZoneL2); flipud(AOA_ZoneL1); AOA_g; AOA_ZoneH1; AOA_ZoneH2];
Cl0 = [flipud(Cl_ZoneL2); flipud(Cl_ZoneL1); Cl_g; Cl_ZoneH1; Cl_ZoneH2];
Cd0 = [flipud(Cd_ZoneL2); flipud(Cd_ZoneL1); Cd_g; Cd_ZoneH1; Cd_ZoneH2];
% Cm0 = [flipud(Cm_ZoneL2); flipud(Cm_ZoneL1); Cm_g; Cm_ZoneH1; Cm_ZoneH2];

[~,indunique] = unique(AOA0);

AOA = AOA0(indunique);
Cl = Cl0(indunique);
Cd = Cd0(indunique);
Cm = interp1([-180;-90;AOA_g;90;180], [0;0;Cm_g;0;0], AOA, 'linear');

%% STEP 2: CORRECT FOR EFFECT OF FINITE ASPECT RATIO PER REF 1

% A. Calculate drag correction factor
kcd = 1 - 0.41*(1 - exp(-17/AR));

% B. Calculate cosine weighting function
AOA_P1 = AOA_lim_pos;
AOA_N2 = 160;
w1 = max(0,cosd(180 * ((AOA(AOA>=0) - AOA_P1)/(AOA_N2 - AOA_P1)) - 90));

AOA_P1 = AOA_lim_neg;
AOA_N2 = -160;
w2 = max(0,cosd(180 * ((AOA(AOA<0) - AOA_P1)/(AOA_N2 - AOA_P1)) - 90));

w = [w2;w1];

% C. Correct lift coefficient
Cl_corr = Cl.*(1 - w.*(1-kcd));

% D. Correct drag coefficient
Cd_corr = Cd.*(1 - w.*(1-kcd));

%%
Cm_corr = Cm.*(1 - w.*(1-kcd));


SecData_out = [AOA,Cl_corr,Cd_corr,Cm_corr];


%% LOCAL FUNCTIONS
    function [AOA_out,Cl_out, Cd_out] = ApplyViterna(AOA_lim, Cl_lim, Cd_lim)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
        Cd_max = 2;
        
        
        B1 = Cd_max;
        B2 = (Cd_lim - Cd_max * (sind(AOA_lim))^2)/cosd(AOA_lim);


        A1 = B1/2;
        A2 = (Cl_lim - Cd_max * sind(AOA_lim)*cosd(AOA_lim)) * sind(AOA_lim) / (cosd(AOA_lim))^2;
        
        
        % generate Cl and Cd on right side of high stall
        nAOAviterna = 50;
        AOA_out = linspace(AOA_lim,90*sign(AOA_lim),nAOAviterna)';
        Cl_out = A1 * sind(2*AOA_out) + A2 .* (cosd(AOA_out)).^2 ./ sind(AOA_out);
        Cd_out = B1 * (sind(AOA_out)).^2 + B2 * cosd(AOA_out);
        
    end


    function [Cl_out, Cd_out] = FlatPlateTheory(AOA_in,Cd0)
        
       Cl_out = 2*sind(AOA_in).*cosd(AOA_in);
       Cd_out = Cd0 + (2-Cd0)*(sind(AOA_in)).^2;
        
    end


        
%         figure
% hold on
% subplot(1,3,1)
% hold on
% plot(AOA,Cl,'blue')
% plot(AOA_g,Cl_g,'red')
% plot(AOA,Cl_corr,'m')
% hold off
% 
% subplot(1,3,2)
% hold on
% plot(AOA,Cd,'blue')
% plot(AOA_g,Cd_g,'red')
% plot(AOA,Cd_corr,'m')
% 
% subplot(1,3,3)
% hold on
% plot(AOA,Cm_corr)
% 
% figure
% plot(AOA,w)
% 
% stopper = 1;



end
