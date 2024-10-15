clc
clear
close

tic

addpath( 'Results and Plots')
addpath( genpath('Results and Plots'))

Architecture = [3];


% Folder where the sweep results get saved
SaveResults = ['Results and Plots/FullFactorial_Archi_',num2str(Architecture),'/'];

% % Creating combinations of DL,WL,Vcruise
DL        = [10 15 20 25 30];                  % Disk Loading (lb/ft^2)
WL        = [30 40 50 60];                     % Wing Loading (lb/ft^2)
Vcruise   = [175 200 225];                     % Cruise Speed (kts)
Range     = [80 100 120 140];                  % Range (km)


% Increase Range for HE and TE cases
if Architecture == 2 || Architecture == 3
    Range     = [80, 150, 200, 250];         % Range (km)
end

KWhKG     = [0.30 0.35 0.40];
CRate     = [3 5 7];

% % Fix Energy density and CRate for Turbo electric
if Architecture == 3
    KWhKG     = [0.40];
    CRate     = [ 5 ];
end


S = allcomb(DL,WL,Vcruise,Range,KWhKG,CRate);
[n,~] = size(S);




%% Running the Sweep to gather FF data
unconCtr = 0;
ConCtr = 0;
maxedOutCtr = 0;

for i = 1:n

    FF.DL                   = S(i,1);
    FF.WL                   = S(i,2);
    FF.Vcruise              = S(i,3);
    FF.Range                = S(i,4);
    FF.EnergyDensity_kWhkg  = S(i,5);
    FF.MaxCRate             = S(i,6);

    FileName = [SaveResults,'LPC_FAA_DL_',...
        num2str(FF.DL),'_WL_',num2str(FF.WL),'_VC_',num2str(FF.Vcruise),'_Range_',num2str(FF.Range),'_KWhKG_',num2str(FF.EnergyDensity_kWhkg),'_CRate_',num2str(FF.MaxCRate)...
        '_Archi_',num2str(Architecture),'.mat'];

    % Try catch so it doesn't throw error if any cases are not found in the
    % results folder
    try
        load(FileName)

        % check if the cases converged

        %  Cases which maxed out
        if Vehicle.Iterations.Iter(end) == 22

            maxedOutCtr = maxedOutCtr + 1;

            maxedOut.DL(maxedOutCtr,:) = FF.DL;
            maxedOut.WL(maxedOutCtr,:) = FF.WL;
            maxedOut.Vcruise(maxedOutCtr,:) = FF.Vcruise;
            maxedOut.Range(maxedOutCtr,:) = FF.Range;
            maxedOut.EnergyDensity_kWhkg(maxedOutCtr,:) = FF.EnergyDensity_kWhkg;
            maxedOut.MaxCRate(maxedOutCtr,:) = FF.MaxCRate;
            maxedOut.MTOM(maxedOutCtr,:) = Vehicle.MassProp.MTOM_kg;
            maxedOut.EEM(maxedOutCtr,:) = Vehicle.Iterations.EEM(end);
            maxedOut.dEnergyMassesFuel(maxedOutCtr,:) = Vehicle.Iterations.dEnergyMasses(end,1);
            maxedOut.dEnergyMassesBatt(maxedOutCtr,:) = Vehicle.Iterations.dEnergyMasses(end,2);

        end

        % Unconverged cases for MTOM going crazy
        if Vehicle.Iterations.Conv(end) == -1
            unconCtr = unconCtr + 1;

            UnConv.DL(unconCtr,:) = FF.DL;
            UnConv.WL(unconCtr,:) = FF.WL;
            UnConv.Vcruise(unconCtr,:) = FF.Vcruise;
            UnConv.Range(unconCtr,:) = FF.Range;
            UnConv.EnergyDensity_kWhkg(unconCtr,:) = FF.EnergyDensity_kWhkg;
            UnConv.MaxCRate(unconCtr,:) = FF.MaxCRate;
            UnConv.MTOM(unconCtr,:) = Vehicle.MassProp.MTOM_kg;
            UnConv.EEM(unconCtr,:) = Vehicle.Iterations.EEM(end);
            UnConv.dEnergyMassesFuel(unconCtr,:) = Vehicle.Iterations.dEnergyMasses(end,1);
            UnConv.dEnergyMassesBatt(unconCtr,:) = Vehicle.Iterations.dEnergyMasses(end,2);

        else

            % Converged cases
            ConCtr = ConCtr + 1;

            Tbl.DL(ConCtr,:) = FF.DL;
            Tbl.WL(ConCtr,:) = FF.WL;
            Tbl.Vcruise(ConCtr,:) = FF.Vcruise;
            Tbl.Range(ConCtr,:) = FF.Range;
            Tbl.EnergyDensity_kWhkg(ConCtr,:) = FF.EnergyDensity_kWhkg;
            Tbl.MaxCRate(ConCtr,:) = FF.MaxCRate;
            Tbl.MTOM(ConCtr,:) = Vehicle.MassProp.MTOM_kg;
            Tbl.EEM(ConCtr,:) = Vehicle.Iterations.EEM(end);
            Tbl.dEnergyMassesFuel(ConCtr,:) = Vehicle.Iterations.dEnergyMasses(end,1);
            Tbl.dEnergyMassesBatt(ConCtr,:) = Vehicle.Iterations.dEnergyMasses(end,2);
            Tbl.NumIter(ConCtr,:) = Vehicle.Iterations.Iter(end);

        end

    catch ERRMSG
    end

end

maxedOutTbl = struct2table(maxedOut);
UnConvTbl = struct2table(UnConv);
Tbl = struct2table(Tbl);

SaveName = ['Sweep_Archi_',num2str(Architecture)];
save(SaveName,'Tbl')

time = toc;

