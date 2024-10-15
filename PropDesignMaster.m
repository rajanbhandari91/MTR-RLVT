clc
clear all
close all

% % adding path and subpath to MATLAB path
addpath( 'PropDesign')
addpath( genpath('PropDesign'))

% EXCEL FILE IN WHICH THE REQUIREMENTS FOR THE ***OPEN*** ROTOR ARE SPECIFIED
% Note: The first case is the design case. Rest are off-design cases
FileName = 'PropRequirementsSetup.xlsx';


% SETUP save name
SetupSaveName = 'LiftProp.mat';


%%%%%% AC3D RELATED
% AC3D Flag. Set to 1 to generate AC3D file
GenAC3DFile = 1;
% Mean camber surface flag. Set to 1 to generate only mean surface
MCSFlag =0;
% SAVE NAME FOR AC3D FILE
AC3DSaveName = 'GenericProp.ac';


%% Running a sweep of cases for different Disc Loadings and Diameters

[PropData] = ReadSetupFile(FileName);

DL = PropData.PropDef.DL;         % Disk Loading (Kg/m^2)
Diam = PropData.PropDef.r_t * 2;             % Diameters (m)
% DL = 29.68;
clear PropData

% Correction factors scalers were computed from EvaRotor for DLs 
% DL = [60    80   100   120   140  ];
% T_scaler = [ 0.9800    0.9500    0.9300    0.9100    0.8900    ];
% Q_scaler = [1.0600    1.0500    1.0300    1.0000    0.9900    ];

T_scaler = [ 0.9600    0.9200    0.88   0.86    0.82    ];
Q_scaler = [1.0700    1.0600    1.0400    1.04    0.99    ];

for ii = 1:1:length(DL)

    for jj = 1:1:length(Diam)

        [PropData] = ReadSetupFile(FileName);
        PropData.PropDef.r_t = 0.5*Diam(jj);
        Mass_kg = pi*PropData.PropDef.r_t^2 * DL(ii);
        PropData.Cases.Mass_kg = Mass_kg*ones(length(PropData.Cases.Mass_kg),1);

        CSPropDesigner

        % collecting data at specified radial locations
        r = PropDef.HubRadiusRatio:0.0207:1;
%         r = linspace(0.02,0.2287,42);
        c_R = interp1(QMIL.r_R,QMIL.c_R,r,'Linear','extrap')';
        beta = interp1(QMIL.r_R,QMIL.beta,r,'Linear','extrap')';

        % zero out the beta at 75% radius
        beta = beta - interp1(QMIL.r_R,QMIL.beta,0.75,'Linear')';
        PropTable.beta_R(ii,:) = beta;
        PropTable.c_R(ii,:) = c_R;
        

        % plotting normalized twist and chord distribution
        subplot(2,1,1)
        plot(r, c_R)
        xlabel('normalized radius (r/R)')
        ylabel('chord (c/R)')
        hold on

        subplot(2,1,2)
        plot(r, beta)
        hold on
        xlabel('normalized radius (r/R)')
        ylabel('twist (\beta/R)')
        

    end
    clear PropData
end


close all

%% Create and save griddedInterpolant as a function
% c_R = f(DL,r)
% beta_R = f(DL,r)


% Run this section for LiftProp. The DL is constant for cruiseprop, so we don't need the interpolant    

% GI_ThrustScaler_DL = griddedInterpolant(DL,T_scaler);
% GI_TorqueScaler_DL = griddedInterpolant(DL,Q_scaler);
% GI_beta_R = griddedInterpolant({DL',r},PropTable.beta_R);
% GI_c_R = griddedInterpolant({DL',r},PropTable.c_R);
% save('LP_DataBase.mat','GI_c_R','GI_beta_R','GI_TorqueScaler_DL','GI_ThrustScaler_DL');
% save('LP_DataBase.mat','GI_c_R','GI_beta_R');
savefig('PropDesign\Twist&ChordDistribution.fig');







%% as a function of diameter as well

%         for kk = 1:length(r)
%             PropTable.Diam(ii,jj,kk) = Diam(jj);
%             PropTable.DL(ii,jj,kk) = DL(ii);
% 
%             PropTable.r(ii,jj,kk) = r(kk);
%             PropTable.c_R(ii,jj,kk) = c_R(kk);
%             PropTable.beta_R(ii,jj,kk) = beta(kk);
%         end
% 
%         for kk = 1:length(r)
%             PropTable.DL(ii,kk) = DL(ii);
% 
%             PropTable.r(ii,kk) = r(kk);
%             PropTable.c_R(ii,kk) = c_R(kk);
%             PropTable.beta_R(ii,kk) = beta(kk);
%         end
% P = [1 2 3];
% DiamGrid = permute(PropTable.Diam, P);
% DLGrid = permute(PropTable.DL, P);
% rGrid = permute(PropTable.r, P);
% c_RGrid = permute(PropTable.c_R, P);
% betaGrid = permute(PropTable.beta_R, P);
% 
% GI_c_R = griddedInterpolant(DLGrid, DiamGrid, rGrid, c_RGrid);
% GI_beta = griddedInterpolant(DLGrid, DiamGrid, rGrid, betaGrid);
% 
















