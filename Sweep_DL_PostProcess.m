clc
clear
close

addpath( 'Results and Plots')
addpath( genpath('Results and Plots'))

Architecture = [1 2 3];

DL = 14:2:28;
WL = 50:4:80;


[DLplot,WLplot] = meshgrid(DL,WL);
color = {'k','b','g'};

for i = 1:length(Architecture)

    for j = 1:length(DL)
        for k = 1:length(WL)

            FileName = ['LPC_FAA_0inc_DL_',num2str(DL(j)),'_Archi_',num2str(Architecture(i)),'_WL_',num2str(WL(k))];
            load(FileName)
            MTOM(j,k) = Vehicle.MassProp.WBD_Paper.Mass(end);
            
            
            % check if the cases converged
            fuelConv        = abs(Vehicle.Iterations.dEnergyMasses(end,1));
            battConv        = abs(Vehicle.Iterations.dEnergyMasses(end,2));
            PowerFailure    = any(Vehicle.Mission.PointPerf.FLAG < 0);
        
            if fuelConv >= 1 || battConv >= 1 || PowerFailure == 1
                fprintf('Case DL = %.2f, WL = %.2f \n\nNot Converged or Power Failure',DL(j),WL(k));
                pause
            end

        end  
    end
    
    % figuring out the minimum of  MTOM for give DL and WL combinations
    minMTOM(i) = min(MTOM(:));
    [r,c] = find(MTOM == minMTOM(i));

    % creating carpet plot for MTOM wrt DL and WL
    surf(DLplot,WLplot,MTOM,'FaceAlpha',0.0,'EdgeColor',color{i},'Marker','.')
    hold on

    % plotting minimum MTOM on the carpet plot
    scatter3(DL(r),WL(c),minMTOM(i),'red','filled')

end


grid on
grid minor
xlabel('Disk Loading (lb/ft^2)')
ylabel('Wing Loading (lb/ft^2)')
zlabel('MTOM (kg)')

legend('AE','','HE','','AE','Location','best')
