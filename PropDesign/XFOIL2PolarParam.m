function PropParam = XFOIL2PolarParam(Foil_Data)

[alpha, sort_ind] = sort(Foil_Data(:,1));
CL = Foil_Data(sort_ind,2);
CD = Foil_Data(sort_ind,3);
CDp = Foil_Data(sort_ind,4);
CM = Foil_Data(sort_ind,5);

nf = numel(findobj(0,'type','figure'));

% figure(nf + 1)
% plot(alpha,CL)
% grid on
% xlabel '\alpha'
% ylabel 'C_L'
% 
% figure(nf + 2)
% plot(CD,CL)
% grid on
% xlabel 'C_D'
% ylabel 'C_L'
% 
% figure(nf + 3)
% plot(alpha,CD)
% grid on
% xlabel '\alpha'
% ylabel 'C_D'

%% Lift Curve Parameters

alpha_lim = 5;
PropParam.CL0 = CL(alpha == 0);
del_alpha = linspace(1,alpha_lim,20);
del_CL = zeros(size(del_alpha));
for i = 1:numel(del_alpha)
    CL_temp = interp1(alpha,CL,del_alpha(i));
    if length(CL_temp) ~= length(PropParam.CL0)
        fprintf('\n\n CL0 had to be interpolated from "ALPHA = 0.1,0.2" since no value existed for "ALPHA = 0" \n\n')
        PropParam.CL0 = -(CL(alpha==0.2)-CL(alpha==0.1))+CL(alpha==0.1);                                             %Linear Interpolation, change if the interval is changed in XFOIL input
        del_CL(i) = CL_temp - PropParam.CL0;
    else
    del_CL(i) = CL_temp - PropParam.CL0;    
    end
end

% CL_a = del_CL./deg2rad(del_alpha);
% PropParam.CL_a = mean(CL_a);

%% Drag Polar Parameters
[PropParam.CL_min, min_idx] = min(CL);
[PropParam.CL_max, max_idx] = max(CL);

PropParam.AOACLmax = alpha(max_idx);
PropParam.CL_a = (180/pi)*(PropParam.CL_max - PropParam.CL0)/(PropParam.AOACLmax - 0);


[value,idx] = (min(abs(CL)));
PropParam.CL0AOA = Foil_Data(idx,1);
[PropParam.CD0, CD0_ind] = min(CD);
PropParam.CLCD0 = CL(CD0_ind);
PropParam.REexp = -0.5;
delCL_u = abs(PropParam.CL_max - PropParam.CLCD0);
delCL_l = abs(PropParam.CL_min - PropParam.CLCD0);

CL3_2 = (CL.^1.5)./CD;
[value,IDCL3_2] = max(CL3_2);
PropParam.CL3_2 = CL(IDCL3_2);

delCD_u = abs(CD(max_idx) - PropParam.CD0);
delCD_l = abs(CD(min_idx) - PropParam.CD0);

PropParam.CD2u = delCD_u/delCL_u^2;
PropParam.CD2l = delCD_l/delCL_l^2;

% %
% nf = 0;
% figure(nf+1)
% hold on
% plot(alpha,PropParam.CL_a * deg2rad(alpha) + PropParam.CL0)
% 
% CL_plot = linspace(PropParam.CL_min,PropParam.CL_max);
% CLu = CL_plot(CL_plot > PropParam.CLCD0) - PropParam.CLCD0;
% CLl = CL_plot(CL_plot < PropParam.CLCD0) - PropParam.CLCD0;
% figure(nf+2)
% hold on
% plot(PropParam.CD0 + PropParam.CD2u*CLu.^2,CL_plot(CL_plot >PropParam.CLCD0))
% plot(PropParam.CD0 + PropParam.CD2l*CLl.^2,CL_plot(CL_plot <PropParam.CLCD0))
% 
