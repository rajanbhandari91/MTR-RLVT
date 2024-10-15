function W_FC_lb = EvalFCWeight_lb(L_fus,b_ft,nult,W_dg)
   % Reviewed 11/19/21 2132 hrs
   % Flight Controls Weight
   %% Nomenclature
   % L_fus = Length of Fuselage [ft]
   % b_ft = Length of total Wing span [ft]
   % nult = Ultimate Load Factor
   % W_dg = Design Flight Gross Weight (We usually assume WTO) [lb]


   W_FC_lb = 0.053*(L_fus^1.536)*...                                            % Raymer, pg 576
        (b_ft^0.371)*(nult*W_dg*1e-4)^0.80;
end