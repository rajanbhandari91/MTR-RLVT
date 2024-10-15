function [Edisch_J] = CalcDischEnergy(DOD_i, DOD_f, CRATE, CellParams)

% cell capacity
Q_Ah = CellParams.CellCapacity_Ah;
Q_As = Q_Ah * 3600;

% calculate 1C current
i_1C = Q_Ah;

% calculate discharge current
i_vec = CRATE * i_1C;

% create testing points between DOD_i and DOD_f
x_vec = linspace(DOD_i, DOD_f, 1000);

% find OCV
ocv_vec = CellParams.GI_OCV_fcn_DOD(x_vec);

% find resistance
r0_vec = CellParams.GI_R0_fcn_DOD(x_vec);

% find terminal voltage
v_vec = ocv_vec - i_vec.*r0_vec;

% find mean voltage
v_mean = 0.5 * (v_vec(1:end-1) + v_vec(2:end));

% calculate DOD increments
dx = diff(x_vec);

% calculate integral of V.dx
int_V_dx = sum(v_mean.*dx);

% multiply by cell capacity
Edisch_J = Q_As * int_V_dx;

end
