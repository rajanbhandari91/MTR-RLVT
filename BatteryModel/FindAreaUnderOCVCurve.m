function [int_OCV_dx, x_vec, RE_cons] = FindAreaUnderOCVCurve(DOD_i, DOD_f, BattParams)


% create testing points
x_vec = linspace(DOD_i, DOD_f, 1000);
x_vec = x_vec';

% find OCV
ocv_vec = BattParams.GI_OCV_fcn_DOD(x_vec);

% find mean OCV
ocv_mean = 0.5 * (ocv_vec(1:end-1) + ocv_vec(2:end));

dx = diff(x_vec);

dA = ocv_mean.*dx;

int_OCV_dx = sum(dA);

sdA = cumsum(dA);

RE_cons = ([0; sdA])/int_OCV_dx;

end