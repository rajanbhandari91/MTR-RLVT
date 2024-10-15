function [R_BS_out, R_SB_out] = ApplySingleAxisRotation(R_BS, th, axindex)



[R_BS_11, R_BS_12, R_BS_13, R_BS_21, R_BS_22, R_BS_23, R_BS_31, R_BS_32, R_BS_33]...
        = deal(R_BS(:,1),R_BS(:,2),R_BS(:,3),R_BS(:,4),R_BS(:,5),R_BS(:,6),R_BS(:,7),R_BS(:,8),R_BS(:,9));


sind_th = sind(th);
cosd_th = cosd(th);

% convert necessary elements of R_BS
R_BS_11_new = R_BS_11.*cosd_th                - R_BS_13.*sind_th;
R_BS_12_new =                      R_BS_12;
R_BS_13_new = R_BS_11.*sind_th                + R_BS_13.*cosd_th;

R_BS_21_new = R_BS_21.*cosd_th                - R_BS_23.*sind_th;
R_BS_22_new =                      R_BS_22;
R_BS_23_new = R_BS_21.*sind_th                + R_BS_23.*cosd_th;


R_BS_31_new = R_BS_31.*cosd_th                - R_BS_33.*sind_th;
R_BS_32_new =                      R_BS_32;
R_BS_33_new = R_BS_31.*sind_th                + R_BS_33.*cosd_th;

R_BS_out = [R_BS_11_new, R_BS_12_new, R_BS_13_new, R_BS_21_new, R_BS_22_new, R_BS_23_new, R_BS_31_new, R_BS_32_new, R_BS_33_new];


CaptureOrder = [1, 4, 7, 2, 5, 8, 3, 6, 9];

R_SB_out = R_BS_out(:, CaptureOrder);