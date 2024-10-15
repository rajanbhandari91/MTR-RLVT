function [AF1, AF2] = InterpAF(AF1, LS, Index)

% specified second airfoilname
AF2 = ReadAF(LS, Index);

% takes airfoil coordinates for two airfoils and interpolates second
% airfoil based on the first airfoil X-coordinates

% extract AF - coordiantes from 1st airfoil AF1 
% find origin index
[~, idx1] = min(abs(AF1(1:end,1)));
% for Upper Surface
US_1 = AF1(1:idx1, :); 
% for lower surface
LS_1 = AF1(idx1:end, :);

% extract AF - coordiantes from 2nd airfoil AF2 
% find origin index
[~, idx2] = min(abs(AF2(1:end,1)));
% for Upper Surface
US_2 = AF2(1:idx2, :);  
% for lower Surface
LS_2 = AF2(idx2:end, :); 


% Interpolate
US_AF2 = interp1(US_2(:,1), US_2, US_1(:,1));

LS_AF2 = interp1(LS_2(:,1), LS_2, LS_1(:,1));

AF2 = [US_AF2; LS_AF2(2:end, :)];


% for i = 1:height(y1)
%     plot3(eta', AF2(i).*ones(10,1), linspace(y1(i), y2(i), 10)', '.-c')
%     hold on
% end

end

