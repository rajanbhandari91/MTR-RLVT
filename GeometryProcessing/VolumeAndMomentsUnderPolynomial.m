function [Vol, VolMom_x, VolMom_y, PressureCenter_X, PressureCenter_Y, Polygon]=VolumeAndMomentsUnderPolynomial(Amn,xy)
%
% function [Vol, VolMom_x, VolMom_y, Polygon]=VolumeAndMomentsUnderPolynomial(Amn,xy)
% Given a bivariate polynomial as Polygon = sum (Amn*x^m*y^n)
% where Amn is a matrix with coefficients and
% xy is a Nx2 matrix with coordinates [x y] in each row. 
% xy is the set of points that describe the area where the polynomial acts upon,
% this function computes 
%   - The volume between the polynomial and the X-Y plane:
%     Vol = integral(sum(Amn*x^m*y^n))dxdy
%   - The first moment of the volume with respect to the x axis:
%     VolMon_x=integral(y*sum(Amn*x^m*y^n))dxdy
%   - The first moment of the volume with respect to the y axis:
%     VolMon_y=integral(x*sum(Amn*x^m*y^n))dxdy
%   - PressureCenter_X and PressureCenter_X: the coordinates of the pressure center
%     of the polynomial on the XY plane.
%   - Polygon = structure with the results from PolygonMoments.m
%
% This funtion uses formulas from this reference:
% Reference: Marin, Joaquin, 1984, 
%            "Computing Columns, Footings and Gates through Moments of Area,"
%            Computers & Structures, Vol. 18., Nr. 2, pp. 343-349,1984.
%
% Example:
% if the polynomial is P = 5 + 10*y + 20*x - 30*x*y + 40 * x^2y^3
% which is the same as P= 5*x^0*y^0 + 10*x^0*y^1 + 20*x^1*y^0 - 30*x^1*y^1 + 40*x^2*y^3
% Then, because matrix indices cannot be zero in Matlab, we need to add 1 to indices
% Amn(1,1)=5; Amn(1,2)=10; Amn(2,1)=20; Amn(2,2)=-30; Amn(3,4)=40;
% resulting in 
% Amn =[5    10    0    0
%      20   -30    0    0
%       0     0    0   40 ];  
% % the polynomial is acting on an rectangular area of 10x20: 
% xy=[0,0; 10, 0; 10,20;0,20]; 
% [Vol, VolMom_x, VolMom_y, Polygon]=VolumeAndMomentsUnderPolynomial(Amn,xy)
% resulting in 
% Vol =  533074333.333333
% VolMom_x =  8529810000.00000
% VolMom_y =  3998238333.33333
% PressureCenter_X =  7.50033922723723
% PressureCenter_Y =  16.0011643154207
%
%
% Software written by: Ciro A. Soto, <ciro@kavyata.com>
% Created: 2015-11-22 

% Copyright (c) 2015, Ciro A. Soto
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without modification,
% are permitted provided that the following conditions are met:
% 
% 1. Redistributions of source code must retain the above copyright notice, this 
% list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright notice, 
% this list of conditions and the following disclaimer in the documentation 
% and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
% USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% determine what polygonal area moments are needed
MaxM=size(Amn,1);
MaxN=size(Amn,2);

mn=[];
for iM=1:MaxM % add 1 to compute moments wrt to Y
for iN=1:MaxN % add 1 to compute moments wrt to X
if Amn(iM,iN) ~=0
mn=[mn;
iM-1,iN-1;
iM-1,iN;
iM,iN-1];

end
end
end
mn=unique(mn,'rows'); % get rid of repeated moments

% compute the polygonal area moments
Polygon=PolygonMoments(xy,mn);


% compute the volume
Vol=0;
VolMom_x=0;
VolMom_y=0;
for iM=1:MaxM
for iN=1:MaxN
if Amn(iM,iN) ~=0
nameV=['M_',num2str(iM-1),'_',num2str(iN-1)]; 
Vol=Vol+Amn(iM,iN)*Polygon.(nameV);

nameX=['M_',num2str(iM-1),'_',num2str(iN)]; 
VolMom_x=VolMom_x+Amn(iM,iN)*Polygon.(nameX);

nameY=['M_',num2str(iM),'_',num2str(iN-1)]; 
VolMom_y=VolMom_y+Amn(iM,iN)*Polygon.(nameY);

end
end
end

% the pressure center of the polynomial on the XY plane has coordinates:
PressureCenter_X=VolMom_y/Vol;
PressureCenter_Y=VolMom_x/Vol;