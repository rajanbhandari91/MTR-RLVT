function [TMpq]=TranslateMoment(p,q,X0,Y0,xy,Polygon)
%
% function [TMpq, Polygon]=TranslateMoment(p,q,X0,Y0,xy)
% This function computes the translated area moments of order (p,q)
% defined as M(p,q)=integral( (x+X0)^p * (y+Y0)^q * dxdy)
% see Formula (2.6) from Marin's paper:
%
% Reference: Marin, Joaquin, 1984,
%            "Computing Columns, Footings and Gates through Moments of Area,"
%            Computers & Structures, Vol. 18., Nr. 2, pp. 343-349,1984.
%
% xy is a Nx2 matrix with coordinates [x y] in each row. There are N points
% The output is the translated area moment of order (p,q) called TMpq
% Polygon is an optional argument. If supplied, it is assumed that it has
% all the moments required for the translation (p,q).
% If Polygon is not supplied, moments are computed by calling
% the function PolygonMoments (part of this package)
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
%

TMpq=0;
for m=0:p
    C1=nchoosek(p,m)*X0^(p-m);
    for n=0:q
        if nargin<6
            Polygon = PolygonMoments(xy,[m,n],0);
        end
        name=['M_',num2str(m),'_',num2str(n)];
        TMpq=TMpq+C1*nchoosek(q,n)*Y0^(q-n)*Polygon.(name);
    end
end

end
