function Polygon = PolygonMoments (xy,mn,PlotFlag)
%
% function Polygon = PolygonMoments (xy,mn,PlotFlag)
%
% It computes commonly used area moments of a planar polygon defined by its vertices.
% The polygon can be simple, or disconnected with multiple polygons.
%
% The function also computes 
%     - area centroid (AC)
%     - area moments at the AC
%     - angle of the principal axes
%     - area moments in the principal axes
%     - giration radius in each axis
%     - coordinates of the original vertices in the rotated principal axes.
%
% The function also computes 
%     - area moments of order (m,n)
%       defined as M(m,n)=integral( x^m * y^n * dxdy). 
%       See equation 2.5 in Marin's paper:
%
%       Reference: Marin, Joaquin, 1984,
%                 "Computing Columns, Footings and Gates through Moments of Area,"
%                 Computers & Structures, Vol. 18., Nr. 2, pp. 343-349,1984.
%
% The area to be integrated is defined by vertices in matrix xy.
% xy is a Nx2 matrix with coordinates [x y] in each row. There are N points
%
% mn is a Kx2 matrix with [m,n] pairs in each row. There are K rows
% if mn is empty, the function computes only the commonly used area moments
%
% PlotFlag controls the graphic output
%  PlotFlag  ~= 0 : plots the figure
% |PlotFlag| == 1 : plots the figure only
% |PlotFlag| == 2 : plots the figure and legend
% |PlotFlag| == 3 : plots the figure, vertices (and id if PlotFlag<0)
% |PlotFlag| == 4 : plots the figure, legend, vertices (and id if PlotFlag<0)
%
% The ouput is a structure with fields containing all results described above.
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

% handling input
if nargin==0
error('*** PolygonMoments: missing all arguments')
elseif nargin==1
    mn=[];
    PlotFlag=0;
elseif nargin<=2
    PlotFlag=0;
elseif nargin>3
    error('*** PolygonMoments: Number of arguments > 3')
end
if PlotFlag<0, NoIds=1;PlotFlag=abs(PlotFlag);else NoIds=0;end
%
N=size(xy,1);
x=xy(:,1);
y=xy(:,2);

% make sure the last point is the first too
x=[x;x(1)];
y=[y;y(1)];

% Compute common values and Area Centroid (AC) always
wi=x(1:N).*y(2:N+1)-x(2:N+1).*y(1:N);

% area of the polygon
Polygon.Area=0.5*sum(wi);

% area moments
Polygon.MAx=1/6 *sum(wi.*(y(1:N) + y(2:N+1)));
Polygon.MAy=1/6 *sum(wi.*(x(1:N) + x(2:N+1)));

% Area moments of second degree (Inertia)
Polygon.Ixx=1/12*sum(wi.*(  (y(1:N)+y(2:N+1)).^2 - y(1:N).*y(2:N+1)));
Polygon.Iyy=1/12*sum(wi.*(  (x(1:N)+x(2:N+1)).^2 - x(1:N).*x(2:N+1)));
Polygon.Ixy=1/24*sum(wi.*( (x(1:N)+x(2:N+1)).*(y(1:N)+y(2:N+1))  + x(1:N).*y(1:N) + x(2:N+1).*y(2:N+1) ));
Polygon.Izz=Polygon.Ixx+Polygon.Iyy;

% giration radius in each axis
Polygon.XGirationRadius=(Polygon.Ixx/Polygon.Area)^0.5;
Polygon.YGirationRadius=(Polygon.Iyy/Polygon.Area)^0.5;
Polygon.ZGirationRadius=(Polygon.Izz/Polygon.Area)^0.5;

% coordinates of the area centroid:
Polygon.ACx=Polygon.MAy/Polygon.Area;
Polygon.ACy=Polygon.MAx/Polygon.Area;

% area moments in the area centroid:
Polygon.IxxAC=Polygon.Ixx-Polygon.ACy^2*Polygon.Area;
Polygon.IyyAC=Polygon.Iyy-Polygon.ACx^2*Polygon.Area;
Polygon.IxyAC=Polygon.Ixy-Polygon.ACx*Polygon.ACy*Polygon.Area;
Polygon.IzzAC=Polygon.IxxAC+Polygon.IyyAC;

% giration radius in the area centroid
Polygon.kxx=(Polygon.IxxAC/Polygon.Area)^0.5;
Polygon.kyy=(Polygon.IyyAC/Polygon.Area)^0.5;
Polygon.kzz=(Polygon.IzzAC/Polygon.Area)^0.5;
Polygon.kxy=(Polygon.IxyAC/Polygon.Area)^0.5;

Polygon.PrincAxesRotationDeg=0.5*atan(2*Polygon.IxyAC/(Polygon.IyyAC-Polygon.IxxAC))*180/pi;
B=-(Polygon.IxxAC+Polygon.IyyAC);C=Polygon.IxxAC*Polygon.IyyAC-Polygon.IxyAC^2;
Polygon.MaxInertiaPrincAxes=(-B+sqrt(B^2-4*C))/2;
Polygon.MinInertiaPrincAxes=(-B-sqrt(B^2-4*C))/2;


% % Formula (2.5) from Marin's paper for M(m,n)
% if ~isempty(mn)
%     for imn=1:size(mn,1)
%         m=mn(imn,1);
%         n=mn(imn,2);
%         Si=0;
%         for i=1:N
%             Sjk=0;
%             for j=0:m
%                 for k=0:n
%                     Sjk=Sjk+nchoosek(j+k,j)*nchoosek(m+n-j-k,n-k)*x(i)^(m-j)*x(i+1)^j*y(i)^(n-k)*y(i+1)^k;
%                 end
%             end
%             Si=Si+(x(i)*y(i+1)-x(i+1)*y(i))*Sjk;
%         end
%         MomentMN=factorial(m)*factorial(n)/factorial(m+n+2)*Si;
%         
%         name=['M_',num2str(m),'_',num2str(n)];
%         Polygon.(name)=MomentMN;
%         
%     end
% end
% 
% % coordinates x,y in the principal axes
% a=Polygon.PrincAxesRotationDeg*pi/180;
% Polygon.xyInPrincAxes=(xy-repmat([Polygon.ACx,Polygon.ACy],size(xy,1),1))*[cos(a),-sin(a);sin(a),cos(a)];


% plotting (if requested)
if PlotFlag
    figure;
    % plot area in gray
    patch(x,y,0.7*[1 1 1],'LineStyle','none')
    ax=axis;dx=ax(2)-ax(1);dy=ax(4)-ax(3);
    grid on;hold on
    axis([ax(1)-0.01*dx, ax(2)+0.01*dx, ax(3)-0.01*dy, ax(4)+0.01*dy])
    ax2=axis;
    axis equal
    axe=axis;
    axis(ax2);
    % plot original axes
    plot(ax2(1:2),[0 0],'-m','linewidth',3)
    plot([0 0],ax2(3:4),'-m','linewidth',3)
    a=Polygon.PrincAxesRotationDeg*pi/180;
    xcg=Polygon.ACx;
    ycg=Polygon.ACy;
    s=tan(a);
    xx=ax2(1:2);
    yy=s*(xx-xcg)+ycg;
    
    % plot principal axes
    plot(xx,yy,'-b','linewidth',3)
    yy=-1/s*(xx-xcg)+ycg;
    plot(xx,yy,'-b','linewidth',3)
    
    % plot area centroid AC
    plot(Polygon.ACx,Polygon.ACy,'or','markerfacecolor','r','markersize',10)
    text(Polygon.ACx,Polygon.ACy,' AC','fontsize',16);
    
    % print results on the figure
    if PlotFlag==2 || PlotFlag==4
        xlim([ax2(1),ax2(2)+(ax2(2)-ax2(1))*0.1])
        L=fieldnames(rmfield(Polygon,'xyInPrincAxes'));
        leg='';
        for iL=1:numel(L)
            leg=[leg,sprintf('%s = %.2f\n',L{iL},Polygon.(L{iL}))];
        end
        text(max(xy(:,1))*1.01,mean(ax2(3:4)),leg,'backgroundcolor','w');
    end
    
    % plot each vertice with a circle
    if PlotFlag==3 || PlotFlag==4
        plot(x,y,'ok','markersize',2)
        if ~NoIds
        for i=1:N
            text(x(i),y(i),mat2str(i),'fontsize',16,'color','k');
        end
        end
    end
    axis(axe);
    
end


end




