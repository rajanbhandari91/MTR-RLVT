function [AOAi,InducedDrag, Lift] = CalcInducedAOA(scalefactor,aero,AOA,StripVel,a,symm,Root,Tip, rho)

StripROM = aero ;
nStrips = height(StripROM(:,1));



Strips.Y      = StripROM(:,1);
Strips.dy     = StripROM(:,2);
Strips.c0     = StripROM(:,3);
Strips.dCdA   = StripROM(:,4);
Strips.IF     = StripROM(:,5);
CDi_3DCorr    = StripROM(1,6);

% Calculating the dA wrt to the reference -5 degree
dA = AOA + 5;                           % 5 AoA (specified angle) - (-5 AoA (baseline) )

%% COMPUTE CIRCULATION GRADIENTS FOR EACH STRIP "dCdY"

c10 = (Strips.c0(1:nStrips - 1) + Strips.dCdA(1:nStrips - 1).*dA(1:nStrips - 1)).*StripVel(1:nStrips - 1);
c20 = (Strips.c0(2:nStrips) + Strips.dCdA(2:nStrips).*dA(2:nStrips)).*StripVel(2:nStrips);
dy = Strips.dy(1:nStrips - 1);
dCdY = (c20 - c10)./dy;
dCdY(nStrips) = dCdY(nStrips - 1);


%% COMPUTE DOWNWASH VELOCITY FOR EACH STRIP

% Downwash = zeros(nStrips,1);
y0 = Strips.Y;

if symm == 1
    x = ((1./(Strips.Y' - Strips.Y)).* (dCdY.* Strips.dy))./(4*pi) + ...
        ((1./(Strips.Y' - ( - Strips.Y))).* (- dCdY.* Strips.dy))./(4*pi);
else
    x = ((1./(Strips.Y' - Strips.Y)).* (dCdY.* Strips.dy))./(4*pi);
end

x(isinf(x)|isnan(x)) = 0;
Downwash = sum(x)';



% % For Root and Tip Correction
%

%     Compute the circulation values of the start and end strips.
y1 = Strips.Y(1);
y2 = Strips.Y(nStrips);

%     compute the circulation values of the start and end strips.
c0 = Strips.c0(1);
dCdA = Strips.dCdA(1);

c1 = (c0 + dCdA*dA(1)).*StripVel(1);

c0 = Strips.c0(nStrips);
dCdA = Strips.dCdA(nStrips);

c2 = (c0 + dCdA*dA(nStrips)).*StripVel(nStrips);


% % Starting strip vortex contribution
%     (Note: don't add this if the
%     starting strip is connected to some other non-lifting component like fuselage, pod nacelle etc.)
% 

if Root == 1
    Downwash(1) = Downwash(1) + ((2./(Strips.dy(1)))*c1)/(4*pi);
    Downwash(2:end) = Downwash(2:end) + ((1./((y0(2:end) - y1)))*c1)/(4*pi);
    if symm == 1
        Downwash = Downwash + ((1./(y0 + y2))*c2)/(4*pi);
    end
end


% % Ending strip vortex contribution
%     (Note: don't add this if the end strip
%      is connected to some other non-lifting component like fuselage, pod nacelle etc.)


if Tip == 1
    Downwash(1:nStrips-1) = Downwash(1:nStrips-1) - ((1./((y0(1:nStrips-1) - y2)))*c2)/(4*pi);
    Downwash(nStrips) = Downwash(nStrips) + ((2./(Strips.dy(nStrips)))*c2)/(4*pi);
    if symm == 1
        Downwash = Downwash + ((1./(y0 + y2))*c2)/(4*pi);
    end
end




% Total Downwash After adding the root/tip correction
Strips.Downwash = Downwash;


%% COMPUTE INDUCED LIFT & DRAG FROM ENTIRE COMPONENT

circulation = (Strips.c0 + Strips.dCdA.*dA).*StripVel .*scalefactor ;
% rho = 1.225;
dw = Strips.Downwash;
dy0 = Strips.dy;

%% Compressibility Correction
M = StripVel ./ a;
Beta = sqrt(1 - M.^2);

Lift_Strip = (rho.*StripVel.*circulation.*dy0)./Beta;                                        
DragInd = (rho.*dw.*circulation.*dy0.*Strips.IF)./Beta;   


%% zero out the negative induced drags that might occur at some lower AOA
% numbers
ii = find(DragInd < 0);
DragInd(ii) = 0.0000;


% % Induced AOA
AOAInd = atan2d(dw,StripVel + 0.0000000000000001);
% AOAInd = atand((dw)./StripVel);

% Check the indices for +ve Strips.Y and calculate the induced drag and
% AOAi only for those strips. This is the way to decouple the loads on left
% and right side of the lifting surfaces which are continuous along the
% plane of symmetry (For example: Left and Right Horz tail of a T-Tail configuration)
% 
% NOTE: the Strips.Y will always be +ve for the lifting surfaces of interest. This is
% defined by the way the axis system is organized when collecting the ROM
% data. 

jj = find(Strips.Y >= 0);
InducedDrag = CDi_3DCorr.*DragInd(jj);

Lift = Lift_Strip(jj);
AOAi = AOAInd(jj);

stopper = 1;

end