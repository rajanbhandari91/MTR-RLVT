function [Ixx,Iyy,x_section,Q_NA,A] = MOI_Calc(b,h,tw,flange,flag)

y_bar = h/2;

if flag == 1

    x_section = 'hollow rect';
    Ixx = 1/12*(b*h^3 - (b-2*tw)*(h-2*tw)^3); %(un^4)
    Iyy = 1/12*(h*b^3 - (h-2*tw)*(b-2*tw)^3); %(un^4)
    Q_NA = 2*((1/2*h-tw)*tw)*(y_bar - tw) + b*tw*(y_bar - tw/2); %(un^3)
    A = b*h - (b-2*tw)*(h-2*tw); %(un^2)

elseif flag == 2

    x_section = 'c-channel';
    A = 2*flange*tw + (h - 2*tw)*tw; %(un^2)
    x_c = 1/A*(1/2*(h - 2*tw)*tw^2 + tw*flange^2);
    Ixx = flange*h^3/12 - 1/12*(flange - tw)*(h - 2*tw)^3; %(un^4)
    Iyy = (1/3*(h - 2*tw)*tw^3 + 2/3*tw*flange^3) - A*x_c^2; %(un^4)
    Q_NA = (h/2*tw)*(y_bar + h/2) + (flange*tw)*(y_bar - tw/2); %(un^3)


elseif flag == 3

    x_section = 'hollow circle';
    Ixx = pi/64*(b^4 - (b-tw)^4);
    Iyy = Ixx;

elseif flag == 4

    x_section = 'I-beam';
    A = 2*b*tw + (h - 2*tw)*tw; %(un^2)
    Ixx = (h - 2*tw)^3*tw/12 + 2*(tw^3*b/12 + tw*tw((h - 2*tw) + tw)^2/4); %(un^4)
    Iyy = tw^3*(h - 2*tw)/12 + 2*(b^3*tw/12); %(un^4)
    Q_NA = 0;

end



end