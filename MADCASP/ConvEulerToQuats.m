function [e0,e1,e2,e3,E] = ConvEulerToQuats(ph,th,ps)

e0 = cos(ph/2)*cos(th/2)*cos(ps/2) + sin(ph/2)*sin(th/2)*sin(ps/2);

e1 = sin(ph/2)*cos(th/2)*cos(ps/2) - cos(ph/2)*sin(th/2)*sin(ps/2);

e2 = cos(ph/2)*sin(th/2)*cos(ps/2) + sin(ph/2)*cos(th/2)*sin(ps/2);

e3 = cos(ph/2)*cos(th/2)*sin(ps/2) - sin(ph/2)*sin(th/2)*cos(ps/2);

E = [e0;e1;e2;e3];