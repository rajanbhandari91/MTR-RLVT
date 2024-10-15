clc
clear all
close

file = 'NonLifting Aero.xlsx'

AeroData = xlsread(file);
Aero.Fuselage.AOA = AeroData(:,1);
Aero.Fuselage.BETA = AeroData(:,2);
% Aero.Fuselage.V = AeroData(:,3);
Aero.Fuselage.Cx = AeroData(:,4);
Aero.Fuselage.Cy = AeroData(:,5);
Aero.Fuselage.Cz = AeroData(:,6);
% Aero.Fuselage.CL = AeroData(:,7);
% Aero.Fuselage.CDi = AeroData(:,8);
% Aero.Fuselage.CD0 = AeroData(:,9);
Aero.Fuselage.Cmx = AeroData(:,10);
Aero.Fuselage.Cmy = AeroData(:,11);
Aero.Fuselage.Cmz = AeroData(:,12);
Table = struct2table(Aero.Fuselage);

Table = sortrows(Table,2)

for i = 1:height(Table)
    if Table.BETA(i) == 0.00 && Table.Cy(i) ~= 0.00
        ii = i;
        Table.Cy(ii:ii+length(unique(Table.BETA))-1) = Table.Cy(ii:ii+length(unique(Table.BETA))-1) - (Table.Cy(i));
    end
    
    if Table.BETA(i) == 0.00 && Table.Cmx(i) ~= 0.00
        ii = i;
        Table.Cmx(ii:ii+length(unique(Table.BETA))-1) = Table.Cmx(ii:ii+length(unique(Table.BETA))-1) - (Table.Cmx(i));
    end
    
    if Table.BETA(i) == 0.00 && Table.Cmz(i) ~= 0.00
        ii = i;
        Table.Cmz(ii:ii+length(unique(Table.BETA))-1) = Table.Cmz(ii:ii+length(unique(Table.BETA))-1) - (Table.Cmz(i));
    end
end

Table = table2array(sortrows(Table,2));




% % Write to the excel sheet

nAOA = length(unique(Aero.Fuselage.AOA));
nBETA = length(unique(Aero.Fuselage.BETA));

Sheet = 2:1:7;
Range = ["B2";"C2";"D2";"E2";"F2";"G2";"H2"];


for j = 1:1:6
    init = 1;
    for i = 1:1:nBETA
        A = Table(init:i*nAOA,j+2);
        xlswrite(file,A,Sheet(j),Range(i));
        init = nAOA + init;
    end
end






















