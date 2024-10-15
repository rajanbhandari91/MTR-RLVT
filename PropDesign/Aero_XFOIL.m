
function PropDef = Aero_XFOIL(foil,M,Re)
%% XFOIL...
% This script generates and executes a run of XFOIL
% TODO:
% - segment into sub-functions as needed
% - parse output (Done.)
% - extract aero parameters for InputDef.m (Done.)

%% Run conditions

a_min = -5;
a_max = 15;

negLoop = 0:-2:a_min;

a_space = 0.1;

%% Generate .bat file
here = pwd;

foil_loc = ['Airfoil\',foil,'.dat'];

% foil = upper(foil(9:end-4));
% foil = sprintf('MH117_%.2f',Re*10^-5);
% foil = sprintf('MH117');
filename = [foil,'.txt'];
xfoil_loc = 'Software\xfoil';
fnm_polar = ['Airfoil\',foil,'.polar'];
fnm_dump = ['Airfoil\',foil,'.dump'];

% Delete polar and dump if it already exists
if (exist(fnm_polar,'file')==2)
    delete(fnm_polar);
end
if exist(fnm_dump,'file')==2
    delete(fnm_dump);
end

    fid = fopen(filename,'w');

%     fprintf(fid,'cd %s\n',here);
%     fprintf(fid,'%s\n',xfoil_loc);
    fprintf(fid,'PLOP\n');
    fprintf(fid,'G\n');
    fprintf(fid,'\n');
    fprintf(fid,'load %s\n',foil_loc);
    fprintf(fid,'panel\n');
    fprintf(fid,'oper\n');
    fprintf(fid,'visc\n');
    fprintf(fid,'%.1e\n',Re);
    fprintf(fid,'M %.1f\n',M);
    fprintf(fid,'iter 100\n');
    fprintf(fid,'pacc\n');
    fprintf(fid,'%s\n',fnm_polar);
    fprintf(fid,'%s\n',fnm_dump);
    fprintf(fid,'aseq 0 %.0f %.2f\n',a_max,a_space);
    for i = 1:numel(negLoop)-1
        fprintf(fid,'init\n');
        fprintf(fid,'aseq %.1f %.1f %.1f\n',negLoop(i)-a_space,negLoop(i+1),a_space);
    end
    fprintf(fid,'pacc\n\n');
    fprintf(fid,'quit\n');

    fclose(fid);
    %% Execute .bat file
    
    [stat_xfoil, res_xfoil] = dos(['Software\xfoil.exe < ', filename]);
 
%% Parse Output File

 [Labels Foil_Data] = Parse_XFOIL(fnm_polar)
 Foil_Data = sortrows(Foil_Data,1)
 
 

%% Aerodynamic Parameters
PropDef = XFOIL2PolarParam(Foil_Data);
PropDef.REref = Re;

% append parsed XFOIL data to struct
XFoilData.alpha = Foil_Data(:,1);
XFoilData.CL = Foil_Data(:,2);
XFoilData.CD = Foil_Data(:,3);
XFoilData.CDp = Foil_Data(:,4);
XFoilData.CM = Foil_Data(:,5);
XFoilData.Top_Xtr = Foil_Data(:,6);
XFoilData.Bot_Xtr = Foil_Data(:,7);

XFoilData.CLCD = XFoilData.CL./XFoilData.CD;
XFoilData.CL32CD = (abs(XFoilData.CL).^(3/2))./XFoilData.CD;

XFoilData = struct2table(XFoilData);

PropDef.XFoilData = XFoilData;
PropDef.XFoilLabels = Labels;

save(['Airfoil\',foil,'.mat'])


end