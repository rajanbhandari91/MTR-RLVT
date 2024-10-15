function [QMIL,QPROP] = QMIL_PROP(FltConDef,PropDef,RunQMIL,RunQPROP,QPROPAnalysis,casename)

% check license
[~] = CheckClearToRun();

QMIL = [];
QPROP = [];

%% writing atmospheric conditions to qcon.def


[T, a, P, rho] = atmosisa(FltConDef.Alt/3.28);
if FltConDef.Sig~=0
    rho = FltConDef.Sig*1.225;
end
%Mu = 0.14568*10^-5 * sqrt(T)/[1+110/T];


Mu = 17.5e-6;

AtmFileName = ['qcon.def'];

fID = fopen(AtmFileName,'w');
fprintf(fID,'\n%f    ! rho  kg/m^3', rho);
fprintf(fID,'\n%f  ! mu   kg/m-s',Mu);
fprintf(fID,'\n%f    ! a    m/s',a);

fclose(fID);

%%  Writing an Input file for Qmil.exe
%




if RunQMIL == 1
    filename = ['PropDesign\Input\','IP_mil.in'];                    %Input the Name of Input File (.in format) eg. 'Input.in'
    
    filename;
    fid = fopen(filename,'w');
    PropDef.blurb = ['Test-',PropDef.Airfoil];                     % Input the Propeller name
    PropDef.Ldes = 0;                                   % Value is usually '0' for usual prop designs, others ('1' and '2') are specified for windmill design
    PropDef.KQdes = 0;                                  % Controls the optimality condition for 'Ldes = 2'-- case to get more practical windmill Geom
    PropDef.Nout = 40;                                  % Number of Output Stations
    
    fprintf('\n Creating QMIL input file...')
    fprintf(fid,'%s\n\n',PropDef.blurb);
    fprintf(fid,'%.0f       ! Nblades\n\n',PropDef.Nblades);
    fprintf(fid,'%.4f  %.4f    ! CL0    CL_a\n',PropDef.CL0,PropDef.CL_a);
    fprintf(fid,'%.4f  %.4f    ! CLmin  CLmax\n\n',PropDef.CL_min,PropDef.CL_max);
    fprintf(fid,'%.4f  %.4f  %.4f  %.4f  ! CD0    CD2u   CD2l  CLCD0\n',PropDef.CD0,PropDef.CD2u,PropDef.CD2l,PropDef.CLCD0);
    fprintf(fid,'%.1f  %.4f               ! REref  REexp\n\n',PropDef.REref,PropDef.REexp);
    fprintf(fid,'%.2f ',PropDef.XIdes);fprintf(fid,'      ! XIdes (r/R locations where design cl is specified)\n');
    fprintf(fid,'%.2f ',PropDef.CLdes);fprintf(fid,'      ! CLdes   (specified cl)\n\n');
    fprintf(fid,'%.2f    !  hub radius(m)\n',PropDef.r_h);
    fprintf(fid,'%.2f    !  tip radius(m)\n',PropDef.r_t);
    fprintf(fid,'%.2f    !  speed(m/s)\n',FltConDef.Vel);
    fprintf(fid,'%.2f   !  rpm \n\n',FltConDef.rpm);
    fprintf(fid,'%.1f      !  Thrust(N)   ( 0 if power  specified )\n',FltConDef.T);
    fprintf(fid,'%.1f      !  Power(W)    ( 0 if thrust specified )\n\n',FltConDef.P);
    fprintf(fid,'%.1f   %.1f   ! Ldes    KQdes\n\n',PropDef.Ldes,PropDef.KQdes);
    fprintf(fid,'%.0f       ! Nout    number of output stations (optional)',PropDef.Nout);
    
    fclose(fid);
    
    
    %% Running the Qmil.exe and Parsing the Output File
    %
    
%     fprintf('\n Running QMIL...')  
    [ ~, cmdout ] = system('PropDesign\Software\qmil.exe PropDesign\Input\IP_mil.in PropDesign\Output\OP_mil.out') ;          % Running the Qmil.exe
    
    copyfilename = ['PropDesign\Output\','OP_mil_',casename,'.out'];
    copyfile('PropDesign\Output\OP_mil.out',copyfilename);
    %copyfile('Output\OP_mil.out',['Output\','OP_mil_',foil,'.out']);         % The output file is stored as a copy with the used Airfoil ID
    %copyfile('Output\OP_mil.out',['PropPlotting\','OP_mil_',foil,'.out']);
    
    
    %     QMIL = Parse_QMIL(['Output\','OP_mil_',foil,'.out']);
    QMIL = Parse_QMIL('PropDesign\Output\OP_mil.out');
    
    QMIL.r_R = QMIL.r/QMIL.r(end);
    QMIL.c_R = QMIL.c/QMIL.r(end);
    QMIL.RE = rho .* (QMIL.r.*FltConDef.rpm*2*pi/60) .* QMIL.c./Mu;


    % find twist at 0.75R
    beta75R = interp1(QMIL.r_R,QMIL.beta,0.75,'linear');

    % subtract this value from pitches of all stations
    QMIL.relbeta = QMIL.beta - beta75R;

    % 0.75R stations remains "flat". Adjust chords of all other stations
    QMIL.c_R_proj = QMIL.c_R.*cosd(QMIL.relbeta);

end



if RunQPROP == 1
    % Feeding QMIL output as INPUT to QPROP for Propeller Analysis
    % % % qprop propfile motorfile 4.0 0 0 0.0 0 0.03
    % %                          ( Vel Rpm Volt FltConDef.dBeta Thrust Torque Amps Pele )
    % % specifies
    % % Vel = 4.0 m/s
    % % Rpm = unspecified
    % % Volt = unspecified
    % % PropDef.dBeta = 0.0 deg
    % % Thrust = unspecified
    % % Torque = 0.03 N-m
    % % Amps = unspecified
    % % Pele = unspecified
    
    % Multi-Point Runs
    % % Vel replaced by Vel1,Vel2,dVel
    % % Rpm replaced by Rpm1,Rpm2,dRpm
    % % Volt replaced by Volt1,Volt2,dVolt
    % % PropDef.dBeta replaced by PropDef.dBeta1,PropDef.dBeta2,dPropDef.dBeta
    
    if strcmp(QPROPAnalysis,'RPM-Vel')
        SweepParam = sprintf('\t%d,%d,%d\t%d,%d,%d\t0\t%d\t%d',FltConDef.Vel1,FltConDef.Vel2,FltConDef.VelDel,FltConDef.rpm1,FltConDef.rpm2,FltConDef.rpmDel,FltConDef.dBeta,FltConDef.T);
    end
    
    if strcmp(QPROPAnalysis,'RPM-Vel-Beta')
        SweepParam = sprintf('\t%d,%d,%d\t%d,%d,%d\t0\t%d,%d,%d\t%d',FltConDef.Vel1,FltConDef.Vel2,FltConDef.VelDel,FltConDef.rpm1,FltConDef.rpm2,FltConDef.rpmDel,FltConDef.dBeta1,FltConDef.dBeta2,FltConDef.dBetaDel,FltConDef.T);
    end
    
    if strcmp(QPROPAnalysis,'PointPerf')  % order of arguments corrected 05/14/2020
        % note: The parameters after and including "Volt" are optional
        Volt = 0.0;
        FltConDef.T = 0.1;
        SweepParam = sprintf('\t%d\t%d\t%d\t%d\t%d',FltConDef.Vel,FltConDef.rpm,Volt,FltConDef.dBeta,FltConDef.T);
    end
    
    if strcmp(QPROPAnalysis,'RPM')
        SweepParam = sprintf('\t%d\t%d,%d,%d\t0\t%d\t%d',FltConDef.Vel,FltConDef.rpm1,FltConDef.rpm2,FltConDef.rpmDel,FltConDef.dBeta,FltConDef.T);
    end
    
    if strcmp(QPROPAnalysis,'Vel')
        SweepParam = sprintf('\t%d,%d,%d\t%d\t0\t%d\t%d',FltConDef.Vel1,FltConDef.Vel2,FltConDef.VelDel,FltConDef.rpm,FltConDef.dBeta,FltConDef.T);
    end
    
    if strcmp(QPROPAnalysis,'Beta')
        Volt = 0.0;
        SweepParam = sprintf('\t%d\t%d\t%d\t%d,%d,%d\t%d',FltConDef.Vel,FltConDef.rpm,Volt,FltConDef.dBeta1,FltConDef.dBeta2,FltConDef.dBetaDel,FltConDef.T);
    end
    
    
    InputProp = strcat('PropDesign\Software\qprop.exe PropDesign\Output\OP_mil.out MotorFile ',SweepParam,'> PropDesign\Output\OP_prop.out');
%     fprintf('\n Running QPROP...casename %s...',casename)
    [ ~, cmdout ] = system(InputProp) ;
    copyfilename = {sprintf('PropDesign\\Output\\OP_prop_%s.out',casename)};
    copyfile('PropDesign\Output\OP_prop.out',copyfilename{1});                           % The output file is stored as a copy with the used Airfoil ID
    
    
%     QPROP = Parse_QPROP(['PropDesign\Output\OP_prop.out'],QPROPAnalysis);
    
    try
        % always parse output file generated by QPROP - updated 05/14/2020
        QPROP = Parse_QPROP(['PropDesign\Output\OP_prop.out'],QPROPAnalysis);
        
        QPROP.Filename = repmat(copyfilename,[height(QPROP),1]);
        
    catch ERRMSG
        ERRMSG;
        sprintf('parsing failed')
    end
    
end


end
























