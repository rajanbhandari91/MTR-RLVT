function ParamOut = getParamfromVehicle(Vehicle)

m_ft = 3.28;
sqm_sqft = 10.7639;
%% Start with BODY component
BodyField = fieldnames(Vehicle.OpenVSP.Fus);

for i = 1: length(BodyField)
    try
        ParamIn = Vehicle.OpenVSP.Fus.(['Fus_',num2str(i)]);
        if isempty(Vehicle.Geom.(ParamIn.Name).Stn)
            ParamIn = [];
        end
        Comp = Vehicle.Geom.(ParamIn.Name);
        ParamOut.Fus.(['Fus_',num2str(i)]).Name = Comp.Name;
        ParamOut.Fus.(['Fus_',num2str(i)]).Length = Comp.Length * m_ft ;
        FS = ParamIn.FS;
        ParamOut.Fus.(['Fus_',num2str(i)]).Height = zeros(1,length(FS)-1);
        ParamOut.Fus.(['Fus_',num2str(i)]).Width = zeros(1,length(FS)-1);
        if startsWith(Comp.Name,'Fuselage')
            ParamOut.Fus.RefLocation = [Comp.Stn.Xc(1) Comp.Stn.Yc(1) Comp.Stn.Zc(1)];
            ParamOut.Fus.VSPLocation = ParamIn.Location .* [-1 1 -1];
        end
        for j = 1: length(FS)-1
            ParamOut.Fus.(['Fus_',num2str(i)]).Height(j) = interp1(Comp.Stn.FS,Comp.Stn.Height,FS(j),'linear') * m_ft;
            ParamOut.Fus.(['Fus_',num2str(i)]).Width(j) = interp1(Comp.Stn.FS,Comp.Stn.Width,FS(j),'linear') * m_ft;
        end
        if ~strcmp(Comp.Name,'Fuselage')
            FDLocation = Comp.RefPtLocation;
            FDLocation(1) = FDLocation(1) + Comp.RefPt_FS * Comp.Length;
            Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
            ParamOut.Fus.(['Fus_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;
        end

        RBC = Comp.R_BC;
        theta = asind(-RBC(3));
        psi = asind(RBC(2) / cosd(theta));
        phi = asind(RBC(6) / cosd(theta));
        ParamOut.Fus.(['Fus_',num2str(i)]).Transform.Rotation = [phi theta psi] .* [-1 1 1];
    catch
    end
end


%% Lifting Surface Components
LSField = fieldnames(Vehicle.OpenVSP.LS);
i = 1 ;
count = length(LSField);
while count ~= 0
    try
        ParamIn = Vehicle.OpenVSP.LS.(['LS_',num2str(i)]);
        if startsWith(ParamIn.Type,'Wing')
            if startsWith(ParamIn.Name,'Wing')
                Comp = Vehicle.Geom.RWing;
                if ParamIn.Transform.Symm == 2
                    ParamOut.LS.(['LS_',num2str(i)]).Name = ParamIn.Name;
                    ParamOut.LS.(['LS_',num2str(i)]).Span = Comp.Span * m_ft;
                    ParamOut.LS.(['LS_',num2str(i)]).PlanformArea =  Comp.PlanformArea * sqm_sqft;
                    ParamOut.LS.(['LS_',num2str(i)]).AR = Comp.AspectRatio ;
                    ParamOut.LS.(['LS_',num2str(i)]).eta = Comp.TaperDefn(1,:);
                    Chord = interp1(Comp.Stn.eta,Comp.Stn.c,ParamOut.LS.(['LS_',num2str(i)]).eta)  * m_ft;
                    ParamOut.LS.(['LS_',num2str(i)]).TR = Chord(2:end) ./ Chord(1:end-1);
                    ParamOut.LS.(['LS_',num2str(i)]).Chord = Chord;
                    SectSpan = ParamIn.eta(2:end) * Comp.Span ;
                    SectSpan(2:end) = SectSpan(2:end) - SectSpan(1:end-1);
                    ParamOut.LS.(['LS_',num2str(i)]).SectSpan = SectSpan .* m_ft ;
                    SectArea = interp1(Comp.Stn.eta,Comp.Stn.sdA,ParamIn.eta(2:end),'linear');
                    SectArea(2:end) = SectArea(2:end) - SectArea(1:end-1);
                    ParamOut.LS.(['LS_',num2str(i)]).SectArea = SectArea .* sqm_sqft;
                    SectAR = (SectSpan .^ 2) ./ SectArea;
                    ParamOut.LS.(['LS_',num2str(i)]).SectAR = SectAR ;
                    ParamOut.LS.(['LS_',num2str(i)]).Sweep = Comp.SweepDefn(3,:);
                    ParamOut.LS.(['LS_',num2str(i)]).SweepChordLoc = Comp.SweepDefn(2,:);
                    ParamOut.LS.(['LS_',num2str(i)]).Twist = Comp.Twist(2,:);

                    % ParamOut.LS.(['LS_',num2str(i)]).RootIncidence = Comp.RootIncidence;
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Symm = ParamIn.Transform.Symm;
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [Comp.RootIncidence 0 0];
                    FDLocation = Comp.RefPtLocation;
                    FDLocation(1) = FDLocation(1) + Comp.RefPtChordFrac * Comp.Stn.c(1) ;
                    Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;
                    if ParamIn.Transform.Rotation(1) ~= 0
                        ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = ParamIn.Transform.Rotation;
                        ParamOut.LS.(['LS_',num2str(i)]).Dihedral = zeros(1,length(ParamOut.LS.(['LS_',num2str(i)]).eta));
                    else
                        ParamOut.LS.(['LS_',num2str(i)]).Dihedral = Comp.Dihedral(2,:);
                        ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [0 Comp.RootIncidence 0];

                    end
                end
            end

            if startsWith(ParamIn.Name,'Tail')
                if ParamIn.Transform.Symm == 2
                    Comp = Vehicle.Geom.RHTail;
                else
                    Comp = Vehicle.Geom.VTail;
                end
                ParamOut.LS.(['LS_',num2str(i)]).Name = ParamIn.Name;
                ParamOut.LS.(['LS_',num2str(i)]).Span = Comp.Span * m_ft;
                ParamOut.LS.(['LS_',num2str(i)]).PlanformArea =  Comp.PlanformArea * sqm_sqft;
                ParamOut.LS.(['LS_',num2str(i)]).AR = Comp.AspectRatio ;
                ParamOut.LS.(['LS_',num2str(i)]).eta = Comp.TaperDefn(1,:);
                Chord = interp1(Comp.Stn.eta,Comp.Stn.c,ParamOut.LS.(['LS_',num2str(i)]).eta)  * m_ft;
                ParamOut.LS.(['LS_',num2str(i)]).TR = Chord(2:end) ./ Chord(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).Chord = Chord;
                SectSpan = ParamIn.eta(2:end) * Comp.Span ;
                SectSpan(2:end) = SectSpan(2:end) - SectSpan(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).SectSpan = SectSpan .* m_ft ;
                SectArea = interp1(Comp.Stn.eta,Comp.Stn.sdA,ParamIn.eta(2:end),'linear');
                SectArea(2:end) = SectArea(2:end) - SectArea(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).SectArea = SectArea .* sqm_sqft;
                SectAR = (SectSpan .^ 2) ./ SectArea;
                ParamOut.LS.(['LS_',num2str(i)]).SectAR = SectAR ;
                ParamOut.LS.(['LS_',num2str(i)]).Sweep = Comp.SweepDefn(3,:);
                ParamOut.LS.(['LS_',num2str(i)]).SweepChordLoc = Comp.SweepDefn(2,:);
                ParamOut.LS.(['LS_',num2str(i)]).Twist = Comp.Twist(2,:);

                ParamOut.LS.(['LS_',num2str(i)]).Transform.Symm = ParamIn.Transform.Symm;
                ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [Comp.RootIncidence 0 0];
                FDLocation = Comp.RefPtLocation;
                FDLocation(1) = FDLocation(1) + Comp.RefPtChordFrac * Comp.Stn.c(1) ;
                Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
                ParamOut.LS.(['LS_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;
                if ParamIn.Transform.Rotation(1) ~= 0
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = ParamIn.Transform.Rotation;
                    ParamOut.LS.(['LS_',num2str(i)]).Dihedral = zeros(1,length(ParamOut.LS.(['LS_',num2str(i)]).eta));
                else
                    ParamOut.LS.(['LS_',num2str(i)]).Dihedral = Comp.Dihedral(2,:);
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [0 Comp.RootIncidence 0];
                end
                ParamOut.LS.(['LS_',num2str(i)]).Type = ParamIn.Type;
            end

            if startsWith(ParamIn.Name,'Landing')
                Comp = Vehicle.Geom.(ParamIn.Name);
                ParamOut.LS.(['LS_',num2str(i)]).Name = ParamIn.Name;
                ParamOut.LS.(['LS_',num2str(i)]).Span = Comp.Span * m_ft;
                ParamOut.LS.(['LS_',num2str(i)]).PlanformArea =  Comp.PlanformArea * sqm_sqft;
                ParamOut.LS.(['LS_',num2str(i)]).AR = Comp.AspectRatio ;
                ParamOut.LS.(['LS_',num2str(i)]).eta = Comp.TaperDefn(1,:);
                Chord = interp1(Comp.Stn.eta,Comp.Stn.c,ParamOut.LS.(['LS_',num2str(i)]).eta)  * m_ft;
                ParamOut.LS.(['LS_',num2str(i)]).TR = Chord(2:end) ./ Chord(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).Chord = Chord;
                SectSpan = ParamIn.eta(2:end) * Comp.Span ;
                SectSpan(2:end) = SectSpan(2:end) - SectSpan(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).SectSpan = SectSpan .* m_ft ;
                SectArea = interp1(Comp.Stn.eta,Comp.Stn.sdA,ParamIn.eta(2:end),'linear');
                SectArea(2:end) = SectArea(2:end) - SectArea(1:end-1);
                ParamOut.LS.(['LS_',num2str(i)]).SectArea = SectArea .* sqm_sqft;
                SectAR = (SectSpan .^ 2) ./ SectArea;
                ParamOut.LS.(['LS_',num2str(i)]).SectAR = SectAR ;
                ParamOut.LS.(['LS_',num2str(i)]).Sweep = Comp.SweepDefn(3,:);
                ParamOut.LS.(['LS_',num2str(i)]).SweepChordLoc = Comp.SweepDefn(2,:);
                ParamOut.LS.(['LS_',num2str(i)]).Twist = Comp.Twist(2,:);

                ParamOut.LS.(['LS_',num2str(i)]).Transform.Symm = ParamIn.Transform.Symm;
                ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [Comp.RootIncidence 0 0];
                FDLocation = Comp.RefPtLocation;
                FDLocation(1) = FDLocation(1) + Comp.RefPtChordFrac * Comp.Stn.c(1) ;
                if abs(ParamIn.Transform.Rotation(3)) > 90
                    FDLocation(1) = FDLocation(1) -  Comp.Stn.c(1) ;
                end
                Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
                ParamOut.LS.(['LS_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;
                if ParamIn.Transform.Rotation(1) ~= 0
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = ParamIn.Transform.Rotation;
                    ParamOut.LS.(['LS_',num2str(i)]).Dihedral = zeros(1,length(ParamOut.LS.(['LS_',num2str(i)]).eta));
                else
                    ParamOut.LS.(['LS_',num2str(i)]).Dihedral = Comp.Dihedral(2,:);
                    ParamOut.LS.(['LS_',num2str(i)]).Transform.Rotation = [0 Comp.RootIncidence 0];
                end
                ParamOut.LS.(['LS_',num2str(i)]).Type = ParamIn.Type;
            end
            count = count - 1;
        end
        if startsWith(Vehicle.OpenVSP.LS.(['LS_',num2str(i)]).Type,'Wheel')
            Comp = Vehicle.Geom.(ParamIn.Wheel.Name);
            ParamOut.LS.(['LS_',num2str(i)]).Diameter = Comp.Stn.c(1) * m_ft;
            ParamOut.LS.(['LS_',num2str(i)]).Span = Comp.Span * m_ft;
            FDLocation = Comp.RefPtLocation;
            FDLocation(2) = FDLocation(2);
            Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
            ParamOut.LS.(['LS_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;
            ParamOut.LS.(['LS_',num2str(i)]).Type = ParamIn.Type;
            ParamOut.LS.(['LS_',num2str(i)]).Name = Comp.Name;
            count = count - 1;
        end

    catch
    end
    i = i + 1;
end

%% Rotor Components
RotorFields = fieldnames(Vehicle.OpenVSP.Disk);

for i = 1:length(RotorFields)
    try
        ParamIn = Vehicle.OpenVSP.Disk.(['Rotor_',num2str(i)]);
        Comp = Vehicle.Geom.(['Rotor_',num2str(i)]);
        ParamOut.Disk.(['Rotor_',num2str(i)]).Name = ParamIn.Name;
        ParamOut.Disk.(['Rotor_',num2str(i)]).Diameter = Comp.Diam * m_ft;
        ParamOut.Disk.(['Rotor_',num2str(i)]).theta = 90 - Comp.Theta;
        ParamOut.Disk.(['Rotor_',num2str(i)]).phi = Comp.Phi;
        ParamOut.Disk.(['Rotor_',num2str(i)]).NBlades = ParamIn.NBlades;
        FDLocation = Comp.RefPtLocation;
        Offset = [ParamOut.Fus.RefLocation - FDLocation]  .* [-1 1 -1];
        ParamOut.Disk.(['Rotor_',num2str(i)]).Transform.Location = [ParamOut.Fus.VSPLocation - Offset] * m_ft ;

    catch
    end

end


