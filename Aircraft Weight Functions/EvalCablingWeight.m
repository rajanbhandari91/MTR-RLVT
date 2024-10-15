function [Vehicle] = EvalCablingWeight(Vehicle)

global Settings





WingQueryEtas = [...
    0;
    Vehicle.Geom.Pylons_2.RefPtLocation(2); Vehicle.Geom.Nacelle_4.RefPtLocation(2)
    ]/abs(Vehicle.Geom.Nacelle_4.RefPtLocation(2));

RWStn = Vehicle.Geom.RWing.Stn;
Pts = interp1(RWStn.eta,[RWStn.xLE, RWStn.yLE, RWStn.zLE, RWStn.xTE, RWStn.yTE, RWStn.zTE],WingQueryEtas,'linear');

LE = Pts(:,1:3);
TE = Pts(:,4:6);

Rc10 = LE + (TE-LE)*0.10;
Rc25 = LE + (TE-LE)*0.25;
Rc50 = LE + (TE-LE)*0.50;
Rc70 = LE + (TE-LE)*0.70;


Lc10 = Rc10.*[1,-1,1];
Lc25 = Rc25.*[1,-1,1];
Lc50 = Rc50.*[1,-1,1];
Lc70 = Rc70.*[1,-1,1];


% BATT_L1 = [Vehicle.Geom.LCan.RefPtLocation(1),0.1,0];
% BATT_L2 = [Vehicle.Geom.LCan.RefPtLocation(1),-0.1,0];
% BATT_L3 = [0.3, 0.5, 0];
%
% BATTHUB_C1 = [0, -0.5, 0];
% BATTHUB_C2 = [0, +0.5, 0];

ctr = 1;

% B1: hub to MP-2
P = [Lc10(1:2,:); Vehicle.Geom.Motors.RefPtLocation(2,:)];
Pwr = Vehicle.Propulsion.CruiseMotorPower_Each_kW;
ConnName(ctr,1) = {'B2 hub to LP2'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;

% B1: hub to LP-6
P = [Rc10(1:2,:); Vehicle.Geom.Motors.RefPtLocation(6,:)];
Pwr = Vehicle.Propulsion.LiftMotorPower_Each_kW;
ConnName(ctr,1) = {'B2 hub to LP7'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;

% B2: hub to MP-1
P = [Lc25; Vehicle.Geom.Motors.RefPtLocation(1,:)];
Pwr = Vehicle.Propulsion.CruiseMotorPower_Each_kW;
ConnName(ctr,1) = {'B1 hub to MP1'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;

% B3: hub to MP-4
P = [Rc25; Vehicle.Geom.Motors.RefPtLocation(4,:)];
Pwr = Vehicle.Propulsion.CruiseMotorPower_Each_kW;
ConnName(ctr,1) = {'B1 hub to MP4'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;

% B3: hub to MP-3
P = [Rc50(1:2,:); Vehicle.Geom.Motors.RefPtLocation(3,:)];
Pwr = Vehicle.Propulsion.CruiseMotorPower_Each_kW;
ConnName(ctr,1) = {'B3 hub to MP3'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;

% B3: hub to LP-5
P = [Lc50(1:2,:); Vehicle.Geom.Motors.RefPtLocation(5,:)];
Pwr = Vehicle.Propulsion.LiftMotorPower_Each_kW;
ConnName(ctr,1) = {'B3 hub to LP6'};
[ConnRow(ctr,:)] = EvalConnection(P, Pwr);
ctr = ctr + 1;


% Architecture 1: All-ELectric (AE)
if Vehicle.Architecture == 1

    % Find the Path on Fuselage that will lead to Lc50
    index1 = ceil(interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.i,Vehicle.Geom.Batteries.RefPtLocation(1,1),'linear'));
    % index2 = ceil(interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.i,Lc50(1,1),'linear'));
    index2 = ceil(interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.i,Vehicle.Geom.Batteries.RefPtLocation(2,1),'linear'));
    Path = Vehicle.Geom.Fuselage.Stn(index1:index2,:);

    Route = [...
        Vehicle.Geom.Batteries.RefPtLocation(1,:);
        Path.Xc, Path.Yc, Path.Zc - 0.47*Path.Height;
        Vehicle.Geom.Batteries.RefPtLocation(2,:)];


    % BATT ZONE 1
    P = Route;
    Pwr = Vehicle.Propulsion.BatteryPackPeakPower_kW;
    ConnName(ctr,1) = {'BATT Zone'};
    [ConnRow(ctr,:)] = EvalConnection(P, Pwr);
   

end


% Architecture 2: Hybrid-ELectric (HE) or Architecture 3: Turbo-Electric (TE)
if(Vehicle.Architecture == 2 || Vehicle.Architecture == 3)
    % hub to GEN 3
    if Vehicle.Architecture == 3
        Index1 = ceil(interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.i,Rc70(1,1),'linear'));
        Index2 = ceil(interp1(Vehicle.Geom.Fuselage.Stn.Xc,Vehicle.Geom.Fuselage.Stn.i,Vehicle.Geom.Generator.RefPtLocation(1),'linear'));
        Path = Vehicle.Geom.Fuselage.Stn(Index1:Index2,:);

        RouteEng = [Path.Xc , Path.Yc, Path.Zc - 0.46*Path.Height; Vehicle.Geom.Generator.RefPtLocation; Vehicle.Geom.Batteries.RefPtLocation];
        P = RouteEng;
        Pwr = Vehicle.Propulsion.GeneratorPower_Each_kW;
        ConnName(ctr,1) = {'hub to GEN3'};
        [ConnRow(ctr,:)] = EvalConnection(P, Pwr);
        % ctr = ctr + 1;
    end


end



% %% Drawing Lines and Evaluating Each Segment
%
ConnectionSummary.Name = ConnName;
ConnectionSummary.Length_m = ConnRow(:,1);
ConnectionSummary.xcg = ConnRow(:,2);
ConnectionSummary.ycg = ConnRow(:,3);
ConnectionSummary.zcg = ConnRow(:,4);
ConnectionSummary.kVA = ConnRow(:,5);
ConnectionSummary.kVA_m = ConnRow(:,6);
ConnectionSummary.Mass_kg = ConnectionSummary.kVA_m /Vehicle.SOTA.Cabling_kVAmkg;

Vehicle.Geom.Power_Cabling.ConnectionSummary = struct2table(ConnectionSummary);
Vehicle.Geom.Power_Cabling.Mass = sum(ConnectionSummary.Mass_kg);
Vehicle.Geom.Power_Cabling.CG = [sum(ConnectionSummary.Mass_kg.*ConnectionSummary.xcg), sum(ConnectionSummary.Mass_kg.*ConnectionSummary.ycg), sum(ConnectionSummary.Mass_kg.*ConnectionSummary.zcg)]/sum(ConnectionSummary.Mass_kg);




%
%
    function [ConnRow] = EvalConnection(pts,Pwr)

    x = pts(:,1);
    y = pts(:,2);
    z = pts(:,3);

    len = sum(sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2));

    cg = [mean(x), mean(y), mean(z)];

    kvam = Pwr * len;

    ConnRow = [len, cg, Pwr, kvam];

    if Settings.plotFLAG == 1
        plot3(x,y,z,'m','linewidth',3);
    end

    end



end




