clc
clear


RangeGrid_km = [1000,1500,2000,2600];
EWFGrid = [0.4,0.5,0.6,0.7];

[Range_km, EWF] = ndgrid(RangeGrid_km,EWFGrid);

[imax, jmax] = size(Range_km);


for i = 1:1:imax
    
    
    for j = 1:1:jmax
        
        SaveName = sprintf('Vehicle_%s_%s',num2str(i),num2str(j));
        FailName = sprintf('Fail_%s_%s',num2str(i),num2str(j));
        
        clc
        
        
        ctr = 1;
        
        % Points 1,2
        Segments(ctr).Name = 'Climb_1';
        Segments(ctr).Type = 'climb';
        Segments(ctr).StartAlt = 0;
        Segments(ctr).StartVel = 00;
        Segments(ctr).StartVelType = 'EAS';
        Segments(ctr).EndAlt = 6000/3.28;
        Segments(ctr).TAS = [];
        Segments(ctr).EAS = 42*0.5144;
        Segments(ctr).Mach = [];
        Segments(ctr).Points = 10;
        ctr = ctr+1;
        % Points 2,3
        Segments(ctr).Name = 'MainCruise';
        Segments(ctr).Type = 'cruise';
        Segments(ctr).StartAlt = 8000/3.28;
        Segments(ctr).EndAlt = 10000/3.28;
        Segments(ctr).TAS = 60*0.5144;
        ctr = ctr+1;
        % Points 3,4
        Segments(ctr).Name = 'Descent_1';
        Segments(ctr).Type = 'descent';
        Segments(ctr).StartAlt = [];
        Segments(ctr).EndAlt = 500;
        Segments(ctr).EAS = 55*0.5144;
        Segments(ctr).ROD = 500/(3.28*60);
        ctr = ctr+1;
        % Points 4,5
        Segments(ctr).Name = 'Loiter';
        Segments(ctr).Type = 'loiter';
        Segments(ctr).StartAlt = 500;
        Segments(ctr).EndAlt = 500;
        Segments(ctr).EAS = 45*0.5144;
        ctr = ctr+1;
        % Points 5,6
        Segments(ctr).Name = 'Climb_2';
        Segments(ctr).Type = 'climb';
        Segments(ctr).StartAlt = [];
        Segments(ctr).EndAlt = 2000;
        Segments(ctr).EAS = 42*0.5144;
        ctr = ctr+1;
        % Points 6,7
        Segments(ctr).Name = 'ReserveCruise';
        Segments(ctr).Type = 'cruise';
        Segments(ctr).StartAlt = [];
        Segments(ctr).EndAlt = 3000;
        Segments(ctr).TAS = 55*0.5144;
        ctr = ctr+1;
        % Points 7,8
        Segments(ctr).Name = 'Descent_2';
        Segments(ctr).Type = 'descent';
        Segments(ctr).StartAlt = 3000;
        Segments(ctr).EndAlt = 000;
        Segments(ctr).EAS = 55*0.5144;
        Segments(ctr).ROD = 500/(3.28*60);
        ctr = ctr+1;
        
        
        Mission.Segments = Segments;
        
        % specify distance requirements
        Dists = zeros(8,8);
        Dists(1,4) = Range_km(i,j)*1000;
        Dists(5,8) = 100e3;
        Mission.Dists = Dists;
        
        % specify duration requirements
        Durations(4,5) = 30;
        Mission.Durations = Durations;
        
        Mission.PropTypes = 1;
        Mission.dISA = 0;
        
        Mission.Payload_kg = (47000/2.2);
        
        Mission.ResEnergyPerc = 0.05;
        
        Mission = Initializer(Mission);
        
        Vehicle.MassProp.OEWWtFrac = EWF(i,j);
        
        
        try
            [Vehicle] = MissionAnalyzer(Mission,Vehicle);
            save(SaveName,'Vehicle')
            
            Summary.ShipVolume(i,j) = Vehicle.ShipVolume;
            Summary.TOGW(i,j) = Vehicle.MassProp.TOGM_kg;
            Summary.OEW(i,j) = Vehicle.MassProp.OEW_kg;
            Summary.Fuel(i,j) = Vehicle.Mission.History.EMass(1);
            Summary.Power(i,j) = Vehicle.Propulsion.RatedPower_kW;
            
            clear Vehicle
            clear Mission
            
        catch MSG
            Vehicle.MSG = MSG;
            save(FailName,'Vehicle');
            clear Vehicle
            clear Mission
            
        end
    end
    
    
end


% Mission.History

