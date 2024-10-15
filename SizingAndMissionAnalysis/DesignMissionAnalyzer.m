

        PropTypes = Mission.PropTypes;                                              % capture number of propulsor types

        % get energy mass on board for this iteration
        EnergyMass_kg = Vehicle.MassProp.EnergyMass_kg;
        
        [npts,~] = size(Mission.History);
        [~,nseg] = size(Mission.Segments);
        
        H = Mission.History;
        S = Mission.Segments;
        
        H.THROT = zeros(npts,PropTypes);
        H.dmdt = zeros(npts,PropTypes);
        H.dEdt = zeros(npts,PropTypes);
        
        DistTable = zeros(nseg+1,nseg+1);
        TimeTable = zeros(nseg+1,nseg+1);
        
        % Step FORWARD through the mission
        History = [];
        
        
        DummySegment.Points = 1;
        DummySegment.Name = 'blank';
        DummySegment.Type = 'blank';
        DummySegNum = 0;
        
        [History] = SegmentInitializer(DummySegment,DummySegNum,PropTypes);
        
        History.Time(1) = 0;
        History.Alt(1) = 0;
        History.TAS(1) = 0;
        History.Dist(1) = 0;
        
        % set starting mass
        H.Mass(1) = Vehicle.MassProp.MTOM_kg;
        History.Mass(1) = Vehicle.MassProp.MTOM_kg;
        Mission.Segments(1).StartMass = Vehicle.MassProp.MTOM_kg;
        
        
        % set starting energy
        History.Energy(1,:) = EnergyMass_kg.*Vehicle.SOTA.SpecificEnergy_MJkg;
        History.Emass(1,:) = EnergyMass_kg;
        
        fprintf('\n Running design mission\n')
 
        for i = 1:1:nseg
            
            
            SegInd = find(H.SegNum==i);
            
            % climb segment
            switch S(i).Type
                
                case 'climb'
                    SegmentHistory = SegEval_Climb(Mission,i,History,Vehicle);
                    
                    if i>1
                        History = [History;SegmentHistory];
                    else
                        History = SegmentHistory;
                    end
                    
                case 'cruise'
                    SegmentHistory = SegEval_Cruise(Mission,i,History,Vehicle);
                    History = [History;SegmentHistory];
                    
                case 'descent'
                    SegmentHistory = SegEval_Descent(Mission,i,History,Vehicle);
                    History = [History;SegmentHistory];
                    
                case 'loiter'
                    SegmentHistory = SegEval_Loiter(Mission,i,History,Vehicle);
                    History = [History;SegmentHistory];
                    
                case 'verticalflight'
                    SegmentHistory = SegEval_VertFlight(Mission,i,History,Vehicle);
                    
                    if i>1
                        History = [History;SegmentHistory];
                    else
                        History = SegmentHistory;
                    end
                    
                case 'transition'
                    SegmentHistory = SegEval_Transition(Mission,i,History,Vehicle);
                    History = [History;SegmentHistory];
                    
                    
            end
            
            History.dhdt = History.TAS.*sin(History.FPA);
            
            [DistTable,TimeTable] = UpdateDistTable(History,DistTable,TimeTable,i);
            
            Mission.DistTable = DistTable;
            
            
            
            
        end