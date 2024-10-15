function AF_out = PerformLEandTERefinement(US0,LS0)


nc = 40;     % number of chordwise points along along upper and lower surfaces (each)

%%%% COMPUTE GROWTH FACTOR
growth_rate = 1.1;
ipts = nc;

ipts3 = FLOOR(0.5*(ipts-1));
ipts4 = ipts-1-ipts3;

    
growth_factor = 0;

for i = 1:ipts3
        growth_factor = growth_factor + growth_rate^(i-1);
end

for i = 1:ipts4
        growth_factor = growth_factor + growth_rate^(i-1);
end


Parametric_Spacing = 0.0;

sizer = 0;
for i = 1:ipts
        
        if(i == 1)
            uc(i) = 0;
        end
        
        if(i >= 2)
            uc(i) = sizer + 1*(growth_rate^(i-2))/growth_factor;
        end
       
        if(i == ipts)
            uc(i) = 1;
        end
        
%         ulocal(i) = uc;
                    
        if(i >= 2)
            sizer = sizer + 1*(growth_rate^(i-2))/growth_factor;
        end
end
cquery = uc';

US = [cquery,interp1(US0(:,1),US0(:,2),cquery,'linear')];
LS = [cquery,interp1(LS0(:,1),LS0(:,2),cquery,'linear')];

AF_out = [flipud(US);LS(2:end,:)];

AF_out(:,3) = 0;