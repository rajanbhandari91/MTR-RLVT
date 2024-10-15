function Cross = getCrossSection(x,y,z,Rotation,FS)

phi = -Rotation(1);
theta = Rotation(2);
psi = -Rotation(3);
coords = [x y z];

% Evaluate the rotation tensor
R1 = [1 0 0;...
    0 cosd(phi) sind(phi);...
    0 -sind(phi) cosd(phi)];

R2 = [cosd(theta) 0 -sind(theta);...
    0 1 0;...
    sind(theta) 0 cosd(theta)];

R3 = [cosd(psi) sind(psi) 0;...
    -sind(psi) cosd(psi) 0;...
    0 0 1];

R = R1*R2*R3;

new_coords = R * coords';

x = reshape(new_coords(1, :),size(x));
y = reshape(new_coords(2, :), size(y));
z = reshape(new_coords(3, :), size(z));

x_vec = x(:);
y_vec = y(:);
z_vec = z(:);

Length = max(x) - min(x) ;

coord_up = [x_vec,y_vec,z_vec];
Cross = struct();
maxz = 0;

% Find the cross sections at different fuselage stations provided in FS
% First = [y_vec(x_vec == x(1),:)  z_vec(x_vec == x(1),:)];
for i = 1:length(FS)-1
    x1 = min(x) + FS(i) * Length;
    diff = abs(coord_up(:,1) - x1);
    difftemp = diff;
    threshold = 0.005;
    diff = coord_up(diff(:,1)<threshold, :);
    while isempty(diff) == 1
            threshold = 1.1*threshold;
            diff = coord_up(difftemp(:,1)<threshold, :);
    end
    
    RefPt = [median(diff(:,2)),median(diff(:,3))];
    zdim = diff(:,3) - RefPt(2);
    ydim = diff(:,2) - RefPt(1); 

    th = atan2d(zdim,ydim);
    Pol = [th ydim zdim];
    Pol(:,1) = Pol(:,1) + 180;
    Pol = sortrows(Pol,1);
    [~, uniqueIdx] = unique(Pol(:,1), 'stable');
    Pol = Pol(uniqueIdx,:);
    thref = linspace(0,360,75)';
    InPol = interp1(Pol(:,1),[Pol(:,2),Pol(:,3)],thref,'linear','extrap');
    InPol = [thref InPol];
    SorPol = InPol(InPol(:,1) >= 90 & InPol(:,1) <= 270, :);
    SorPol = flipud(SorPol);
    SorPol(:,3) = SorPol(:,3) ./ max(SorPol(:,2));
    SorPol(:,2) = SorPol(:,2) ./ max(SorPol(:,2));

    SorPolup = SorPol(SorPol(:,1) > 180 & SorPol(:,1) <= 270, :);
    SorPoldn = SorPol(SorPol(:,1) <= 180 & SorPol(:,1) >= 90, :);

    % if any(SorPolup(:,2) > 0.999) && any(SorPoldn(:,2) > 0.999)
    %     SorPolup = SorPolup(SorPolup(:,2) < 0.999,:);
    %     SorPolup(end,2) = 1; 
    %     SorPolup(end,3) = 0;
    % 
    %     SorPoldn = SorPoldn(SorPoldn(:,2) < 0.999,:);
    % end
    CS = [SorPolup(:,2:3) ; SorPoldn(:,2:3)];
    
    
    % [a,~] = convhull(diff(:,2),diff(:,3));
    % diff = diff(a,:);
    % y_values = diff(:,2);
    % mid_y = mean([min(y_values), max(y_values)]);
    % z_values = diff(:,3);
    % diff(:,2) = diff(:,2) - mid_y;
    % y_values = y_values - mid_y;
    % max_y = max(y_values);
    % mid_y = mean([min(y_values), max(y_values)]);
    % coord_x1_rt = diff(diff(:,2) >= mid_y,2:3);
    % coord_x1_lft = diff(diff(:,2) <= mid_y,2:3);
    % if length(coord_x1_lft) > length(coord_x1_rt)
    %     coord_x1_rt = coord_x1_lft;
    %     coord_x1_rt(:,1) = -coord_x1_lft(:,1);
    % end

    




    % z_max = mean([max(z_values),min(z_values)]);
    % coord_ups = coord_x1_rt(coord_x1_rt(:,2) >= z_max,:);
    % coord_downs = coord_x1_rt(coord_x1_rt(:,2) <= z_max,:);
    % 
    % % coord_upstemp = coord_ups;
    % % coord_downstemp = coord_downs;
    % [~, uniqueIdx] = unique(coord_ups(:,2), 'stable');
    % coord_ups = coord_ups(uniqueIdx,:);
    % coord_ups = sortrows(coord_ups,1,'ascend');
    % 
    % [~, uniqueIdx] = unique(coord_downs(:,2), 'stable');
    % coord_downs = coord_downs(uniqueIdx,:);
    % coord_downs = sortrows(coord_downs,1,'ascend');
    % 
    % yinterp1 = linspace(mid_y,max_y,45)';
    % % yinterp1 = [yinterp1(1:end-1) ; linspace(yinterp1(end-1),yinterp1(end),20)'];
    % yinterp2 = yinterp1(1:end-1);
    % 
    % % Sort the index based on unique values in first column
    % [~, uniqueIdxup] = uniquetol(coord_ups(:,1), 1e-7);
    % coord_ups = coord_ups(uniqueIdxup,:);
    % [~, uniqueIdxdown] = uniquetol(coord_downs(:,1), 1e-7);
    % coord_downs = coord_downs(uniqueIdxdown,:);
   
    % allIdxup = 1:size(coord_upstemp, 1);
    % nonUniqueIdxup = setdiff(allIdxup, uniqueIdxup);
    % 
    % allIdxdown = 1:size(coord_downstemp, 1);
    % nonUniqueIdxdown = setdiff(allIdxdown, uniqueIdxdown);

    
    % Find out either up or down coordinates have the max y value
    % if (max_y - max(coord_ups(:,1))) <= (max_y - max(coord_downs(:,1)))
    %     new_co_up = interp1(coord_ups(:,1),coord_ups(:,2),yinterp1,'linear','extrap');
    %     new_co_down = interp1(coord_downs(:,1),coord_downs(:,2),yinterp2,'linear','extrap');
    %     new = [yinterp1, new_co_up  ; flipud(yinterp2), flipud(new_co_down)];
    % else
    %     new_co_up = interp1(coord_ups(:,1),coord_ups(:,2),yinterp2,'linear','extrap');
    %     new_co_down = interp1(coord_downs(:,1),coord_downs(:,2),yinterp1,'linear','extrap');
    %     new = [yinterp2, new_co_up  ; flipud(yinterp1), flipud(new_co_down)];
    % end
    % 
    % new(:,2) = new(:,2) - mean([max(new(:,2)),min(new(:,2))]);
    % new(:,2) = new(:,2) ./ max(new(:,1));
    % new(:,1) = new(:,1) ./ max(new(:,1)); 
    % 
    % CS = new;
    % [~,CSmax] = max(CS(:,1));
    % if CS(CSmax,2) ~= 0
    %     CS(CSmax,2) = 0;
    % end
    Cross.(['CS' num2str(i)]) = CS;
    % maxz = max(maxz,max(abs(CS(:,2))));
end

for i = 2: length(FS) - 1
    CSField = ['CS',num2str(i-1)];
    CSFields = ['CS',num2str(i)];
    Off = Cross.(CSField)(1,2) - Cross.(CSFields)(1,2);
    Cross.(CSFields)(:,2) = Cross.(CSFields)(:,2) + Off;
end


end




