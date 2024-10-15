function Cross = interpolateCS(Fus,nfs,CS_ind,fs)
CSunique = length(unique(CS_ind));
CSzinter = Fus.CrossSections(1).CS;
thref = linspace(0,180,45)';
thCSinter = atan2d(CSzinter(:,1),CSzinter(:,2));
radCSinter = sqrt(CSzinter(:,1).^2 + CSzinter(:,2).^2);
radCSinter = interp1(thCSinter,radCSinter,thref,'linear','extrap');
CSzinter = [radCSinter .* cosd(thref)];
CSind1 = height(CS_ind(CS_ind == 1));
CSzinter = repmat(CSzinter,1,CSind1);
radCSinter = repmat(radCSinter,1,CSind1);
for i = 2: CSunique
    CS1 = Fus.CrossSections(i-1).CS;
    thCS1 = atan2d(CS1(:,1),CS1(:,2));
    radCS1 = sqrt(CS1(:,1).^2 + CS1(:,2).^2);
    radCS1 = interp1(thCS1,radCS1,thref,'linear','extrap');

    CS2 = Fus.CrossSections(i).CS;
    thCS2 = atan2d(CS2(:,1),CS2(:,2));
    radCS2 = sqrt(CS2(:,1).^2 + CS2(:,2).^2);
    radCS2 = interp1(thCS2,radCS2,thref,'linear','extrap');

    radinterp = [];
    %
    % try
    for j = 1 : height(thref)
        radinterp(j,:) = interp1([Fus.CrossSections(i).FS(1) Fus.CrossSections(i).FS(2)], [radCS1(j) radCS2(j)]...
            ,fs(CS_ind == i)','linear','extrap');
    end
    % catch
    % end
    CSinterp = [radinterp .* cosd(thref)];
    CSzinter = [CSzinter CSinterp];
    radCSinter = [radCSinter radinterp];
end
CSyinter = [radCSinter .* sind(thref)];

Cross = arrayfun(@(i) struct('CS', [CSyinter(:,i), CSzinter(:,i)]), 1:size(CSyinter, 2));

