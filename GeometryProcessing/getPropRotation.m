function PropRotation = getPropRotation(degenGeom, PropLoc,Name)

conv_ft_m = 0.30488;
index = find(arrayfun(@(s) strcmp(s.name, Name), degenGeom));
for i = 1:length(index)
    % Extract the coordinates of the points in x, y, and z axis.
    x = degenGeom(index(i)).surf.x ;
    y = degenGeom(index(i)).surf.y ;
    z = degenGeom(index(i)).surf.z ;

    xa = x(:) .* conv_ft_m;
    ya = y(:) .* conv_ft_m;
    za = z(:) .* conv_ft_m;

    coords = [xa';ya';za'];

    if any(ya > PropLoc(2))

        [max_y1,~] = max(ya(:,1));
        maxidy1 = find(ya == max_y1);
        x_maxy1 = mean(xa(maxidy1));
        z_maxy1 = mean(za(maxidy1));
        % x_maxy1 = xa(maxidy1); z_maxy1 = za(maxidy1);

        % Capture the points
        Tip = [x_maxy1 max_y1 z_maxy1];
        RefPt = PropLoc - [0 0.2*PropLoc(2) 0];

        % Compute the vectors in the xy-plane
        v1 = Tip(1:2) - PropLoc(1:2);
        v2 = RefPt(1:2) - PropLoc(1:2);

        % Compute the dot product of the vectors
        dot_product = dot(v1, v2);

        % Compute the magnitudes of the vectors
        magnitude_v1 = norm(v1);
        magnitude_v2 = norm(v2);

        % Compute the angle in radians
        cos_psi = dot_product / (magnitude_v1 * magnitude_v2);
        psi_deg = acosd(cos_psi);
        psi_deg(psi_deg<1) = 0;

        % Rotate the coordinates using theta_degrees
        phi = 0;
        theta = 0;
        psi = -psi_deg;

        %Translate coordinates
        coords = coords - PropLoc';
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

        new_coords = R * coords;

        scatter3(coords(1,:),coords(2,:),coords(3,:))
        hold on
        scatter3(new_coords(1,:),new_coords(2,:),new_coords(3,:))
        break
    end
end






end