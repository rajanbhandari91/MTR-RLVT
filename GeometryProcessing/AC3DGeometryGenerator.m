function [out] = AC3DGeometryGenerator(Vehicle,Settings)
out = 0;

    % capture the Geometry part of the Vehicle structure
    ac3dGEOM = Vehicle.Geom;
    
    % remove all components that do not have vertices and surfaces defined
    Components = fieldnames(ac3dGEOM);
    for i = 1:length(Components)
        if ~isfield( ac3dGEOM.(Components{i}),'Vertices') && ~isfield( ac3dGEOM.(Components{i}),'Surfaces')
            ac3dGEOM = rmfield(ac3dGEOM,Components{i});
        elseif  isfield( ac3dGEOM.(Components{i}),'Vertices') && isfield( ac3dGEOM.(Components{i}),'Surfaces')
            
            if isempty( ac3dGEOM.(Components{i}).Vertices) && isempty( ac3dGEOM.(Components{i}).Surfaces)
                ac3dGEOM = rmfield(ac3dGEOM,Components{i});
            end
        end
    end
    
    % get list of trimmed down components
    Components = fieldnames(ac3dGEOM);


    % running over all the components...
    for i = 1:length(Components)
        % if the component is a lifting surface...
        if strcmpi(ac3dGEOM.(Components{i}).Type,'liftingsurface')
            % if it has control surfaces defined...
            if~isempty(ac3dGEOM.(Components{i}).Controls)
                % set parent
                Parent = ac3dGEOM.(Components{i});
                ParentName = Components{i};
                % find out how many controls are defined...
                [~,nctrl] = size(Parent.Controls);                
                % loop over each control
                for j = 1:nctrl
                    % capture the control
                    CaptCtrl = Parent.Controls(j);
                    % name control as (PARENT NAME)_(CONTROL NAME)
                    CtrlName = strcat(ParentName,'_',Parent.Controls(j).Name);
                    % add it to the Geom struct
                    ac3dGEOM.(CtrlName).Name = CtrlName;
                    % add its vertices
                    ac3dGEOM.(CtrlName).Vertices = CaptCtrl.Vertices;
                    % add its surfaces
                    ac3dGEOM.(CtrlName).Surfaces = CaptCtrl.Surfaces;
                    
                end
            end
            
        end
    end
    
%     makeAC3D(geom, filename, mats,PlotMode,TrackPts)
        
    [VertColl,VertColl_FSBLWL]=makeAC3D(ac3dGEOM, strcat(Settings.SaveFileNameac3d,'.ac'), [], 0, [],Settings.ac3dFLAG);
    
    
end