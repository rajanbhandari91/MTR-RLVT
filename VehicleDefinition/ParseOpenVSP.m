function Parameters = ParseOpenVSP(filename)

filenamemat = [filename,'_DegenGeom.m'];
run(filenamemat);

% Initialize an empty structure to store the results
BodyComp = struct('name', {}, 'type', {});
LiftingSurfComp = struct('name', {}, 'type', {});
DiskComp = struct('name', {}, 'type', {});
Parameters = struct();

% Iterate through degenGeom and find entries with type 'BODY'
for i = 1:length(degenGeom)
    if strcmp(degenGeom(i).type, 'BODY')
        % Append to the new structure
        BodyComp(end+1).name = degenGeom(i).name;
        BodyComp(end).type = degenGeom(i).type;
        BodyComp(end).GeomID = degenGeom(i).geom_id;
    end

    if strcmp(degenGeom(i).type, 'LIFTING_SURFACE')
        % Append to the new structure
        LiftingSurfComp(end+1).name = degenGeom(i).name;
        LiftingSurfComp(end).type = degenGeom(i).type;
        LiftingSurfComp(end).GeomID = degenGeom(i).geom_id;
    end

    if strcmp(degenGeom(i).type, 'DISK')
        % Append to the new structure
        DiskComp(end+1).name = degenGeom(i).name;
        DiskComp(end).type = degenGeom(i).type;
        DiskComp(end).GeomID = degenGeom(i).geom_id;
    end
end

% Remove the symmetric components for lifting surfaces
identifier = strcat({BodyComp.name},{BodyComp.type},string([BodyComp.GeomID]));
[~,uniqueidx] = unique(identifier);
BodyComp = BodyComp(uniqueidx);

% Remove the symmetric components for Body components
identifier = strcat({LiftingSurfComp.name},{LiftingSurfComp.type},string([LiftingSurfComp.GeomID]));
[~,uniqueidx] = unique(identifier);
LiftingSurfComp = LiftingSurfComp(uniqueidx);


filenameext = [filename,'.vsp3'];
doc = xmlread(filenameext);
VehicleNodes = doc.getElementsByTagName('Vehicle');
VehicleNode = VehicleNodes.item(0);
GeomNodes = VehicleNode.getElementsByTagName('Geom');

% Extract parameters from body component
count = 1;
for i = 1: length(BodyComp)
    try
        CompName = BodyComp(i).name;
        Fus = getParamFus(GeomNodes,degenGeom,CompName);
        Parameters.Fus.(['Fus_',num2str(count)]) = Fus ;
    catch
        count = count - 1;
    end
    count = count + 1;
end

matchingFields = structfun(@(s) isfield(s, 'Symmetry') && s.Symmetry == 2, Parameters.Fus);

% Get the field names where the condition is true
fieldsWithSymmetry = fieldnames(Parameters.Fus);
fieldsWithSymmetry = fieldsWithSymmetry(matchingFields);

for j = 1: length(fieldsWithSymmetry)
    FUSfieldNames = fieldnames(Parameters.Fus);
    if ~isempty(fieldsWithSymmetry)
        % Get the name of the first matching field
        fieldToCopy = fieldsWithSymmetry{j};
        newFieldNumber = length(FUSfieldNames) + 1;
        newFieldName = strcat('Fus_', num2str(newFieldNumber));

        % Copy the content of the original field to the new field
        Parameters.Fus.(newFieldName) = Parameters.Fus.(fieldToCopy);

        % Append the y location of new field
        Parameters.Fus.(newFieldName).Location(2) = - Parameters.Fus.(fieldToCopy).Location(2);
        Parameters.Fus.(newFieldName).Name = [Parameters.Fus.(newFieldName).Name,'_2'];

        Parameters.Fus.(fieldToCopy).Name = [Parameters.Fus.(fieldToCopy).Name,'_1'];

    end
end



% Extract parameters from lifting surface component
count = 1;
for i = 1: length(LiftingSurfComp)
    try
        CompName = LiftingSurfComp(i).name;
        Wing = getParamLiftingSurface(GeomNodes,degenGeom,CompName);
        Parameters.LS.(['LS_',num2str(count)]) = Wing ;
    catch
        count = count - 1;
    end
    count = count + 1;
end
% LS_struct = Parameters.LS;
% LS_fields = fieldnames(Parameters.LS);
% nonWingIdx = cellfun(@(f) ~strcmp(LS_struct.(f).Type, 'Wing'), LS_fields);
% fieldsToRemove = LS_fields(nonWingIdx);
% LS_struct = rmfield(LS_struct, fieldsToRemove);
% LS_fields = fieldnames(LS_struct);
% names = arrayfun(@(f) LS_struct.(f{1}).Name, LS_fields, 'UniformOutput', false);
% [~, uniqueIdx] = unique(names, 'stable');
% uniqueFields = LS_fields(uniqueIdx);
% LS_unique = rmfield(LS_struct, setdiff(LS_fields, uniqueFields));
% %Add back the removed fields at the end
% for i = 1:numel(fieldsToRemove)
%     LS_unique.(fieldsToRemove{i}) = Parameters.LS.(fieldsToRemove{i});
% end
%
% Parameters.LS = LS_unique;


% Extract parameters from Disk component
count = 1;
for i = 1: length(DiskComp)
    try
        CompName = DiskComp(i).name;
        Rotor = getParamRotors(GeomNodes,filename,CompName);
        Parameters.Disk.(['Rotor_',num2str(i)]) = Rotor;
    catch
        count = count - 1;
    end
    count = count + 1;
end








