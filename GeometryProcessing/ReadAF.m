function AF = ReadAF(LS,index)

Name = [pwd,'\',LS.AirfoilName{index}.name,'.dat'];
% Open the file
fid = fopen(Name, 'r');

% Skip the first line (text)
fgetl(fid);

% Read the remaining data as numeric values
data = textscan(fid, '%f %f', 'CollectOutput', true);

% Close the file
fclose(fid);

% Store the numeric data in the variable 'AF'
AF = data{1};
AF(:,3) = zeros(length(AF(:,1)),1);

end