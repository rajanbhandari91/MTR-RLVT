function [Header , Output_Data] = Parse_XFOIL(OutputFileName)

% OutputFileName = 'Airfoil\RAF6.polar';

[fid,errmsg] = fopen(OutputFileName,'r');

if(isempty(errmsg)~=1)
    error('\n Error: Failed to parse from %s - %s \n', OutputFileName, errmsg);
end

% Get number of lines
%# Get file size.
fseek(fid, 0, 'eof');
fileSize = ftell(fid);
frewind(fid);
%# Read the whole file.
data = fread(fid, fileSize, 'uint8');
%# Count number of line-feeds and increase by one.
numLines = sum(data == 10) + 1;
frewind(fid);


for i = 1:1:numLines-1
    l{i} = fgetl(fid);
end

Array = char(l);
[r,c] = size(Array);

%% Parse out Airfoil Data
[LineNum, ~] = SearchForString(Array,'alpha');

Header = textscan(Array(LineNum,:), '%s');
ReadLine = LineNum + 2;

for i = ReadLine:r
    temp = cell2mat(textscan(Array(i,:), '%f'));
    Output_Data((i - ReadLine)+1,:) = temp(:)';
end



fclose(fid);