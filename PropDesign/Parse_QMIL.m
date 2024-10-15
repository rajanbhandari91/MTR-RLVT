function [OPData] = Parse_QMIL(OutputFileName)

% OutputFileName = 'Output\Testing.out';

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

%% Parse out Propeller Geometry
[LineNum, ~] = SearchForString(Array,'#        r            c        beta');

Header = textscan(Array(LineNum,:), '%s');
ReadLine = (LineNum + 1):1:r;

Output_Data = zeros(numel(ReadLine),3);
Empty = [];
for i = 1:numel(ReadLine)
    temp = cell2mat(textscan(Array(ReadLine(i),:), '%f'));
    if isempty(temp)
        Empty = [Empty,i];
    else
        Output_Data(i,:) = temp(:)';
    end
end

Output_Data(Empty,:) = [];
%%


OPData.r = Output_Data(:,1);
OPData.c = Output_Data(:,2);
OPData.beta = Output_Data(:,3);

OPData = struct2table(OPData);

fclose(fid);