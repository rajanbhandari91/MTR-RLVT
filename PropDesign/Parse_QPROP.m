function [OPData] = Parse_QPROP(OutputFileName,Analysis)


[fid,errmsg] = fopen(OutputFileName,'r');
fprintf('Parsing QPROP file...')

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


if c < 143
    sprintf('Bad Array!')
end

for i = 1:r
    tempArray(i,:) = [Array(i,1:64),'   ',Array(i,65:143),'  ',Array(i,144:end)];
end
Array = tempArray;


%% Parse out Propeller Data
[LineNum, ~] = SearchForString(Array,'#  V(m/s)');
[FiltNum, ~] = SearchForString(Array,'GVCALC: Not converged.',1,50);
if numel(FiltNum) == 50
    [FiltNum, ~] = SearchForString(Array,'GVCALC: Not converged.',1,100);
end

% [LineNum, ~] = SearchForString(Array,'#  V(m/s)    rpm      Dbeta      T(N)       Q(N-m)    Pshaft(W)    Volts     Amps    effmot   effprop   adv       CT          CP        DV(m/s)   eff     Pelec       Pprop        cl_avg  cd_avg')
% Header = textscan(Array(LineNum,:), '%s');
ReadLine = (LineNum + 1):1:r;

for i = 1:numel(FiltNum)
    ReadLine = ReadLine(ReadLine ~= FiltNum(i));
end

% Output_Data = zeros(numel(ReadLine),19);
Empty = [];


if strcmpi(Analysis,'PointPerf')
     imax = 1;
     readstart = 1;
else
    imax = numel(ReadLine);
    readstart = 0;
end

    k = 0;
    
    for i = 1:imax
    
        temp = textscan(Array(ReadLine(i),:), '%s');
        yy=temp{1};
        
        [nr,~] = size(temp{1});
          
        if nr~=0
            
            k = k+1;
            trial = str2double(yy');
        
            OP.V(k,1) = trial(readstart+1);
            OP.rpm(k,1) = trial(readstart+2);
            OP.Dbeta(k,1) = trial(readstart+3);
            OP.T_N(k,1) = trial(readstart+4);
            OP.Q_Nm(k,1) = trial(readstart+5);
            OP.Pshaft_W(k,1) = trial(readstart+6);
            OP.cl_avg(k,1) = trial(end-1);
            OP.cd_avg(k,1) = trial(end);
            
        end
              
        if isempty(temp{1})
            Empty = [Empty,i];
        else
  
            Output_Data = 1;
        end
    end
    
    % calculate the efficiency
    OP.effprop = OP.T_N.*OP.V./OP.Pshaft_W;
    
    OPData = struct2table(OP);
    

fprintf('Parsing complete \n')

fclose(fid);







