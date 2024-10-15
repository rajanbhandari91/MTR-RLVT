function [LineNum, LineChars, HitStatus] = SearchForString(Array,String,varargin)

% If additional inputs are not provided (varargin) to the contrary, 
% (1) begin the search at the top and
% (2) stop at the first hit
counter = 1;
StopAtOccurrence = 1;

% If an optional third input is provided, then the third input is the
% starting line. Set the counter equal to this
if nargin==3 || nargin==4
    if ~isempty(varargin{1})
        counter = varargin{1};
    end
end

% If an optional fourth input is provided, then the fourth input is the
% number of occurrences (hits) at which the search is to be stopped
if nargin==4
    if strcmpi(varargin{2},'all')
        StopAtOccurrence = 9999999999;
    else
        StopAtOccurrence = varargin{2};
    end
end


[nrow,ncol] = size(Array);

found = [];

HitCounter = 0;
HitStatus = 0;
LineNum = [];



while counter <=nrow
 %disp('entered here')
        found =  strfind(Array(counter,:),String);
        
        if ~isempty(found)
            HitCounter = HitCounter + 1;
            LineNum(HitCounter) = counter;
        end
        
        counter = counter + 1;
        
        if HitCounter == StopAtOccurrence
            break;
        end

end

if HitCounter > 0
    HitStatus = 1;
end



%disp('got to here')
LineChars = Array(LineNum,:);



