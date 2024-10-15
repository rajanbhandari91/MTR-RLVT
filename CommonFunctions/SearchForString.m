function [LineNum, LineChars] = SearchForString(Array,String)

counter = 1;
[nrow,ncol] = size(Array);

found = [];


while isempty(found) && counter <nrow
    counter = counter + 1;
    found =  strfind(Array(counter,:),String);
end



LineNum = counter;
LineChars = Array(counter,:);



