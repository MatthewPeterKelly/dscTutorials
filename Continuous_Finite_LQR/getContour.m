function C = getContour(g,data,level)

%This function is used to obtain a set of level-curves for a surface
%function. Used for computing backward reachable set.

% There is some weird meshgrid vs ndgrid stuff going on here. 

% MESHGRID vs NDGRID  --  switch x and y  
c = contourc(g.vs{2}, g.vs{1}, data, [level, level]);

if isempty(c)
    C(1).x = [];
    C(1).y = [];
else
    idx = 1;  %Index of the next delimiter
    len = size(c,2);  %Total number of indicies
    count = 0;  %Check number of segments
    while idx<len  %Count the number of segments:
        count = count + 1;
        idx = idx + 1 + c(2,idx);
    end
    
    C(count).x = [];
    C(count).y = [];
    idx= 1;
    for i=1:count
        idxLow = idx + 1;
        idxUpp = idx + c(2,idx);
        % MESHGRID vs NDGRID  --  switch x and y  
        C(i).y = c(1,idxLow:idxUpp);
        C(i).x = c(2,idxLow:idxUpp);
        idx = idxUpp + 1;
    end
end

end
