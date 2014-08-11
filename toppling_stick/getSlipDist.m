function dist = getSlipDist(D)

%This function computes the amount that the stick slid in each direction

N = length(D.phase);
dist = zeros(N,1);

for i=1:N
   if strcmp(D.phase{i},'SLIDE_POS') 
       dist(i) = range(D.raw(i).state.x);
   elseif strcmp(D.phase{i},'SLIDE_NEG') 
       dist(i) = -range(D.raw(i).state.x);
   end
end

end