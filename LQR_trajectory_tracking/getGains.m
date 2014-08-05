function [K, xNom, uNom] = getGains(T,t)

%Uses linear interpolation between two gain matricies

N = length(T);

if t<=T(1).t
    idx = [1,2];
    alpha = 0;
elseif t>=T(N).t
    idx = [(N-1),N];
    alpha = 1;
else
    for i=2:N
        if t<T(i).t
            break
        end
    end
    idx = [i-1,i];
    alpha = (T(i).t - t)/(T(i).t-T(i-1).t);
end

K = alpha*T(idx(1)).K + (1-alpha)*T(idx(2)).K;
xNom =alpha*T(idx(1)).x + (1-alpha)*T(idx(2)).x;
uNom = alpha*T(idx(1)).u + (1-alpha)*T(idx(2)).u;

end