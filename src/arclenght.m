function [a,s] = arclenght(tf,tc,t)
a = -1/(tc*tc - tf*tc);
s = zeros(size(t,1),1);
for i = 1:size(t)
    if t(i)<=tc
        s(i) = 0.5*a*t(i)*t(i);
    elseif t(i)>tc && t(i) <= tf-tc
        s(i) = a *tc*(t(i)-tc/2);
    else
        s(i) = 1 - 0.5*a*(tf-t(i))^2;
    end
end
end