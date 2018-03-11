function G = centroid(P)
n = size(P,2);
M = [0 1;-1 0];
A = 0;
S = 0;
for i = 1 : n
    ri = P(:,i);
    if i < n
        j = i + 1;
    else
        j = 1;
    end
    rj = P(:,j);
    rjo = M * rj;
    A = A + ri'*rjo;
    S = S + (ri' * rjo * (ri + rj));
end
A = A / 2;
S = S / 6;
G = S / A;
end