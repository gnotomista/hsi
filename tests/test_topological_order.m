clc
clear
close all

N = 6;
topological_order = cell((N^2-N)/2,1);

n = 0;
for i = 1 : N
    for j = 1 : N
        if i~=j && j>i
            n = n + 1;
            topological_order{n} = [i,j];
        end
    end
end

idx = get_index(topological_order, [3,4])
topological_order{idx}
