function [Lapl,topological_order] = build_laplacian(graph_topology_mat_file_name)

load(graph_topology_mat_file_name)
Lapl = zeros(N);
for i = 1 : N
    for j = 1 : N
        if i~=j && (any(get_index(topological_order,[i,j])) || any(get_index(topological_order,[j,i])))
            Lapl(i,j) = -1;
        end
    end
    Lapl(i,i) = -sum(Lapl(i,:));
end

end