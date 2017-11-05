function idx = get_index(topological_order, ij)

idx = cellfun(@(x) all(x==ij), topological_order);

end