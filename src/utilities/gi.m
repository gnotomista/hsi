function out = gi(h)
%AD  Performs the adjoint transform
%
%	A = AD(x)

  if isnumeric(h)
     out = zeros(3,2, size(h,3));
  end

  for i=1:size(h,3)
    r = h(1:2,1:2,i);
    p = h(1:2,3,i);

    out(1:2,1:2,i) = r;
    out(3,1:2,i) = [p(2) -p(1)]*r;
  end
  
end
