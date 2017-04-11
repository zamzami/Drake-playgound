function Adj = Adj(g)
R = g(1:3,1:3);
p = g(1:3,4);
Adj = [R skew(p)*R;...
      zeros(3,3) R];
end
