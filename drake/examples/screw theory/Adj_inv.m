function Adjinv = Adj_inv(g)
R = g(1:3,1:3);
p = g(1:3,4);
Adjinv = [R.'  -R.'*skew(p);...
          zeros(3,3) R.'];
end