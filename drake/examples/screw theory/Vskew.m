function Vhat=Vskew(V)
omega=V(1:3,:);
v=V(4:6,:);
omegahat=skew(omega);

Vhat=[omegahat v; zeros(1,3) 0]; 
end 