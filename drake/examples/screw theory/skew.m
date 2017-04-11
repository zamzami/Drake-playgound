function mat=skew(r)

mat=[0 -r(3) r(2);
    r(3) 0  -r(1);
    -r(2) r(1) 0];
%mat=transpose(matT);
end 
