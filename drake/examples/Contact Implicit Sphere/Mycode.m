syms d0 d1 q1  
dof=3

R=zeros(3,3);
P=zeros(3,1);

     g= [R(1,1) R(1,2) R(1,3) P(1,1); ...
        R(2,1) R(2,2) R(2,3) P(2,1); ...
        R(3,1) R(3,2) R(3,3) P(3,1); ...
        0 0 0 1];
    R1=rotZ(q1);
    P1=[-d1*sin(q1);d0+d1*cos(q1);0]
    g1=[R1 P1;zeros(1,3) 1]
 
    
 Ad=Adj(g1)
Ad_inverse=Adj_inv(g1)
