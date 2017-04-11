syms c1 c2 l0 l1 q1 q2 qd1 qd2 phi  omegaZ Vbx Vby m1 m2 I1x I1y I1z mb Ibx Iby Ibz I2x I2y I2z

assume(q1>0 & q1>2*pi)
assume(q2>0 & q2>2*pi)
assume(phi>0 & phi>2*pi)
assume(c1 > 0)
assume(l0 > 0)
assume(l1 > 0)
assume(m1 > 0)
assume(m2 > 0)
assume(mb > 0)
assume(I1x > 0)
assume(I1y > 0)
assume(I1z > 0)
assume(I2x > 0)
assume(I2y > 0)
assume(I2z > 0)

qdot=[qd1; qd2]
V0b=[Vbx;Vby;0;0;0;omegaZ]
V0b_hat=Vskew(V0b);

I1=diag([m1;m1;m1;I1x; I1y; I1z])
I2=diag([m2;m2;m2;I2x; I2y; I2z])
Ib=diag([mb;mb;mb;Ibx; Iby; Ibz])

Mb=[Ib zeros(6,length(qdot));zeros(length(qdot),6) zeros(length(qdot))] 

H=diag([1; 1; 0; 0; 0; 1])

%%%% Joint 1 to COM 1 Arm %%%%
noRot= diag([1 1 1]);

R11=noRot; 
P11=[0; c1; 0];

g11=[R11 P11;zeros(1,3) 1]

Adg11=Adj(g11)  %%Adjoint 
Adg11_inverse=Adj_inv(g11)
Adg11_inv=simplify(Adg11_inverse)

xi=[0;0;0;0;0;1];

Xi_1=Adg11_inv*xi

J1=[Xi_1 0]
Vb1=J1*qdot
M1=J1.'*I1*J1
%%%%base to link 1%%
Rb1=rotZ(q1);
%Pb1=[-c1*sin(q1);l0+c1*cos(q1);0]
Pb1=[-c1*sin(q1+phi)-l0*cos(phi);l0*sin(phi)+c1*cos(q1+phi);0]
gb1=[Rb1 Pb1;zeros(1,3) 1]

Adgb1=Adj(gb1)  %%Adjoint 
Adgb1_inverse=Adj_inv(gb1)
Adgb1_inv=simplify(Adgb1_inverse)


V01_term1=Adgb1_inv*V0b

Vb1_ad=Adgb1_inv*Vb1

V01=V01_term1+Vb1
V01_ad=V01_term1+Vb1_ad

%%%Book Version
%JG=[Adgb1_inv Adgb1_inv*J1]
%M1_G=JG.'*I1*JG
%M1G_simple=simplify(M1_G)

Jg1=[Adgb1_inv J1]
M1_SE=Jg1.'*I1*Jg1
M2_simple=simplify(M1_SE)

MVV=simplify(Adgb1_inv.'*I1*Adgb1_inv)
MVV_plan=H.'*MVV*H
MVV_plan(:,[3 4 5])=[]
MVV_plan([3 4 5],:)=[]
Mqq=M1
MVq=Adgb1_inv.'*I1*J1
MqV=J1.'*I1*Adgb1_inv

Mtot=Mb+M2_simple
%%% MASS MATRIX%%%%%
%    ___                                                      ????
%   |                                                            |
%   |    (Ad_g_bi)^T* I_i* Ad_g_bi       (Ad_g_bi)^T* I_i *J_i   |
%   |                                                            |   
%   |                                                            |   
%   |     (J_i)^T *I_i* Ad_g_bi             J_i *I_i *J_i        |
%   ?????                                                       ??


%%%% Joint 2 to COM 2 Arm %%%%
noRot= diag([1 1 1]);

R22=noRot; 
P22=[0; c2; 0];

g22=[R22 P22;zeros(1,3) 1];

 Adg22=Adj(g22);
Adg22_inverse=Adj_inv(g22);
Adg22_inv=simplify(Adg22_inverse);
Xi_22=Adg22_inv*xi
%%%%%%%% Joint 1 to COM 2 Arm%%%
       
     R12=rotZ(q2);
     P12=[-c2*sin(q2);l1+c2*cos(q2);0]
     g12=[R12 P12;zeros(1,3) 1]
     
  Adg12=Adj(g12)
 Ad_inverse12=Adj_inv(g12)
 Adg12_inv=simplify(Ad_inverse12)
 Xi_12=Adg12_inv*xi
 
 J2=[Xi_12 Xi_22]
 M2=J2.'*I2*J2
 Vb2=J2*qdot
 %%%%%%%% Base to COM 2 Arm%%%
       
     Rb2=rotZ(q2+q1);
     Pb2=[-l1*sin(q1)-c2*sin(q2+q1);l0+l1+c2*cos(q2+q1);0]
     gb2=[R12 P12;zeros(1,3) 1]
     
  Adgb2=Adj(gb2)
 Ad_inverseb2=Adj_inv(gb2)
 Adgb2_inv=simplify(Ad_inverseb2)
 
 Jg2=[Adgb2_inv J2]
 M2_SE=Jg2.'*I2*Jg2
M2_simple=simplify(M2_SE)

MVV2=simplify(Adgb2_inv.'*I2*Adgb2_inv)
MVV2_plan=H.'*MVV2*H
MVV2_plan(:,[3 4 5])=[]
MVV2_plan([3 4 5],:)=[]
Mqq2=simplify(M2)
MVq2=simplify(Adgb2_inv.'*I2*J2)

V02_term1=Adgb2_inv*V0b
V02=V02_term1+Vb2

 
 
 
