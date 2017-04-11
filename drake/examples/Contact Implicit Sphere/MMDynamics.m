syms m0 m1 m2 mleg l0 l1 c1 cleg c2 phi alpha q1 q2 I0 I1 I2 Ileg xdd ydd phidd q1dd q2dd tauX tauY tauPhi q1tau q2tau

assume(q1>0 & q1>2*pi)
assume(q2>0 & q2>2*pi)
assume(phi>0 & phi>2*pi)
assume(alpha>0 & alpha>2*pi)
assume(c1 > 0)
assume(l0 > 0)
assume(l1 > 0)
assume(m1 > 0)
assume(m2 > 0)
assume(m0 > 0)
assume(I0 > 0)
assume(I1 > 0)
assume(I2 > 0)
assume(Ileg > 0)



M=sym(zeros(5,5));
% M(1,1)=m0+m1+m2
% M(2,2)=m0+m1+m2
% M(1,3)=-(m1+m2)*l0*cos(phi)-(m1*c1+m2*(c1+l1))*sin(phi+q1)-m2*c2*sin(phi+q1+q2)
% M(1,4)=-(m1*c1+m2*(c1+l1))*sin(phi+q1)-m2*c2*sin(phi+q1+q2)
% M(1,5)=-m2*c2*sin(phi+q1+q2)
% M(2,3)=-(m1+m2)*l0*sin(phi)+(m1*c1+m2*(c1+l1))*cos(phi+q1)+m2*c2*cos(phi+q1+q2)
% M(2,4)=(m1*c1+m2*(c1+l1))*cos(phi+q1)+m2*c2*cos(phi+q1+q2)
% M(2,5)=m2*c2*cos(phi+q1+q2)
% M(3,3)=I0+I1+I2+m1*((l0)^2+(c1)^2)+m2*((l0)^2+(c1+l1)^2+(c2)^2)+2*(m1*c1+m2*(c1+l1))*l0*sin(q1)+2*m2*(c1+l1)*c2*cos(q2)+2*((m2*l0*c2)*sin(q1+q2))
% M(3,4)=I1+I2+m1*(c1^2)+m2*((c1+l1)^2+c2^2)+m2*l0*c2*sin(q1+q2)+(m1*c1+m2*(c1+l1))*l0*sin(q1)+2*m2*(c1+l1)*c2*cos(q2)
% M(3,5)=I2+m2*(c2^2)+m2*l0*c2*sin(q1+q2)+m2*(c1+l1)*c2*cos(q2)
% M(4,4)=I1+I2+m1*(c1^2)+m2*((c1+l1)^2+c2^2)+2*m2*(c1+l1)*c2*cos(q2)
% M(4,5)=I2+m2*(c2^2)+m2*(c1+l1)*c2*cos(q2)
% M(5,5)=I2+m2*(c2^2)

% with leg 
M(1,1)=m0+m1+m2+mleg;
M(2,2)=m0+m1+m2+mleg;
M(1,3)=-(m1+m2)*l0*cos(phi)-(m1*c1+m2*(c1+l1))*sin(phi+q1)-m2*c2*sin(phi+q1+q2)+cleg*mleg*cos(alpha + phi);
M(1,4)=-(m1*c1+m2*(c1+l1))*sin(phi+q1)-m2*c2*sin(phi+q1+q2);
M(1,5)=-m2*c2*sin(phi+q1+q2);
M(1,6)=cleg*mleg*cos(alpha + phi);
M(2,3)=-(m1+m2)*l0*sin(phi)+(m1*c1+m2*(c1+l1))*cos(phi+q1)+m2*c2*cos(phi+q1+q2);
M(2,4)=(m1*c1+m2*(c1+l1))*cos(phi+q1)+m2*c2*cos(phi+q1+q2);
M(2,5)=m2*c2*cos(phi+q1+q2);
M(2,6)=cleg*mleg*-sin(alpha + phi);
M(3,3)=I0+I1+I2+Ileg+m1*((l0)^2+(c1)^2)+m2*((l0)^2+(c1+l1)^2+(c2)^2)+2*(m1*c1+m2*(c1+l1))*l0*sin(q1)+2*m2*(c1+l1)*c2*cos(q2)+2*((m2*l0*c2)*sin(q1+q2))+mleg*cleg^2;
M(3,4)=I1+I2+m1*(c1^2)+m2*((c1+l1)^2+c2^2)+m2*l0*c2*sin(q1+q2)+(m1*c1+m2*(c1+l1))*l0*sin(q1)+2*m2*(c1+l1)*c2*cos(q2);
M(3,5)=I2+m2*(c2^2)+m2*l0*c2*sin(q1+q2)+m2*(c1+l1)*c2*cos(q2);
M(3,6)=mleg*cleg^2+Ileg;
M(4,4)=I1+I2+m1*(c1^2)+m2*((c1+l1)^2+c2^2)+2*m2*(c1+l1)*c2*cos(q2);
M(4,5)=I2+m2*(c2^2)+m2*(c1+l1)*c2*cos(q2);
M(4,6)=0;
M(5,5)=I2+m2*(c2^2);
M(5,6)=0;
M(6,6)=Ileg+mleg*cleg^2;



%%%add a leg 



M=M+M.'-diag(diag(M))
Mbb=M(1:3,1:3)
Mqq=M(4:6,4:6)
M_bq=M(1:3,4:6)


% 
% qdd=[xdd; ydd; phidd; q1dd; q2dd]
% Tau=[tauX; tauY; tauPhi; q1tau; q2tau]
% 
% %Tau=M*qdd
% 
% tauX= xdd*(m0 + m1 + m2) - q1dd*(sin(phi + q1)*(c1*m1 + m2*(c1 + l1)) + c2*m2*sin(phi + q1 + q2)) - phidd*(sin(phi + q1)*(c1*m1 + m2*(c1 + l1)) + c2*m2*sin(phi + q1 + q2) + l0*cos(phi)*(m1 + m2)) - c2*m2*q2dd*sin(phi + q1 + q2)
% 
% tauX_max= xdd*(m0 + m1 + m2) - q1dd*((c1*m1 + m2*(c1 + l1)) + c2*m2) - phidd*((c1*m1 + m2*(c1 + l1)) + c2*m2 + l0(m1 + m2)) - c2*m2*q2dd;
%     q2dd*(m2*c2^2 + I2) + q1dd*(m2*c2^2 + m2*cos(q2)*(c1 + l1)*c2 + I2) + phidd*(I2 + c2^2*m2 + c2*m2*cos(q2)*(c1 + l1) + c2*l0*m2*sin(q1 + q2)) + c2*m2*ydd*cos(phi + q1 + q2) - c2*m2*xdd*sin(phi + q1 + q2)
%     
% %%%Mass matrix analysis %%%
% 
% %Determinant of coupling matrix
% det_coupling=c1*c2*(m2)^2*sin(phi + q1 + q2)*cos(phi + q1) - c1*c2*m2^2*cos(phi + q1 + q2)*sin(phi + q1) - c2*l1*m2^2*cos(phi + q1 + q2)*sin(phi + q1) + c2*l1*m2^2*sin(phi + q1 + q2)*cos(phi + q1) - c1*c2*m1*m2*cos(phi + q1 + q2)*sin(phi + q1) + c1*c2*m1*m2*sin(phi + q1 + q2)*cos(phi + q1)
% 
% %Deterimenant of coupling matrix with leg %%
%  
% det_coupling_leg=Ileg*c1*c2*m2^2*sin(phi + q1 + q2)*cos(phi + q1) - Ileg*c1*c2*m2^2*cos(phi + q1 + q2)*sin(phi + q1) - Ileg*c2*l1*m2^2*cos(phi + q1 + q2)*sin(phi + q1) + Ileg*c2*l1*m2^2*sin(phi + q1 + q2)*cos(phi + q1) - Ileg*c1*c2*m1*m2*cos(phi + q1 + q2)*sin(phi + q1) + Ileg*c1*c2*m1*m2*sin(phi + q1 + q2)*cos(phi + q1) + I1*c2*cleg*m2*mleg*sin(phi + q1 + q2)*sin(alpha + phi) + c1*c2^2*cleg*m2^2*mleg*cos(alpha + phi)*cos(phi + q1) + c2^2*cleg*l1*m2^2*mleg*cos(alpha + phi)*cos(phi + q1) - c1*c2^2*cleg*m2^2*mleg*sin(alpha + phi)*sin(phi + q1) + I2*c1*cleg*m1*mleg*cos(alpha + phi)*cos(phi + q1) + I2*c1*cleg*m2*mleg*cos(alpha + phi)*cos(phi + q1) - c2^2*cleg*l1*m2^2*mleg*sin(alpha + phi)*sin(phi + q1) + I2*cleg*l1*m2*mleg*cos(alpha + phi)*cos(phi + q1) - I2*c1*cleg*m1*mleg*sin(alpha + phi)*sin(phi + q1) - I2*c1*cleg*m2*mleg*sin(alpha + phi)*sin(phi + q1) - I2*cleg*l1*m2*mleg*sin(alpha + phi)*sin(phi + q1) - c1^2*c2*cleg*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi) - c2*cleg*l1^2*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi) - c1*c2*cleg^2*m2^2*mleg*cos(phi + q1 + q2)*sin(phi + q1) + c1*c2*cleg^2*m2^2*mleg*sin(phi + q1 + q2)*cos(phi + q1) + c1^2*c2*cleg*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi) - I1*c2*cleg*m2*mleg*cos(phi + q1 + q2)*cos(alpha + phi) - c2*cleg^2*l1*m2^2*mleg*cos(phi + q1 + q2)*sin(phi + q1) + c2*cleg^2*l1*m2^2*mleg*sin(phi + q1 + q2)*cos(phi + q1) + c2*cleg*l1^2*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi) + c1^2*c2*cleg*m2^2*mleg*cos(alpha + phi)*cos(phi + q1)*cos(q2) + c2*cleg*l1^2*m2^2*mleg*cos(alpha + phi)*cos(phi + q1)*cos(q2) - c1^2*c2*cleg*m2^2*mleg*sin(alpha + phi)*sin(phi + q1)*cos(q2) - c2*cleg*l1^2*m2^2*mleg*sin(alpha + phi)*sin(phi + q1)*cos(q2) + c1*c2^2*cleg*m1*m2*mleg*cos(alpha + phi)*cos(phi + q1) - c1*c2^2*cleg*m1*m2*mleg*sin(alpha + phi)*sin(phi + q1) - c1*c2^2*cleg*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi)*cos(q2) - c2^2*cleg*l1*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi)*cos(q2) + c1*c2^2*cleg*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi)*cos(q2) + c2^2*cleg*l1*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi)*cos(q2) - 2*c1*c2*cleg*l1*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi) - c1^2*c2*cleg*m1*m2*mleg*cos(phi + q1 + q2)*cos(alpha + phi) + 2*c1*c2*cleg*l1*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi) - c1*c2*cleg^2*m1*m2*mleg*cos(phi + q1 + q2)*sin(phi + q1) + c1*c2*cleg^2*m1*m2*mleg*sin(phi + q1 + q2)*cos(phi + q1) + c1^2*c2*cleg*m1*m2*mleg*sin(phi + q1 + q2)*sin(alpha + phi) + c1*c2*cleg*l0*m2^2*mleg*cos(alpha + phi)*cos(phi + q1)*sin(q1 + q2) + c2*cleg*l0*l1*m2^2*mleg*cos(alpha + phi)*cos(phi + q1)*sin(q1 + q2) + 2*c1*c2*cleg*l1*m2^2*mleg*cos(alpha + phi)*cos(phi + q1)*cos(q2) - c1*c2*cleg*l0*m2^2*mleg*sin(alpha + phi)*sin(phi + q1)*sin(q1 + q2) + c1^2*c2*cleg*m1*m2*mleg*cos(alpha + phi)*cos(phi + q1)*cos(q2) - c2*cleg*l0*l1*m2^2*mleg*sin(alpha + phi)*sin(phi + q1)*sin(q1 + q2) - 2*c1*c2*cleg*l1*m2^2*mleg*sin(alpha + phi)*sin(phi + q1)*cos(q2) - c1^2*c2*cleg*m1*m2*mleg*sin(alpha + phi)*sin(phi + q1)*cos(q2) - c1*c2*cleg*l0*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi)*sin(q1) - c2*cleg*l0*l1*m2^2*mleg*cos(phi + q1 + q2)*cos(alpha + phi)*sin(q1) + c1*c2*cleg*l0*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi)*sin(q1) + c2*cleg*l0*l1*m2^2*mleg*sin(phi + q1 + q2)*sin(alpha + phi)*sin(q1) + c1*c2*cleg*l0*m1*m2*mleg*cos(alpha + phi)*cos(phi + q1)*sin(q1 + q2) + c1*c2*cleg*l1*m1*m2*mleg*cos(alpha + phi)*cos(phi + q1)*cos(q2) - c1*c2*cleg*l0*m1*m2*mleg*sin(alpha + phi)*sin(phi + q1)*sin(q1 + q2) - c1*c2*cleg*l1*m1*m2*mleg*sin(alpha + phi)*sin(phi + q1)*cos(q2) - c1*c2*cleg*l0*m1*m2*mleg*cos(phi + q1 + q2)*cos(alpha + phi)*sin(q1) + c1*c2*cleg*l0*m1*m2*mleg*sin(phi + q1 + q2)*sin(alpha + phi)*sin(q1)
% 
% %subs(det_coupling_leg,[m0, m1, m2, c1, c2, l1,I1,I2 phi, q1, q2,alpha],[1, 1, 1, 1, 1, 2,0.1,0.1, 0,0,0,0])
% subs(Z,[m0, m1, m2, mleg,c1, c2, cleg,l0,l1,I1,I2,Ileg,I0 phi, q1, q2,alpha],[1, 1, 1,1, 1, 1, 1,1,2,0.1,0.1,0.1,0.2,0,0,0,0])
% 
% 
