%exampleSVD
%Geometric interpretation of the singular value decomposition
%H Richter, CSU 2015

%A=[-4 0 1;-1 -1 0;76/5 11/10 11/10]; %some 2x2 matrix
%A=[- 2*2^(1/2) - 1, -1, 1;2*2^(1/2), 0, 0;5*2^(1/2) + 61/5, (3*2^(1/2))/2 + 21/10, 11/10]; % 0;pi/4;pi/4;0
A=[- 2*2^(1/2) , 0, 1;2*2^(1/2)+1, 1, 0;5*2^(1/2) + 56/5, (3*2^(1/2))/2 + 11/10, 11/10];
%sweep the plane with a unit vector in polar coordinates
%compute images and plot
figure(1)
hold on
axis equal

% th=[0:0.01:2*pi];
% phi=[0:0.01:pi];
% for t=1:length(phi),
%     for i=1:length(th),
%     x=cos(th(i))*sin(phi(t));
%     y=sin(th(i))*sin(phi(t));
%     z=cos(phi(t));
%     outv=A*[x;y;z];
%     surf(outv(1),outv(2),outv(3),'k*')
%     end
% end
[x y z]=sphere(50);
%plot3(x, y, z)
for i=1:length(x) 
 for j=1:length(x)
outv=A*[x(i,j);y(i,j);z(i,j)];
plot3(outv(1),outv(2),outv(3),'-')
colormap winter
 end
end 

grid
%Compare amplification factors in singular directions

[U,S,V]=svd(A)
%Process max amplification direction
Vmax=U(:,1);
sigma1=S(1,1)
umaxout=A*Vmax;
plot3([0 umaxout(1)],[0 umaxout(2)],[0 umaxout(3)],'r') 
plot3([0 sigma1*Vmax(1)],[0 sigma1*Vmax(2)],[0 sigma1*Vmax(3)],'--r','LineWidth', 2) %this should line up the ellipse's major axis
%norm(umaxout) %this should match the max svd of A (since Vmax has norm=1)
%norm(A,2) %this should also match the max svd of A by definition of 2-norm 
%umaxout/norm(umaxout) %this should match one of the U columns!

%Process min amplification direction
Vmin=U(:,2);
sigma2=S(2,2)
uminout=A*Vmin;
plot3([0 uminout(1)],[0 uminout(2)],[0 uminout(3)],'b')
plot3([0 sigma2*Vmin(1)],[0 sigma2*Vmin(2)], [0 sigma2*Vmin(3)],'--b','LineWidth',2)%this should line up the ellipse's minor axis
%norm(uminout) %this should match the min svd of A (since Vmin has norm=1)
%uminout/norm(uminout) %this should match the other U column

%Process min amplification direction
Vmid=U(:,3);
sigma3=S(3,3)
umidout=A*Vmid;
plot3([0 umidout(1)],[0 umidout(2)],[0 umidout(3)],'g')
plot3([0 sigma3*Vmid(1)],[0 sigma3*Vmid(2)], [0 sigma3*Vmid(3)],'--g','LineWidth',2)%this should line up the ellipse's minor axis
%norm(umidout) %this should match the min svd of A (since Vmin has norm=1)
%umidout/norm(umidout) %this should match the other U column

figure(2)
[xe,ye,ze] = ellipsoid(0,0,0,sigma1,sigma2,sigma3,20)
surf(xe,ye,ze)
colormap winter
axis equal
xlabel('x')
ylabel('y')
zlabel('phi')

%Ellipse_plot(A,[0;0;0],50)


%%%
% Graphical check
% Generate many points on ellipse
[U S V] = svd(Q);
s = diag(S);
radii = sqrt(1./s);
A = U*diag(radii);

clf;
if n==2
    theta = linspace(0,2*pi,181);
    ellipse = A*[cos(theta); sin(theta)]; 
    
    plot(ellipse(1,:),ellipse(2,:));
    axis equal;
    hold on;
    % Plot c projected to the ellipse
    plot(c(1),c(2),'ok');
    plot([x(1) c(1)],[x(2) c(2)],'-r.');
elseif n==3
    N = 64;
    [X Y Z] = ellipsoid(0,0,0,radii(1),radii(2),radii(3),N);
    XYZ = U*[X(:) Y(:) Z(:)]';
    X = reshape(XYZ(1,:),[N N]+1);
    Y = reshape(XYZ(2,:),[N N]+1);
    Z = reshape(XYZ(3,:),[N N]+1);
    surf(X,Y,Z);
    axis equal;
    hold on;
    % Plot c projected to the ellipse
    plot3(c(1),c(2),c(3),'ok');
    plot3([x(1) c(1)],[x(2) c(2)],[x(3) c(3)],'-r.');
end



for i=1:length(xs) 
 for j=1:length(xs)
outv=Z_numerique*[xs(i,j);ys(i,j);zs(i,j)];
plot3(outv(1),outv(2),outv(3),'-')
 end
end 


