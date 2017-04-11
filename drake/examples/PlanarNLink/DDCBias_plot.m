function DDCBias_plot(Zstar,Cstar,n)

figure (30)
Atemp=Zstar(:,:,n);
Ctemp=Cstar(:,:,n);
Ellipse_plot(Atemp,Ctemp)
BiasVector(Cstar,n)
% if i>1
% quiver(0,0,Cstar(1,:,i-1), Cstar(2,:,i-1))
% end 
drawnow 
end 
function Ellipse_plot(A, C, varargin)
%
%  Ellipse_Plot(A,C,N) plots a 2D ellipse or a 3D ellipsoid 
%  represented in the "center" form:  
%               
%                   (x-C)' A (x-C) <= 1
%
%  A and C could be the outputs of the function: "MinVolEllipse.m",
%  which computes the minimum volume enclosing ellipsoid containing a 
%  set of points in space. 
% 
%  Inputs: 
%  A: a 2x2 or 3x3 matrix.
%  C: a 2D or a 3D vector which represents the center of the ellipsoid.
%  N: the number of grid points for plotting the ellipse; Default: N = 20. 
%
%  Example:
%  
%       P = rand(3,100);
%       t = 0.001;
%       [A , C] = MinVolEllipse(P, t)
%       figure
%       plot3(P(1,:),P(2,:),P(3,:),'*')
%       hold on
%       Ellipse_plot(A,C)
%  
%
%  Nima Moshtagh
%  nima@seas.upenn.edu
%  University of Pennsylvania
%  Feb 1, 2007
%  Updated: Feb 3, 2007

%%%%%%%%%%%  start  %%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 20; % Default value for grid

% See if the user wants a different value for N.
%----------------------------------------------
if nargin > 2
 	N = varargin{1};
end


% check the dimension of the inputs: 2D or 3D
%--------------------------------------------
if length(C) == 3,
    Type = '3D';
elseif length(C) == 2,
    Type = '2D';
elseif length(C) == 1,
    Type = 'Degenrate';
    disp('degenrate ellipse')
else
    display('Cannot plot an ellipse with more than 3 dimensions!!');
    return
end

% "singular value decomposition" to extract the orientation and the
% axes of the ellipsoid
[U D V] = svd(A);

if strcmp(Type, 'Degenerate'),
    % get the major and minor axes
    %------------------------------------
    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));

    theta = [0:1/N:2*pi+1/N];

    % Parametric equation of the ellipse
    %----------------------------------------
    state(1,:) = a*cos(theta); 
    state(2,:) = b*sin(theta);

    % Coordinate transform 
    %----------------------------------------
    X = V * state;
    X(1,:) = X(1,:) + C;
    X(2,:) = X(2,:) + C;

elseif strcmp(Type, '2D'),
    % get the major and minor axes
    %------------------------------------
    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));

    theta = [0:1/N:2*pi+1/N];

    % Parametric equation of the ellipse
    %----------------------------------------
    state(1,:) = a*cos(theta); 
    state(2,:) = b*sin(theta);

    % Coordinate transform 
    %----------------------------------------
    X = V * state;
    X(1,:) = X(1,:) + C(1);
    X(2,:) = X(2,:) + C(2);
    
elseif strcmp(Type,'3D'),
    % generate the ellipsoid at (0,0,0)
    %----------------------------------
    a = 1/sqrt(D(1,1))
    b = 1/sqrt(D(2,2))
    c = 1/sqrt(D(3,3))
    [X,Y,Z] = ellipsoid(0,0,0,a,b,c,N);
    
    %  rotate and center the ellipsoid to the actual center point
    %------------------------------------------------------------
    XX = zeros(N+1,N+1);
    YY = zeros(N+1,N+1);
    ZZ = zeros(N+1,N+1);
    for k = 1:length(X),
        for j = 1:length(X),
            point = [X(k,j) Y(k,j) Z(k,j)]';
            P = V * point;
            XX(k,j) = P(1)+C(1);
            YY(k,j) = P(2)+C(2);
            ZZ(k,j) = P(3)+C(3);
        end
    end
end


% Plot the ellipse
%----------------------------------------
if strcmp(Type,'2D'),
    plot(X(1,:),X(2,:),'linewidth',2);
title('Dynamic Coupling Map');
% %title('$Joint\:Effort\: Profile$','interpreter','latex','FontSize',20);
 xlabel('Horizontal acceleration $(m/s^{2})$','interpreter','latex','FontSize',25);
 ylabel('Vertical acceleration $(m/s^{2})$','interpreter','latex','FontSize',25);
% ylabel('$q \: (rad)$','interpreter','latex','FontSize',20);
    hold on;
    plot(C(1),C(2),'r*');
    axis square
    set(gca,'FontSize',30)
    grid on
    
elseif strcmp(Type,'Degenerate'),
    plot(X(1,:),X(2,:));
    hold on;
    plot(C(1),C(2),'r*');
    axis equal
    grid on 
else
    mesh(XX,YY,ZZ);
    %surf(XX,YY,ZZ);
    disp('plotting 3D')
    axis equal
    hidden off
end
end 
function BiasVector(Xbias2D,n)
    dx=Xbias2D(1,:,n)-Xbias2D(1,:,n-1);
    dy=Xbias2D(2,:,n)-Xbias2D(2,:,n-1);
    h1=quiver(Xbias2D(1,:,n-1), Xbias2D(2,:,n-1),dx,dy,0)%'Autoscale','on')
   set(h1,'linewidth',2);
 %quiver(Cstar(1,:,n-1),Cstar(2,:,n-1),Cstar(1,:,n), Cstar(2,:,n))
end 
