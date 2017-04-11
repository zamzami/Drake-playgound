figure 
hold on 
for n=2:20
    start=[Xbias2D(1,:,n-1),Xbias2D(2,:,n-1)];
    fin=[Xbias2D(1,:,n), Xbias2D(2,:,n)];
    dx=Xbias2D(1,:,n)-Xbias2D(1,:,n-1);
    dy=Xbias2D(2,:,n)-Xbias2D(2,:,n-1);
    h1=quiver(Xbias2D(1,:,n-1), Xbias2D(2,:,n-1),dx,dy,0)%'Autoscale','on')
    %xlim([-10 10])
    %ylim([-20 20])
 %qscale = 1; % scaling factor for all vectors
 %hU = get(h1,'UData') ;
 %hV = get(h1,'VData') ;
 %set(h1,'UData',qscale*hU,'VData',qscale*hV)
% hU = get(h2,'UData') ;
% hV = get(h2,'VData') ;
% set(h2,'UData',qscale*hU,'VData',qscale*hV)
    %line(start',fin')
end
 