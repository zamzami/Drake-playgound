figure 
hold on 
for radius = 0:0.1:1
theta = linspace(0,2*pi);
phi = linspace(0,2*pi);
[theta,phi] = meshgrid(theta,phi);
[xs,ys,zs] = sph2cart(theta,phi,radius);
surf(xs,ys,zs);
end