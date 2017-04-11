figure(22)
hold on 
for i=1:length(xs) 
 for j=1:length(xs)
outv=Z*[xs(i,j);ys(i,j);zs(i,j)];
surf(outv(1),outv(2),outv(3),'*')
 end
end 
xlabel('x')
ylabel('y')
zlabel('z')
view([0 0]);
