function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.


%map sequence:
%for given element row: 
%xmin,ymin,zmin,xmax,ymax,zmax,block(0) or open(1),element index

%Last row contains entire domain's
%xmin,ymin,zmin,xmax,ymax,zmax,xy_res,z_res

xmin=map(1,1);    ymin=map(1,2);    zmin=map(1,3);
xmax=map(1,4);    ymax=map(1,5);    zmax=map(1,6);
margin=map(end,3);
figure(1);
hold on;
axis equal vis3d;
axis([xmin xmax ymin ymax zmin zmax]);
grid on;
view(45,45);

 for i=2:size(map,1)-1
     plot_block(map(i,1)+margin,map(i,2)+margin,map(i,3)+margin,map(i,4)-margin,map(i,5)-margin,map(i,6)-margin,[1 0 0]);
 end

%plotting path
for i=1:(size(path,1)-1)
    plot3([path(i,1),path(i+1,1)],[path(i,2),path(i+1,2)],[path(i,3),path(i+1,3)],'b');
end
    
hold off;
end