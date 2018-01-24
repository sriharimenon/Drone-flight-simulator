function plot_block(xmin, ymin, zmin, xmax, ymax, zmax,c)

% figure(1);
% hold on;
% axis equal vis3d;
% axis([0.0 10.0 -5.0 20.0 0.0 6.0]);
% grid on;

%zmin
fill3([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],[zmin,zmin,zmin,zmin],c);

%zmax
fill3([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],[zmax,zmax,zmax,zmax],c);

%xmin
fill3([xmin,xmin,xmin,xmin],[ymin,ymin,ymax,ymax],[zmin,zmax,zmax,zmin],c);

%xmax
fill3([xmax,xmax,xmax,xmax],[ymin,ymin,ymax,ymax],[zmin,zmax,zmax,zmin],c);

%ymin
fill3([xmin,xmin,xmax,xmax],[ymin,ymin,ymin,ymin],[zmin,zmax,zmax,zmin],c);

%ymax
fill3([xmin,xmin,xmax,xmax],[ymax,ymax,ymax,ymax],[zmin,zmax,zmax,zmin],c);


%hold off;

end