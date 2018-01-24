function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end

if(isempty(map))
    path=double.empty(0,3); num_expanded=0;
    return;
end

xmin=map(1,1);  ymin=map(1,2);  zmin=map(1,3);
xmax=map(1,4);  ymax=map(1,5);  zmax=map(1,6);
xy_res=map(end,1);  z_res=map(end,2);

%accounting for large resolution numbers
if(xy_res>min(xmax,ymax))
    xy_res=min(xmax,ymax);
end

if(z_res>zmax)
    z_res=zmax;
end

start_node_ind=[];  start_node_dist=inf;
goal_node_ind=[];   goal_node_dist=inf;
path=start;
num_expanded=0;

%% DISCRETIZATION, DEMARKATION & FINDING CLOSEST NODES %%

X=xmin:xy_res:xmax; 
if (round(X(end),2)~=round(xmax,2))     %generating X, Y and Z. 
   X=[X,xmax];                          %Also if xmax is not part of X, then include xmax as well.    
end

Y=ymin:xy_res:ymax;
if (round(Y(end),2)~=round(ymax,2))
    Y=[Y,ymax]; 
end

Z=zmin:z_res:zmax;
if (round(Z(end),2)~=round(zmax,2))
    Z=[Z,zmax];
end

nx=numel(X); ny=numel(Y); nz=numel(Z);
totnodes=nx*ny*nz;

node=zeros(totnodes,4);

nodeind=0;

for yloop=1:numel(Y)
     for zloop=1:numel(Z)
         for xloop=1:numel(X)
             
             nodeind=nodeind+1;
             
             node(nodeind,1:3)=[X(xloop),Y(yloop),Z(zloop)];
             
             
             %checking if node is in block
             
             if (collide(map,[X(xloop),Y(yloop),Z(zloop)]))
                 node(nodeind,4)=0;
             else
                 node(nodeind,4)=1;
             end
             
             %search for closest start and goal node indices
             if (sqrt((X(xloop)-start(1))^2+(Y(yloop)-start(2))^2+(Z(zloop)-start(3))^2)<start_node_dist)
                 start_node_dist=sqrt((X(xloop)-start(1))^2+(Y(yloop)-start(2))^2+(Z(zloop)-start(3))^2);
                 start_node_ind=nodeind;
             end
             
             if (sqrt((X(xloop)-goal(1))^2+(Y(yloop)-goal(2))^2+(Z(zloop)-goal(3))^2)<goal_node_dist)
                 goal_node_dist=sqrt((X(xloop)-goal(1))^2+(Y(yloop)-goal(2))^2+(Z(zloop)-goal(3))^2);
                 goal_node_ind=nodeind;
             end
             
         end
     end
end


%% DIJKSTRA %%
 
  g=inf(totnodes,1); %defualt distance from start
  gtemp=inf(totnodes,1);
  
  P(totnodes)=0;     %parent history
  
  Q=1:totnodes;      %all elements not visited
   
  g(start_node_ind)=0;    %setting first node's distance to 0
  gtemp(start_node_ind)=0;

  
  
  path=[path;node(start_node_ind,1:3)];    %second point on path
 

  %tic;
  %loop begins
  while Q(goal_node_ind)~=-1 && min(g(Q>0))<inf
      
      
      [dummy,utemp]=min(gtemp);
      
      u=utemp(1);
      
      Q(u)=-1;   %setting u as discovered
      gtemp(u)=inf;
      
%      tic;
      
      %neighboring elements
     neighbor_nodes=[];

     %xright 
     if u<totnodes &&(round((node(u,1)+xy_res),2)==round(node(u+1,1),2))...
             && node(u+1,4)==1 && Q(u+1)~=-1   
        neighbor_nodes(end+1)=u+1;
     end
     
      %xleft
     if u>1 && (round(node(u,1),2)==round((node(u-1,1)+xy_res),2))...
             && node(u-1,4)==1 && Q(u-1)~=-1
        neighbor_nodes(end+1)=u-1;
     end
     
     %yfront
     if u<(totnodes-nx*nz) && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz,2),2))...
             && node(u+nx*nz,4)==1 && Q(u+nx*nz)~=-1
        neighbor_nodes(end+1)=u+nx*nz;
     end
    
    %yback
     if u>nx*nz && (round(node(u,2),2)==round((node(u-nx*nz,2)+xy_res),2))...
             && node(u-nx*nz,4)==1 && Q(u-nx*nz)~=-1
        neighbor_nodes(end+1)=u-nx*nz;
     end
     
     %zup 
     if u<(totnodes-nx) && (round((node(u,3)+z_res),2)==round(node(u+nx,3),2))...
              && node(u+nx,4)==1 && Q(u+nx)~=-1
        neighbor_nodes(end+1)=u+nx;
     end
      
      %zdown
      if u>nx && (round(node(u,3),2)==round((node(u-nx,3)+z_res),2))...
              && node(u-nx,4)==1 && Q(u-nx)~=-1
        neighbor_nodes(end+1)=u-nx;
      end
      
     %xy-right-front
     if u<totnodes && u<(totnodes-nx*nz) && (round((node(u,1)+xy_res),2)==round(node(u+nx*nz+1,1),2))... 
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz+1,2),2)) && node(u+nx*nz+1,4)==1 && Q(u+nx*nz+1)~=-1
         neighbor_nodes(end+1)=u+nx*nz+1;
     end
     
     %xy-left-front
     if u>1 && u<(totnodes-nx*nz) && (round((node(u,1)+xy_res),2)==round(node(u+nx*nz-1,1),2))...
        && (round((node(u,2)+xy_res),2)==round(node(u+nx*nz-1,2),2)) && node(u+nx*nz-1,4)==1 && Q(u+nx*nz-1)~=-1
        neighbor_nodes(end+1)=u+nx*nz-1;
     end
     
     %xy-back-left
     if u>1 && u>nx*nz+1 && (round(node(u,1),2)==round((node(u-1-nx*nz,1)+xy_res),2))...
         && (round(node(u,2),2)==round((node(u-1-nx*nz,2)+xy_res),2)) && node(u-1-nx*nz,4)==1 && Q(u-1-nx*nz)~=-1
            neighbor_nodes(end+1)=u-nx*nz-1;
     end
     
     %xy-back-right
     if u<totnodes && u>nx*nz && (round((node(u,1)+xy_res),2)==round(node(u+1-nx*nz,1),2))...
        && (round((node(u,2)+xy_res),2)==round(node(u+1-nx*nz,2),2)) && node(u+1-nx*nz,4)==1 && Q(u+1-nx*nz)~=-1
            neighbor_nodes(end+1)=u+1-nx*nz;
     end
      
      %zy-up-front
      if u<(totnodes-nx-nx*nz) && (round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz,2),2))...
        && (round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz,3),2)) && node(u+nx+nx*nz,4)==1 && Q(u+nx+nx*nz)~=-1
        neighbor_nodes(end+1)=u+nx+nx*nz;
     end
     
    %zy-down-front
     if u<(totnodes-nx*nz) && u>nx && (round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz,2),2))... 
         && (round(node(u,3),2)==round((node(u-nx+nx*nz,3)+z_res),2)) && node(u-nx+nx*nz,4)==1 && Q(u-nx+nx*nz)~=-1
        neighbor_nodes(end+1)=u-nx+nx*nz;
     end
     
     %xz-right-up
      if u<(totnodes-nx) && (round((node(u,3)+z_res),2)==round(node(u+nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u+nx+1,1),2)) && node(u+nx+1,4)==1 && Q(u+nx+1)~=-1   
        neighbor_nodes(end+1)=u+nx+1;
      end
      
     %xz-right-down
     if u<totnodes && u>nx && (round((node(u,3)+z_res),2)==round(node(u-nx+1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u-nx+1,1),2)) && node(u-nx+1,4)==1 && Q(u-nx+1)~=-1   
        neighbor_nodes(end+1)=u-nx+1;
     end
     
     %xz-left-up
     if u>1 && u<(totnodes-nx) && (round((node(u,3)+z_res),2)==round(node(u+nx-1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u+nx-1,1),2)) && node(u+nx-1,4)==1 && Q(u+nx-1)~=-1   
        neighbor_nodes(end+1)=u+nx-1;
     end
     
      %xz-left-down
     if u>nx+1 && u<(totnodes-nx) && (round((node(u,3)+z_res),2)==round(node(u-nx-1,3),2))...
            &&(round((node(u,1)+xy_res),2)==round(node(u-nx-1,1),2)) && node(u-nx-1,4)==1 && Q(u-nx-1)~=-1   
        neighbor_nodes(end+1)=u-nx-1;
     end
     
     %front-up-right
     if u<(totnodes-nx-nx*nz-1) && round((node(u,1)+xy_res),2)==round(node(u+nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz+1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz+1,3),2)...
         && node(u+nx+nx*nz+1,4)==1 && Q(u+nx+nx*nz+1)~=-1
        neighbor_nodes(end+1)=u+nx+nx*nz+1;
     end
     
     %front-up-left
     if u<(totnodes-nx-nx*nz) && round(node(u,1),2)==round(node(u+nx+nx*nz-1,1)+xy_res,2)...
         && round((node(u,2)+xy_res),2)==round(node(u+nx+nx*nz-1,2),2) && round((node(u,3)+z_res),2)==round(node(u+nx+nx*nz-1,3),2)...
         && node(u+nx+nx*nz-1,4)==1 && Q(u+nx+nx*nz-1)~=-1
        neighbor_nodes(end+1)=u+nx+nx*nz-1;
     end
     
     %front-down-right
     if u>nx && u<(totnodes-nx*nz-1) && round((node(u,1)+xy_res),2)==round(node(u-nx+nx*nz+1,1),2)...
         && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz+1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz+1,3),2)...
         && node(u-nx+nx*nz+1,4)==1 && Q(u-nx+nx*nz+1)~=-1
        neighbor_nodes(end+1)=u-nx+nx*nz+1;
     end
     
%      %front-down-left
%      if u>nx && u<(totnodes-nx*nz) && round((node(u,1)+xy_res),2)==round(node(u-nx+nx*nz+1,1),2)...
%          && round((node(u,2)+xy_res),2)==round(node(u-nx+nx*nz+1,2),2) && round((node(u,3)-z_res),2)==round(node(u-nx+nx*nz+1,3),2)...
%          && node(u-nx+nx*nz+1,4)==1 && Q(u-nx+nx*nz+1)~=-1
%         neighbor_nodes(end+1)=u-nx+nx*nz+1;
%      end
      
       %ntime=ntime+toc;
       
       consider_points=neighbor_nodes;
       
     %tic;
     for i=1:numel(consider_points)
         
         dist=g(u)+sqrt((node(consider_points(i),1)-node(u,1))^2+...
                        (node(consider_points(i),2)-node(u,2))^2+...
                        (node(consider_points(i),3)-node(u,3))^2);
              
              if astar
                  dist=dist+sqrt((node(consider_points(i),1)-node(goal_node_ind,1))^2+...
                                 (node(consider_points(i),2)-node(goal_node_ind,2))^2+...
                                 (node(consider_points(i),3)-node(goal_node_ind,3))^2);
              end
              
              if dist<g(consider_points(i))
                  g(consider_points(i))=dist;
                  gtemp(consider_points(i))=dist;
                  P(consider_points(i))=u;
                  
              end
     
     end
      %ntime=ntime+toc;
 
     num_expanded=num_expanded+1;
  end
  
  %toc;
  %disp(ntime);
  
 temp_path=[];
 u=goal_node_ind;

 while u~=start_node_ind
     
     temp_path=[node(u,1:3);temp_path];
     u=P(u);
     
     if u==0
         disp('NO PATH FOUND');
         path=double.empty(0,3);
         return;
     end
 end
 
 path=[path;temp_path;goal];

end