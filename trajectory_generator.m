function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.


persistent map0 path0 x y z vx vy vz ax ay az corner_times

average_velocity = 1.5; %average velocity for the entire course. Variable parameter

%assumptions: the path is made of straight lines and indifferentiable sharp
%corners

if (isempty(t)) && (isempty(qn))
    
    if iscell(path)
        path=cell2mat(path);
    end
    
    map0 = map;
    path0 = path;
    
    %figuring out sharp corners
    
    corners=[];
    
    for i=2:size(path,1)-1
        
        if round(path(i,1)-path(i-1,1),2)~=round(path(i+1,1)-path(i,1),2) ||...
           round(path(i,2)-path(i-1,2),2)~=round(path(i+1,2)-path(i,2),2) ||...
           round(path(i,3)-path(i-1,3),2)~=round(path(i+1,3)-path(i,3),2)
            corners=[corners;path(i,:)];
        end
    end
    
    corners=[path(1,:);corners;path(end,:)]; % all corners including fisrt and last points
    
    corners=optipath(corners,map);
    
    %calculating distances of each sector
    
    sect_len=[];    %individual sector lengths
    sect_dist=[];   %cumulative distances from start
    
    %calculating individual distances
    for i=1:size(corners,1)-1
        sect_len=[sect_len;sqrt(sum((corners(i,:)-corners(i+1,:)).^2))];
    end
    
    %calculating cumulative distances
    sect_dist=sect_len;
    for i=2:size(sect_dist)
        sect_dist(i)=sect_dist(i-1)+sect_dist(i);
    end
    
    
    tot_dist=sect_dist(end);     %the total distance
    
    T=tot_dist/average_velocity; %the total time it takes
    
    sect_time=T*sect_len./tot_dist;
        
    tot_sectors=size(sect_dist,1);
    
    tot_time=zeros(tot_sectors+1,1);
    
    %giving more time to shorter sectors, less time to longer sectors
    for i=1:tot_sectors
        if sect_time(i)<0.05*T
            sect_time(i)=sect_time(i)+sect_time(i);%+0.05*T;
        end
        
        if sect_time(i)>0.3*T
            sect_time(i)=sect_time(i)-0.5*sect_time(i);%-0.1*T;
        else
            if sect_time(i)>0.15*T
                sect_time(i)=sect_time(i)-0.25*sect_time(i);%-0.05*T;
            end
        end
        
        tot_time(i+1)=tot_time(i)+sect_time(i);
        
    end
    
    
    corner_times=tot_time;
    
    x=[];   y=[];   z=[];
    vx=[];  vy=[];  vz=[];
    ax=[];  ay=[];  az=[];
    
    vin=[0 0 0];
    vfin=[0 0 0];%0.15*average_velocity*[1 1 1];    
    
    for i=1:tot_sectors %+1 not required
        
        if i<tot_sectors-2%corners(i+1,:)~=corners(end,:)
            v_comp=(corners(i+2,:)-corners(i+1,:))./norm(corners(i+2,:)-corners(i+1,:));
        else
            v_comp=[0 0 0];
        end
        
        vfin=vfin.*v_comp;
        
        coeffx=quintic_trajectory(corner_times(i),corner_times(i+1),corners(i,1),corners(i+1,1),vin(1),vfin(1));
        x=[x;coeffx'];  vx=[vx;[coeffx(2),2*coeffx(3),3*coeffx(4),4*coeffx(5),5*coeffx(6)]];
        ax=[ax;[2*coeffx(3),6*coeffx(4),12*coeffx(5),20*coeffx(6)]];
        coeffy=quintic_trajectory(corner_times(i),corner_times(i+1),corners(i,2),corners(i+1,2),vin(2),vfin(2));
        y=[y;coeffy'];  vy=[vy;[coeffy(2),2*coeffy(3),3*coeffy(4),4*coeffy(5),5*coeffy(6)]];
        ay=[ay;[2*coeffy(3),6*coeffy(4),12*coeffy(5),20*coeffy(6)]];
        coeffz=quintic_trajectory(corner_times(i),corner_times(i+1),corners(i,3),corners(i+1,3),vin(3),vfin(3));
        z=[z;coeffz'];  vz=[vz;[coeffz(2),2*coeffz(3),3*coeffz(4),4*coeffz(5),5*coeffz(6)]];
        az=[az;[2*coeffz(3),6*coeffz(4),12*coeffz(5),20*coeffz(6)]];
        
        vin=vfin;
    
    end
    
    desired_state={};
    
else

    if nargin==2
        
        pos=zeros(3,1); vel=zeros(3,1); acc=zeros(3,1);
        
        for i=1:size(corner_times,1)-1

            if round(corner_times(i),2)<=round(t,2) && round(corner_times(i+1),2)>=round(t,2)
                pos=[x(i,:);y(i,:);z(i,:)]*[1;t;t^2;t^3;t^4;t^5];
                vel=[vx(i,:);vy(i,:);vz(i,:)]*[1;t;t^2;t^3;t^4];
                acc=[ax(i,:);ay(i,:);az(i,:)]*[1;t;t^2;t^3];
                break;
            end
        end
        
        if round(t,2)>=round(corner_times(end),2)
            pos=path0(end,:);
            vel=[0,0,0];
            acc=[0,0,0];
        end
        
        desired_state.pos=pos;
        desired_state.vel=vel;
        desired_state.acc=acc;
        desired_state.yaw=0;
        desired_state.yawdot=0;
    end
    
    
end

end