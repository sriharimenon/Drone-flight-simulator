function opticorners=optipath(corners,map)

xy_res=map(end,1);  z_res=map(end,2);

res=xy_res;

if xy_res>z_res
    res=z_res;
end

opticorners=corners(1,:);

i=3;

while i<size(corners,1)
    
%     if (i>=size(corners,1))
%         opticorners(end+1,:)=corners(end,:);
%         break;
%     end
    
    p=generate_points(opticorners(end,:),corners(i,:),res);
    
    if(sum(collide(map,p)))
        opticorners(end+1,:)=corners(i-1,:);
        %i=i+1;
    end
    i=i+1;
end

opticorners(end+1,:)=corners(end,:);

end