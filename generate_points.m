function P=generate_points(start,goal,res)

x=[];
if round(start(1),2)~=round(goal(1),2)
    x = start(1):(goal(1)-start(1))*res/norm(goal-start):goal(1);    x=x';
end

y=[];
if round(start(2),2)~=round(goal(2),2)
    y = start(2):(goal(2)-start(2))*res/norm(goal-start):goal(2);    y=y';    
end

z=[];
if round(start(3),2)~=round(goal(3),2)
    z = start(3):(goal(3)-start(3))*res/norm(goal-start):goal(3);    z=z';
end

s(3)=0; s(1)=size(x,1); s(2)=size(y,1); s(3)=size(z,1);

if(isempty(x))
    x=start(1)*ones(max(s),1);
end
if(isempty(y))
    y=start(2)*ones(max(s),1);
end
if(isempty(z))
    z=start(3)*ones(max(s),1);
end

P=[x,y,z];

end