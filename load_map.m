function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

%% READ DATA %%

%Srihari Menon note: make sure there are 10 elements in each row of the
%file. Format: ENTITY XMIN XMAX YMIN YMAX ZMIN ZMAX R G B

fid=fopen(filename,'r');
data=textscan(fid,'%s %f %f %f %f %f %f %f %f %f','commentStyle','#');

if(isempty([data{1}]))
    map=[];
    return;
end

if margin<0.3%sqrt(0.25^2+0.5^2)   %establishing minimum margin
    margin=0.3;%sqrt(0.25^2+0.5^2);
end

entity=data{1};

xmin=data{2};   ymin=data{3};   zmin=data{4};   
xmax=data{5};   ymax=data{6};   zmax=data{7};

R=data{8};  G=data{9};  B=data{10};

totentity=numel(entity);

boundind=find(ismember(entity,'boundary'));

map=zeros(totentity+1,6);
map(1,:)=[xmin(boundind),ymin(boundind),zmin(boundind),xmax(boundind),ymax(boundind),zmax(boundind)];

xmin(boundind)=[];  ymin(boundind)=[];  zmin(boundind)=[];
xmax(boundind)=[];  ymax(boundind)=[];  zmax(boundind)=[];
map(2:end-1,:)=[xmin(:)-margin,ymin(:)-margin,zmin(:)-margin,xmax(:)+margin,ymax(:)+margin,zmax(:)+margin];
map(end,1)=xy_res;  map(end,2)=z_res;   map(end,3)=margin;

 
end