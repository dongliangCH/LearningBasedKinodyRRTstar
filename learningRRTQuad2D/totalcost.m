cost=0;
for i=1:size(path,1)-1
    
    x=path(i,1)-path(i+1,1); y=path(i,2)-path(i+1,2); vx0=path(i,3); vy0=path(i,4); vxf=path(i+1,3); vyf=path(i+1,4);
    segcost=NNval40new([x; y; vx0; vy0; vxf; vyf]);   % NNval20new
    cost=cost+segcost
    
end