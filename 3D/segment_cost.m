function [cost] = segment_cost(from_node,to_point,dim)

if dim==2

    z0=[from_node(1:2)-to_point(1:2),from_node(3:4)]';
    zf=to_point(3:4)'; 
    cost=NNval50v1([z0;zf]);   % NNval40v1
    
else
    
    z0=[from_node(1:dim)-to_point(1:dim),from_node(dim+1:2*dim)]';
    zf=to_point(dim+1:2*dim)'; 
    cost=Val2020([z0;zf]);
    if cost<0.1
        cost=100;
    end
    
end
    
end