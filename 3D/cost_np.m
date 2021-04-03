%calculate the cost from root to to_point through parent from_node
function [cost] = cost_np(from_node,to_point,dim)

if dim == 2

    z0=[from_node(1:dim)-to_point(1:dim),from_node(dim+1:2*dim)]';
    zf=to_point(dim+1:2*dim)'; 
    cost=NNval50v1([z0;zf]);   % NNval40v1
    cost = from_node(:,2*dim+2) + cost;
    
else
    
    z0=[from_node(1:dim)-to_point(1:dim),from_node(dim+1:2*dim)]';
    zf=to_point(dim+1:2*dim)'; 
    cost=Val2020([z0;zf]);
    if cost<0.1
        cost=100;
    end
    cost = from_node(:,2*dim+2) + cost;
    
end

end