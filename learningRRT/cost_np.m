%calculate the cost from root to to_point through parent from_node
function [cost] = cost_np(from_node,to_point,dim)

    z0=[from_node(1:2)-to_point(1:2),from_node(3:4)]';
    zf=to_point(3:4)'; 
    cost=NNval50v1([z0;zf]);   % NNval40v1
    cost = from_node(:,2*dim+2) + cost;

end