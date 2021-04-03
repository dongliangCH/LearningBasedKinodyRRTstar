function [cost] = segment_cost(from_node,to_point,dim)

    z0=[from_node(1:2)-to_point(1:2),from_node(3:4)]';
    zf=to_point(3:4)'; 
    cost=Val1010([z0;zf]);   % NNval40new  NNval20new  Val1010  Val20  Val40 
    if cost<0.01
       cost = max(NNval40new([z0;zf]),Val20([z0;zf]));
    end
    if cost<0.01
        cost=100;
    end
    
end