function path = findMinimumPath(tree,end_node,dim)

    % find nodes that connect to end_node
%     connectingNodes = [];
%     for i=1:size(tree,1)
%         if tree(i,2*dim+1)==1
%             tree(i,2*dim+2)=tree(i,2*dim+2)+segment_cost(tree(i,:),end_node(1:2*dim),dim);
%             connectingNodes = [connectingNodes ; tree(i,:)];
%         end
%     end
    idx = tree(:,2*dim+1)==1;
    connectingNodes = tree(idx,:);
    
    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,2*dim+2));
    tmp

    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,2*dim+3);
    while parent_node>=1
        path = [tree(parent_node,:); path];
        parent_node = tree(parent_node,2*dim+3);       
    end

end