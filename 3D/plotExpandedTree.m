function plotExpandedTree(world,tree,dim)
    ind = size(tree,1);
    while ind>0
    branch = [];
    node = tree(ind,:);
    branch = [ branch ; node ];
    parent_node = node(2*dim+3);
        while parent_node >= 1
        cur_parent = parent_node;
        branch = [branch; tree(parent_node,:)];
        parent_node = tree(parent_node,2*dim+3);
        end
        ind = ind - 1;

        if dim == 2
        X = branch(:,1);
        Y = branch(:,2);

        p = plot(X,Y);
        set(p,'Color','[0 0.4470 0.7410]','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;

        elseif dim == 3
        X = branch(:,1);
        Y = branch(:,2);
        Z = branch(:,3);

        p = plot3(X,Y,Z);
        set(p,'Color','[0 0.4470 0.7410]','LineWidth',0.5,'Marker','.','MarkerEdgeColor','[0 0.4470 0.7410]');
        hold on;
        end
    end
end