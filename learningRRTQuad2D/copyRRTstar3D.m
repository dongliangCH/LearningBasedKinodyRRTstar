

function [its,sizePath,run_time] = copyRRTstar3D(dim,segmentLength,radius,random_world,show_output,samples)

% planning in state space
time=[];

if dim == 2   
    % x,y,vx,vy
    start_cord = [2,2,0,0];    
    goal_cord = [18,18,0,0];
else
    start_cord = [2,18,2,0,0,0];
    goal_cord = [18,2,18,0,0,0];
end


% create random world
Size = 20;
NumObstacles = 10;

if random_world == 1
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
    [world,~] = createKnownWorld(ones(1,dim)*Size,zeros(1,dim),dim);
end

% randomly select start and end nodes
%start_node = generateRandomNode(world,dim)
%end_node   = generateRandomNode(world,dim)
% Each Row Contains States, ConnectToEnd flag, Cost, ParentNodeIdx, and ChildNum,
start_node = [start_cord,0,0,0,0];                
end_node = [goal_cord,0,0,0,0];

% establish tree starting with the start node
tree = start_node;
GChild  = [];
coder.varsize('GChild')

a = clock;

% check to see if start_node connects directly to end_node
if ( sqrt(sum((start_node(1:dim)-end_node(1:dim)).*(start_node(1:dim)-end_node(1:dim))))<segmentLength ...
    &&(collision(start_node,end_node,world,dim)==0) )
  tree(1,2*dim+1)=1;  % path = [start_node; end_node];
else
  if samples >0
    its = 0;
    numPaths = 0;
    FirstSol = 0;
    for i = 1:samples
      [tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,radius,world,0,dim);
      numPaths = numPaths + flag;
      its = its+1;
      
      if numPaths==1 && FirstSol==0
        its
        FirstSol=1;     
        tree_small = tree;
        GChild_small = GChild; 
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end       
      if its==500  
        tree500= tree;
        GChild500 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end  
      if its==1000  
        tree1000= tree;
        GChild1000 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end  
      if its==1500  
        tree1500= tree;
        GChild1500 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end  
      if its==2000  
        tree2000= tree;
        GChild2000 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end 
      if its==2500  
        tree2500= tree;
        GChild2500 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end 
      if its==3000  
        tree3000= tree;
        GChild3000 = GChild;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end 
%       if its==3500  
%         tree3500= tree;
%         GChild3500 = GChild;
%         b = clock;
%         time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
%       end       
    end

  else
    its = 0;
    numPaths = 0;
    while numPaths < 1
      [tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,radius,world,0,dim);
      numPaths = numPaths + flag;
      its = its+1;
    end
  end

end

b = clock;
run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))

if show_output == 1
% figure;
path_first = findMinimumPath(tree_small,end_node,dim);
% plotExpandedTree(tree_small,GChild_small,1,dim);
% plotWorld(world,path_first,dim);
% plotTraj(path_first);
% figure;
path_500 = findMinimumPath(tree500,end_node,dim);
% plotExpandedTree(tree500,GChild500,1,dim);
% plotWorld(world,path_500,dim);
% plotTraj(path_500);
% figure;  
path_1000 = findMinimumPath(tree1000,end_node,dim);
% plotExpandedTree(tree1000,GChild1000,1,dim);
% plotWorld(world,path_1000,dim); 
% plotTraj(path_1000);
% figure;  
path_1500 = findMinimumPath(tree1500,end_node,dim);
% plotExpandedTree(tree1500,GChild1500,1,dim);
% plotWorld(world,path_1500,dim); 
% plotTraj(path_1500);
% figure;  
path_2000 = findMinimumPath(tree2000,end_node,dim);
% plotExpandedTree(tree2000,GChild2000,1,dim);
% plotWorld(world,path_2000,dim); 
% plotTraj(path_2000);
% figure;  
path_2500 = findMinimumPath(tree2500,end_node,dim);
% plotExpandedTree(tree2500,GChild2500,1,dim);
% plotWorld(world,path_2500,dim); 
% plotTraj(path_2500);
% figure;  
path_3000 = findMinimumPath(tree3000,end_node,dim);
% plotExpandedTree(tree3000,GChild3000,1,dim);
% plotWorld(world,path_3000,dim); 
% plotTraj(path_3000);
% figure;  
% path_3500 = findMinimumPath(tree3500,end_node,dim);
% plotExpandedTree(tree3500,GChild3500,1,dim);
% plotWorld(world,path_3500,dim); 
% plotTraj(path_3500);

% path = findMinimumPath(tree,end_node,dim);
% sizePath = size(path,1);
% figure;
% plotExpandedTree(tree,GChild,1,dim);
% plotWorld(world,path,dim);
% plotTraj(path);

end
end



% segmentLength: maximum stepsize,  r: neighbor radius
function [new_tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,r,world,flag_chk,dim)   

  flag1 = 0;
  while flag1==0
    % select a random point
    if rand>0.05
       randomPoint = ones(1,2*dim);
       for i=1:dim
          randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
       end
       for i=1:dim
          % vmin+(vmax-vmin)*rand;
          randomPoint(1,dim+i) = -0.95+(0.95-(-0.95))*rand;                
       end
    else
       randomPoint=end_node(1:2*dim);
    end

    % find node that is closest to randomPoint (Eucl. dist. between positions). 
    tmp = tree(:,1:dim)-randomPoint(1:2);
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [min_dist,idx] = min(sqrd_dist);
    min_parent_idx = idx;
    
    % ensure new_point is within the range of min_parent_idx in terms of NN controller and approximator (Eucl. dist. between positions).
    if min_dist>segmentLength
        % generate a new point that is segmentLength away from tree(idx,1:dim) and closest to randomPoint
        new_point = (randomPoint(1:2)-tree(idx,1:dim));
        new_point = tree(idx,1:dim)+(new_point/sqrt(sum(new_point.*new_point)))*segmentLength;
        new_point = [new_point(1),new_point(2),randomPoint(3:4)];
    else
        new_point=randomPoint;
    end
    
  % check if the new_point is in collision
  if collision_point(new_point, world, dim)==0
      [collision_flag, trajend]=collision(tree(idx,:), new_point, world, dim);  % this collision checking includes a steering function and forward simulation
                                                                                % it will return the endpoint of steering, the point the system actually arrive                                                               
    if collision_flag==0                                                        % && sum((trajend(1:2)-new_point(1:2)).*(trajend(1:2)-new_point(1:2)))<0.1
%      if sum((trajend(1:2)-new_point(1:dim)).*(trajend(1:2)-new_point(1:dim)))<0.12
      new_point = trajend;                                                      % the point the system actually arrive
      new_point_temp = trajend;
      min_cost  = cost_np(tree(idx,:),new_point,dim);                           % total cost from root to new_point through its parent tree(idx,:)
      new_node  = [new_point, 0, min_cost, idx, 0];                             % new node candidate

      tmp_dist = tree(:,1:dim)-new_point(1:2);
      dist = sqr_eucl_dist(tmp_dist,dim);
         
      % find near neighbors 
      gamma   = 40; nun = size(tree,1);
      ner     = gamma*( log(nun+1)/nun )^(1/(dim));
      r1=min(ner,r);
      near_idx = find(dist <= r1^2);
      
      % choose parent node
      if size(near_idx,1)>1
        size_near = size(near_idx,1);      
        for i = 1:size_near                                                
            cost_near = tree(near_idx(i),2*dim+2)+segment_cost(tree(near_idx(i),:),new_point,dim);
            if  cost_near+0.01 < min_cost
                [collision_flag, trajend]=collision(tree(near_idx(i),:), new_node, world, dim);
                if collision_flag==0 % && abs(trajend(3))<1.05 && abs(trajend(4))<1.05 
                    if sum((trajend(1:2)-new_node(1:dim)).*(trajend(1:2)-new_node(1:dim)))<0.12
                        min_cost = cost_near;
                        min_parent_idx = near_idx(i);
                        new_point_temp=trajend;
                    end                
                end
            end
        end        
      end
      new_point=new_point_temp;
      new_node = [new_point, 0 , min_cost, min_parent_idx, 0];
      new_tree = [tree; new_node];
      new_node_idx = size(new_tree,1);
      % ChildNum + 1
      new_tree(min_parent_idx,2*dim+4) = new_tree(min_parent_idx,2*dim+4) + 1;      
      % update GChild matrix
      GChild( min_parent_idx, new_tree(min_parent_idx, 2*dim+4) )  =  new_node_idx ;       
      
      % rewire
      if size(near_idx,1)>1                                                
        reduced_idx = near_idx;
        for j = 1:size(reduced_idx,1)
            near_cost = new_tree(reduced_idx(j),2*dim+2);
            lcost = segment_cost(new_point,new_tree(reduced_idx(j),:),dim);
            rnewcost = min_cost + lcost;
            if near_cost > rnewcost+0.01
                [collision_flag, trajend]=collision(new_node,new_tree(reduced_idx(j),:),world,dim);
                if collision_flag==0 % && abs(trajend(3))<1.05 && abs(trajend(4))<1.05 
                    if sum((trajend(1:2)-new_tree(reduced_idx(j),1:dim)).*(trajend(1:2)-new_tree(reduced_idx(j),1:dim)))<0.12
           
                        ecost = rnewcost - near_cost; 
                        % parent of reduced_idx(j) before rewire, change its child list.
                        GChild( new_tree(reduced_idx(j),2*dim+3), GChild( new_tree(reduced_idx(j),2*dim+3),: ) == reduced_idx(j) ) = -1;
%                 GChild( new_tree(reduced_idx(j),2*dim+3), find( GChild( new_tree(reduced_idx(j),2*dim+3),: ) == reduced_idx(j) ) ) = -1;  
                        % update the cost and parent information of the node being rewired.
                        new_tree(reduced_idx(j),2*dim+2) = rnewcost;           
                        new_tree(reduced_idx(j),2*dim+3) = new_node_idx;
                        % add the node being rewired to the child list of the new added node, new_node_idx.
                        new_tree(new_node_idx, 2*dim+4) = new_tree(new_node_idx, 2*dim+4) + 1;         
                        GChild( new_node_idx, new_tree(new_node_idx, 2*dim+4) )  =  reduced_idx(j) ;
                        % update all cost of the descendant of the node being rewired
                        brunchCost(new_tree, GChild, reduced_idx(j), ecost, dim);       
                    end
                end
            end
        end
      end
      flag1=1;
%      end
    end
    
  end
    
  end

  flag = 0;
  if flag_chk == 0
    % check to see if new node connects directly to end_node
    if  norm(new_node(1:dim)-end_node(1:dim))<0.2
        flag = 1;
        % mark node as connecting to end.
        new_tree(end,2*dim+1)=1;  
    elseif  sqrt(sum((new_node(1:dim)-end_node(1:dim)).*(new_node(1:dim)-end_node(1:dim))))<segmentLength 
        [collision_flag, ~]=collision(new_node,end_node,world,dim);
        
        if  collision_flag==0
            flag = 1;
            % mark node as connecting to end.
            new_tree(end,2*dim+1)=1;          
        end
    end
  end
end

% update all cost of the descendant of the node being rewired, id_candidate
function brunchCost(new_tree, GChild, id_candidate, ecost, dim)

   for i=1:new_tree(id_candidate,2*dim+4) 
        if GChild(id_candidate,i) ~= -1
            new_tree(GChild(id_candidate,i),2*dim+2) = new_tree(GChild(id_candidate,i),2*dim+2) + ecost;
            brunchCost(new_tree, GChild, GChild(id_candidate,i), ecost, dim);
        end        
   end

end




