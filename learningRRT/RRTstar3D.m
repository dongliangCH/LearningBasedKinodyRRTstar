

function [its,sizePath,run_time] =  RRTstar3D(dim,segmentLength,radius,random_world,show_output,samples)

% planning in state space
time=[]; firstSol=1;

if dim == 2   
    start_cord = [5,5,0,0];    %  x,y,vx,vy
    goal_cord = [35,35,0,0];
else
    start_cord = [5,5,5,0,0,0];
    goal_cord = [95,95,95,0,0,0];
end


% create random world
Size = 40;
NumObstacles = 10;

if random_world == 1
world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
[world,~] = createKnownWorld(ones(1,dim)*Size,zeros(1,dim),dim);
end

% randomly select start and end nodes
%start_node = generateRandomNode(world,dim)
%end_node   = generateRandomNode(world,dim)
start_node = [start_cord,0,0,0,0];                %  Each Row Contains States, ConnectToEnd flag, Costz, ParentNodeIdx, and ChildNum,
end_node = [goal_cord,0,0,0,0];

% establish tree starting with the start node
tree = start_node;

GChild  = [];
coder.varsize('GChild')

a = clock;

% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:dim)-end_node(1:dim))<segmentLength )...
    &&(collision(start_node,end_node,world,dim)==0) )
  path = [start_node; end_node];
else

  if samples >0
  its = 0;
  numPaths = 0;
  for i = 1:samples
      [tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,radius,world,0,dim);
      numPaths = numPaths + flag;
      its = its+1;

      if numPaths==1 && firstSol==1
        firstSol=0;
        tree_small = tree;
        its
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==500
        tree_500 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==1000
        tree_1000 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==1500
        tree_1500 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==2000
        tree_2000 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==2500
        tree_2500 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==3000
        tree_3000 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
      if its==3500
        tree_3500 = tree;
        b = clock;
        time = [time, 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))];
      end
  end

  else
  its = 0;
  numPaths = 0;
  while numPaths < 1
      [tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,radius,world,0,dim);
      numPaths = numPaths + flag;
      its = its+1;
  end
  its
  end

end

% find path with minimum cost to end_node
b = clock;
run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6))

if show_output == 1
figure;
path_first = findMinimumPath(tree_small,end_node,dim);
plotExpandedTree(world,tree_small,dim);
plotWorld(world,path_first,dim);
plotTraj(path_first);
figure;
path_500 = findMinimumPath(tree_500,end_node,dim);
plotExpandedTree(world,tree_500,dim);
plotWorld(world,path_500,dim);
plotTraj(path_500);
% figure;
% path_1000 = findMinimumPath(tree_1000,end_node,dim);
% plotExpandedTree(world,tree_1000,dim);
% plotWorld(world,path_1000,dim);
% plotTraj(path_1000);
% figure;
% path_1500 = findMinimumPath(tree_1500,end_node,dim);
% plotExpandedTree(world,tree_1500,dim);
% plotWorld(world,path_1500,dim);
% plotTraj(path_1500);
% figure;
% path_2000 = findMinimumPath(tree_2000,end_node,dim);
% plotExpandedTree(world,tree_2000,dim);
% plotWorld(world,path_2000,dim);
% plotTraj(path_2000);
% figure;
% path_2500 = findMinimumPath(tree_2500,end_node,dim);
% plotExpandedTree(world,tree_2500,dim);
% plotWorld(world,path_2500,dim);
% plotTraj(path_2500);
% figure;
% path_3000 = findMinimumPath(tree_3000,end_node,dim);
% plotExpandedTree(world,tree_3000,dim);
% plotWorld(world,path_3000,dim);
% plotTraj(path_3000);
% figure;
% path_3500 = findMinimumPath(tree_3500,end_node,dim);
% plotExpandedTree(world,tree_3500,dim);
% plotWorld(world,path_3500,dim);
% plotTraj(path_3500);

figure;
path = findMinimumPath(tree,end_node,dim);
sizePath = size(path,1);
plotExpandedTree(world,tree,dim);
plotWorld(world,path,dim); hold on
plotTraj(path);

end
end


function [new_tree,GChild,flag] = extendTree(tree,GChild,end_node,segmentLength,r,world,flag_chk,dim)   % segmentLength: maximum stepsize,  r: neighbor radius

  flag1 = 0;
  while flag1==0
    % select a random point
    if rand>0.05
       randomPoint = zeros(1,2*dim);
       for i=1:dim
          randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
       end
    else
       randomPoint=end_node(1:2*dim);
    end

    % find node that is closest to randomPoint (Eucl. dist. between positions). 
    tmp = tree(:,1:dim)-randomPoint(1:2);
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [min_dist,idx] = min(sqrd_dist);
    min_parent_idx = idx;
    
    Vect=randomPoint(1:2)-tree(idx,1:dim);
    Vect=Vect/norm(Vect);
%     new_point(3:4)=rand*Vect;
    new_point(3:4)=[-1+(1-(-1))*rand, -1+(1-(-1))*rand];     % vmin=-1 vmax=1
    
    % find new_point that is within the range of min_parent_idx in terms of NN controller and approximator (Eucl. dist. between positions).
    if min_dist>segmentLength^2
        % generate a new point that is closest to randomPoint, segmentLength away from tree(idx,1:dim)
        new_point(1:2) = tree(idx,1:dim)+Vect*segmentLength;
    else
        new_point(1:2) = randomPoint(1:2);
    end

  % check if the new_point is in collision
  if collision_point(new_point, world, dim)==0
      [collision_flag, trajend]=collision(tree(idx,:), new_point, world, dim);  % this collision checking includes a steering function and forward simulation
                                                                                % it will return the endpoint of steering, the point the system actually arrive                                                               
    if collision_flag==0  
        
      new_point = trajend;                                                      % the point the system actually arrive
      new_point_temp = trajend;
      min_cost  = cost_np(tree(idx,:),new_point,dim);                           % total cost from root to new_point through its parent tree(idx,:)
      new_node  = [new_point, 0, min_cost, idx, 0];                             % new node candidate

      tmp_dist = tree(:,1:dim)-new_point(1:2);
      dist = sqr_eucl_dist(tmp_dist,dim);
         
      % find near neighbors      
      gamma   = 40; nun = size(tree,1);
      ner     = gamma*( log(nun+1)/nun )^(1/dim);
      r1=min(ner,r);
      near_idx = find(dist <= r1^2);
      
      if size(near_idx,1)>1
      size_near = size(near_idx,1);
      
        for i = 1:size_near                                                % choose parent node
            cost_near = tree(near_idx(i),2*dim+2)+segment_cost(tree(near_idx(i),:),new_point,dim);
            if  cost_near+0.02 < min_cost
                [collision_flag, trajend]=collision(tree(near_idx(i),:), new_node, world, dim);
                if collision_flag==0
                    if sum((trajend(1:2)-new_node(1:dim)).*(trajend(1:2)-new_node(1:dim)))<0.04                 % norm(trajend-new_node(1:2*dim))<0.5        
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
      
      new_tree(min_parent_idx,2*dim+4) = new_tree(min_parent_idx,2*dim+4) + 1;         % ChildNum + 1
      GChild( min_parent_idx, new_tree(min_parent_idx, 2*dim+4) )  =  new_node_idx ;       % update GChild matrix
      

      if size(near_idx,1)>1                                                % rewire
      reduced_idx = near_idx;
        for j = 1:size(reduced_idx,1)
          near_cost = new_tree(reduced_idx(j),2*dim+2);
          lcost = segment_cost(new_point,new_tree(reduced_idx(j),:),dim);
          rnewcost = min_cost + lcost;
            if near_cost > rnewcost
                    [collision_flag, trajend]=collision(new_node,new_tree(reduced_idx(j),:),world,dim);
                if  collision_flag==0 
                    if sum((trajend(1:2)-new_tree(reduced_idx(j),1:dim)).*(trajend(1:2)-new_tree(reduced_idx(j),1:dim)))<0.04    % norm(trajend-new_tree(reduced_idx(j),1:2*dim))<0.5
           
                ecost = rnewcost - near_cost;                
                GChild( new_tree(reduced_idx(j),2*dim+3), GChild( new_tree(reduced_idx(j),2*dim+3),: ) == reduced_idx(j) ) = -1;      % parent of reduced_idx(j) before rewire, change its child list.
                new_tree(reduced_idx(j),2*dim+2) = rnewcost;           % update the cost and parent information of the node being rewired.
                new_tree(reduced_idx(j),2*dim+3) = new_node_idx;
                                
                new_tree(new_node_idx, 2*dim+4) = new_tree(new_node_idx, 2*dim+4) + 1;         % add the node being rewired to the child list of the new added node, new_node_idx.
                GChild( new_node_idx, new_tree(new_node_idx, 2*dim+4) )  =  reduced_idx(j) ;
                
                brunchCost(new_tree, GChild, reduced_idx(j), ecost, dim);       % update all cost of the descendant of the node being rewired
                    end
                end
            end

        end
      end
      flag1=1;
    end
    
  end
    
  end


  if flag_chk == 0
    % check to see if new node connects directly to end_node
    if  norm(new_node(1:dim)-end_node(1:dim))<segmentLength 
        [collision_flag, ~]=collision(new_node,end_node,world,dim);
        
        if  collision_flag==0
            flag = 1;
            new_tree(end,2*dim+1)=1;  % mark node as connecting to end.
        else
            flag = 0;
        end
        
    else
    flag = 0;
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




