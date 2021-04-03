function [collision_flag, trajend] = collision(parent, node, world, dim)

collision_flag = 0;

for i=1:dim
   if (node(i)>world.endcorner(i))||(node(i)<world.origincorner(i))
       collision_flag = 1;
       trajend = 0;
   end
end

if collision_flag == 0 && dim ==2
    
    z0=[parent(1:2)-node(1:2),parent(3:4)]';
    zf=node(3:4)'; 
    Tf=NNtf10100v1([z0;zf]);
    dt = 0.04;
    Horizon = Tf/dt;
    t = linspace(0,Tf,Horizon);
    z1(:,1)=z0;
    for k=1:1:length(t)-1
        
            tSpan=[t(k),t(k+1)];
            u=NN202020([z1(:,k);zf]);
            
            z1(:,k+1)=z1(:,k)+[z1(3:4,k);u]*dt;
            
%             dynFun = @(t,z)(  dynamics(z, u)  );
%             soln1 = ode45(dynFun,tSpan,z1(:,k));
%             z1(:,k+1) = deval(soln1,t(k+1));
    
    end
    
    trajend=z1(:,end)'+[node(1:2),0,0];
    traj=z1(1:2,:)'+ones(size(z1,2),1)*node(1:2);
    check_step=round(size(traj,1)/8);
    
    for i = check_step:check_step:size(traj,1)-check_step
    p = traj(i,:);
      % check each obstacle
      for i=1:world.NumObstacles
        if (norm([p(1);p(2)]-[world.cx(i); world.cy(i)])<=1*world.radius(i))
            collision_flag = 1;
            break;
        end
      end
    end

%%%%% dim=3 case has not been implemented yet %%%%%
elseif collision_flag == 0 && dim ==3
    for sigma = 0:.2:1
    p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
      % check each obstacle
      for i=1:world.NumObstacles
        if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i))
            collision_flag = 1;
            break;
        end
      end
    end
end
end


function dz=dynamics(z,u)
    dz=[z(3);z(4);u(1);u(2)];
end