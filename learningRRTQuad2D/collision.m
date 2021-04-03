function [collision_flag, trajend] = collision(parent, node, world, dim)

pa.g = 9.81; % (m/s^2) gravity
pa.d = 0.3;  % (m) half-width of quad rotor
pa.m = 0.2;  % (m) half-mass of the quad rotor

collision_flag = 0;

% for i=1:dim
%    if (node(i)>world.endcorner(i))||(node(i)<world.origincorner(i))
%        collision_flag = 1;
%        trajend = 0;
%    end
% end

if collision_flag == 0 && dim ==2
    
    x=parent(1)-node(1); y=parent(2)-node(2); vx0=parent(3); vy0=parent(4); vxf=node(3); vyf=node(4); 
    Tf = T202020_1([x; y; 0; vx0; vy0; 0; vxf; vyf]);       % T202020_1  T202020  NNtf2050
    dt = 0.025;
    Horizon = Tf/dt;
    t = linspace(0,Tf,Horizon);
    
    x0=[x;y;0;vx0;vy0;0]; 
    for k=1:1:length(t)-1        
        u=NN204020([x0(:,k);vxf;vyf]); 
        dx=dynamics(x0(:,k),u,pa);
        x0(:,k+1)=x0(:,k)+dx*dt;    
    end
    
    trajend=[x0(1:2,end)'+node(1:2),x0(4:5,end)'];
    if abs(x0(3,end))>1.1 || abs(x0(4,end))>1.1 || norm(x0(1:2,end))>0.4        
        collision_flag = 1;        
    else       
        traj=x0(1:2,:)'+ones(size(x0,2),1)*node(1:2);
        check_step=round(size(traj,1)/8);
    
        for j = check_step:check_step:size(traj,1)-check_step
            p = traj(j,:);
            % check each obstacle
            for i=1:world.NumObstacles
                dist=[p(1);p(2)]-[world.cx(i); world.cy(i)];
                if sum(dist.*dist)<world.radius(i)^2
                    collision_flag = 1;
                    return;   %break;
                end
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