% check if a point is in collision
function collision_flag = collision_point(p, world, dim)

collision_flag = 0;

% for i=1:dim
%    if (p(i)>world.endcorner(i))||(p(i)<world.origincorner(i))
%        collision_flag = 1;
%    end
% end

      % check each obstacle
      for i=1:world.NumObstacles
          dist=[p(1);p(2)]-[world.cx(i); world.cy(i)];
        if sum(dist.*dist)<(world.radius(i)+0.1)^2
            collision_flag = 1;
            break;
        end
      end
end