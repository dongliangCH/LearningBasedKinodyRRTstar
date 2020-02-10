function world = createWorld(NumObstacles, endcorner, origincorner, dim)

  if dim == 2

    % check to make sure that the region is nonempty
  if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2))
      disp('Not valid corner specifications!')
      world=[];

  % create world data structure
  else
    world.NumObstacles = NumObstacles;
    world.endcorner = endcorner;
    world.origincorner = origincorner;

    % create NumObstacles
    maxRadius = min(endcorner(1)- origincorner(1), endcorner(2)-origincorner(2));
    maxRadius = 5*maxRadius/NumObstacles/2;
    for i=1:NumObstacles,
        % randomly pick radius
        world.radius(i) = maxRadius*rand;
        % randomly pick center of obstacles
        cx = origincorner(1) + world.radius(i)...
            + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
        cy = origincorner(2) + world.radius(i)...
            + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
        world.cx(i) = cx;
        world.cy(i) = cy;
    end
  end

  elseif dim ==3
  % check to make sure that the region is nonempty
  if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
      disp('Not valid corner specifications!')
      world=[];

  % create world data structure
  else
    world.NumObstacles = NumObstacles;
    world.endcorner = endcorner;
    world.origincorner = origincorner;

    % create NumObstacles
    bounds = [endcorner(1)- origincorner(1), endcorner(2)-origincorner(2), endcorner(3)-origincorner(3)];
    maxRadius = min(bounds);
    maxRadius = 5*maxRadius/NumObstacles;
    for i=1:NumObstacles
        % randomly pick radius
        world.radius(i) = maxRadius*rand;
        % randomly pick center of obstacles
        cx = origincorner(1) + world.radius(i)...
            + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
        cy = origincorner(2) + world.radius(i)...
            + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
        cz = origincorner(2) + world.radius(i)...
            + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
        world.cx(i) = cx;
        world.cy(i) = cy;
        world.cz(i) = cz;
    end
  end
  end
end