% plot trajectory from path and NN controller
function plotTraj(path)
for i=1:size(path,1)-1
    
    pa.g = 9.81; % (m/s^2) gravity
    pa.d = 0.3;  % (m) half-width of quad rotor
    pa.m = 0.2;  % (m) half-mass of the quad rotor
    
    x=path(i,1)-path(i+1,1); y=path(i,2)-path(i+1,2); vx0=path(i,3); vy0=path(i,4); vxf=path(i+1,3); vyf=path(i+1,4);
    
    Tf = NNtf2050([x; y; 0; vx0; vy0; 0; vxf; vyf]);
    dt = 0.01;
    Horizon = Tf/dt;
    t = linspace(0,Tf,Horizon);
    
    x0=[x;y;0;vx0;vy0;0]; 
    for k=1:1:length(t)-1
        
            u=NN204020([x0(:,k);vxf;vyf]); 
            dx=dynamics(x0(:,k),u,pa);
            x0(:,k+1)=x0(:,k)+dx*dt;
    
    end
    
    traj=[x0(1:2,:)+path(i+1,1:2)'*ones(1,size(x0,2));x0(3:4,:)];
%     figure(1)
    plot(traj(1,1:size(t,2)),traj(2,1:size(t,2)),'color',[0.8500 0.3250 0.0980],'LineWidth',2);hold on
    xl = xlabel('$x (m)$','Interpreter','LaTeX');
    yl = ylabel('$y (m)$','Interpreter','LaTeX');
    set(xl,'FontSize',18);
    set(yl,'FontSize',18);
    set(gca,'FontSize',16,'FontName','Times');
%     figure(2)
%     plot(t,traj(1,1:size(t,2)),'LineWidth',2);hold on
%     plot(t,traj(2,1:size(t,2)),'LineWidth',2);
%     figure(3)
%     plot(t,traj(3,1:size(t,2)),'LineWidth',2);hold on
%     plot(t,traj(4,1:size(t,2)),'LineWidth',2);hold on
    
%     plot(traj(1,end),traj(2,end),'o','LineWidth',2,'MarkerSize',8);hold on
%     plot(path(i+1,1),path(i+1,2),'o','LineWidth',2,'MarkerSize',8);hold on
       
end
end