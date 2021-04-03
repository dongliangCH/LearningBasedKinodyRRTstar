% plot trajectory from path and NN controller
function plotTraj(path)
for i=1:size(path,1)-1
    
    z0=[path(i,1:2)-path(i+1,1:2),path(i,3:4)]';
    zf=path(i+1,3:4)'; 
    Tf=NNtf10100v1([z0;zf]);
    dt = 0.02;
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
    
    traj=[z1(1:2,:)+path(i+1,1:2)'*ones(1,size(z1,2));z1(3:4,:)];
%     figure(1)
    plot(traj(1,1:size(t,2)),traj(2,1:size(t,2)),'color','r','LineWidth',2);hold on
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

% function dz=dynamics(z,u)
%     dz=[z(3);z(4);u(1);u(2)];
% end