% Y=[0.412	8.244	22.755	34.359	44.923	54.512];
% Z=[44.6327	36.2011	33.358	29.9769	29.1802	28.6168];
% X= [45	500	1000	1500	2000	2500];
												
% Y=[0.155	9.879	22.4	32.587	42.352	52.856];
% Z=[65.2972	39.017	35.017	30.1286	30.0403	29.8303];
% X=[21	500	1000	1500	2000	2500];
							
% Y=[0.2169	8.645	20.669	31.012	41.598	51.622];
% Z=[44.3828	35.7302	28.7252	28.7252	28.5322	28.5322];
% X=[22	500	1000	1500	2000	2500];
							
% Y=[0.133	9.995	24.373	35.261	45.941	56.123];
% Z=[49.0596	34.0102	32.5944	29.8732	28.9344	28.9344];
% X=[16	500	1000	1500	2000	2500];
							
Y=[0.241	9.231	22.607	34.684	46.442	57.011];							
Z=[59.9522	35.0235	31.8099	31.4278	29.2564	28.4792];
X=[36	500	1000	1500	2000	2500];


figure(1)
plot(X,Y,'LineWidth',2); hold on
xl = xlabel('$Nodes$','Interpreter','LaTeX');
yl = ylabel('$Time$','Interpreter','LaTeX');
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');
figure(2)
plot(X,Z,'LineWidth',2); hold on
xl = xlabel('$Nodes$','Interpreter','LaTeX');
yl = ylabel('$Cost$','Interpreter','LaTeX');
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');
