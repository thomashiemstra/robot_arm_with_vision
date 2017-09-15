scrsz = get(0,'ScreenSize');
fig1 = figure('Position',[scrsz(3)/4 scrsz(4)/5 scrsz(4)/1.5 scrsz(4)/1.5]);

h1=subplot(2,3,1); 
h(1) = scatter(t,x1,'x');
hold on;
h(2) = scatter(t,y1,'o');

h = legend([h(1) h(2)],{'exact solution','NN solution'},'Interpreter','LaTex','fontsize',17);
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
title('$\theta_{1}$','Interpreter','LaTex','fontsize',17)
hold off;

h2=subplot(2,3,2); 
h(1) = scatter(t,x2,'x');
hold on;
h(2) = scatter(t,y2,'o');

title('$\theta_{2}$','Interpreter','LaTex','fontsize',17)
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
hold off;

h3=subplot(2,3,3); 
h(1) = scatter(t,x3,'x');
hold on;
h(2) = scatter(t,y3,'o');

title('$\theta_{3}$','Interpreter','LaTex','fontsize',17)
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
hold off;

h4=subplot(2,3,4); 
h(1) = scatter(t,x4,'x');
hold on;
h(2) = scatter(t,y4,'o');

title('$\theta_{4}$','Interpreter','LaTex','fontsize',17)
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
hold off;

h5=subplot(2,3,5); 
h(1) = scatter(t,x5,'x');
hold on;
h(2) = scatter(t,y5,'o');

title('$\theta_{5}$','Interpreter','LaTex','fontsize',17)
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
hold off;

h6=subplot(2,3,6); 
h(1) = scatter(t,x6,'x');
hold on;
h(2) = scatter(t,y6,'o');

title('$\theta_{6}$','Interpreter','LaTex','fontsize',17)
xlabel('$t$','Interpreter','LaTex','fontsize',17);
ylabel('$\theta$','Interpreter','LaTex','fontsize',17)
hold off;




set(h1,'Position',[0.10 0.70 0.325 0.225]);
set(h2,'Position',[0.54 0.70 0.325 0.225]);
set(h3,'Position',[0.10 0.37 0.325 0.225]);
set(h4,'Position',[0.54 0.37 0.325 0.225]);
set(h5,'Position',[0.10 0.05 0.325 0.225]);
set(h6,'Position',[0.54 0.05 0.325 0.225]);



