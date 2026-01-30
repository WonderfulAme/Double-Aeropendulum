function simulate_aeropendulum(dynamics_fun, thrust_fun, params, tspan, x0)

l1 = params.l1;
l2 = params.l2;
d  = params.d;

%% Solve the model as a simple and ordinary differential equation based on an explicit Runge-Kutta (4,5)
[t, sol] = ode45(@(t,x) dynamics_fun(t,x,params,thrust_fun), tspan, x0);
theta = mod(sol(:,1),2*pi);
theta_dot = sol(:,2);

%% Create an animated figure to see the behavior
figure('Position',[100 100 1200 500])

% Pendulum how it is seen in the human eyes
subplot(1,2,1)
axis equal
axis([-0.6 0.6 -0.6 0.6])
grid on
xlabel('X (m)')
ylabel('Y (m)')
title('Double Rotor Aeropendulum')
hold on

% Color some things
pivot = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','b');
bar = plot([0 0],[0 0],'b-','LineWidth',3);
r1 = plot(0,0,'ro','MarkerSize',10,'MarkerFaceColor','m');
r2 = plot(0,0,'go','MarkerSize',10,'MarkerFaceColor','g');
cg = plot(0,0,'mx','MarkerSize',10,'LineWidth',2);

label_T1 = text(0.95, 0.95, '$T_1$', ...
    'Units','normalized','FontSize',12,...
    'HorizontalAlignment','right','VerticalAlignment','top',...
    'Color','m','Interpreter','latex');

label_T2 = text(0.95, 0.90, '$T_2$', ...
    'Units','normalized','FontSize',12,...
    'HorizontalAlignment','right','VerticalAlignment','top',...
    'Color','g','Interpreter','latex');

% ---- Angle theta plot ----
subplot(1,2,2)
grid on
xlabel('Time (s)')
ylabel('\theta (rad)')
title('Angle Evolution')
hold on
angle_line = animatedline('Color','b','LineWidth',2);
current_point = plot(0,0,'ro','MarkerSize',8);
ylim([0 2*pi])
xlim([tspan(1) tspan(end)])

%% Animation loop
for k = 1:length(t)
    th = theta(k);
    thdot = theta_dot(k);

    % Positions
    x1 =  l1*cos(th);   y1 =  l1*sin(th);
    x2 = -l2*cos(th);   y2 = -l2*sin(th);
    xcg = d*cos(th);    ycg = d*sin(th);

    set(bar,'XData',[x2 x1],'YData',[y2 y1]);
    set(r1,'XData',x1,'YData',y1);
    set(r2,'XData',x2,'YData',y2);
    set(cg,'XData',xcg,'YData',ycg);

    % Thrust arrows
    delete(findall(gca,'Type','quiver'))
    [T1,T2] = thrust_fun(t(k), th, thdot);

    set(label_T1,'String',sprintf('$T_1 = %.2f$',T1));
    set(label_T2,'String',sprintf('$T_2 = %.2f$',T2));

    scale = 0.05;
    perp = th + pi/2;

    quiver(x1,y1,scale*T1*cos(perp),scale*T1*sin(perp),0,'r','LineWidth',1.5);
    quiver(x2,y2,scale*T2*cos(perp),scale*T2*sin(perp),0,'g','LineWidth',1.5);

    addpoints(angle_line,t(k),th);
    set(current_point,'XData',t(k),'YData',th);

    drawnow
    pause(0.02)
end
end
