close all
q0 = [0 0 0 0 0 0 0]';
tspan = [0:0.1:60];
[t,y] = ode45(@(t,y) QuadrotorOde(t,y), tspan, q0);

plot3(y(:,1), y(:,2), y(:,3))
xlabel('x')
ylabel('y')
zlabel('z')