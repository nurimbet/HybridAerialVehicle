q0 = [0 0 0 0 0 0 0]';
tspan = [0 60];
[t,y] = ode45(@(t,y) QuadrotorOde(t,y), tspan, q0);