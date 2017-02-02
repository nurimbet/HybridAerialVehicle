function dqot = FixedWingOde(t,q)

u = [1,1,1];
qdot(1) = q(4) * cos(q(5));
qdot(2) = q(4) * sin(q(5));
qdot(3) = q(4) * u(2);

qdot(4) = u(1);
qdot(5) = u(3);