function qdot = QuadrotorOde(t,q)

u = [0.1,1,1,0.1];
qdot(1,1) = q(4) * cos(q(5)) + q(6) * sin(q(5)) * sin(q(7));
qdot(2,1) = q(4) * sin(q(5)) - q(6) * cos(q(5)) * sin(q(7));
qdot(3,1) = q(6) * cos(q(7));

qdot(4,1) = u(2); % vx
qdot(6,1) = u(3); % vz
qdot(5,1) = u(4); % wz
qdot(7,1) = u(1); % wx