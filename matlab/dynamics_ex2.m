function [ dqdt ] = dynamics_ex2( t, q, u )
%DYNAMICS_EX2 Summary of this function goes here
%   Detailed explanation goes here

g = 9.81;
v = q(3:4);
vmag = sqrt(dot(v,v));
d = [v(2);-v(1)]./vmag;

dqdt = zeros(size(q));
dqdt(1) = q(3);
dqdt(2) = q(4);
dqdt(3) = d(1)*u;
dqdt(4) = d(2)*u - g;


end

