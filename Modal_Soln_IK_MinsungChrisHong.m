%Closed Form Modal Solution for Inverse Kinematics
%Minsung Chris Hong. For Kinematics Final Project
% Currently unable to successfully implement the integrals on eqn (3) and
% (4) on the "Obstacle Avoidance Algorithm for Hyper-Redundant
% Manipulators" paper to obtain the x1 and x2 positions of each joint. 

clear; clc;
format long

Bessels = [];
% iterate upto the first local min of J0
X = 0:0.0001:3.832;
iterations = length(X);

% finding all the 0th-order of Bessel function of the first kind:
for m = 1:iterations
    Bessels(m) = besselj(0,X(m));
end

% the input coordinates
x_ee = 0.3433;
y_ee = 0.2733;

% the 2nd input coordinate
%x_ee = -0.2933;
%y_ee = 0.6133;

a2 = atan2(x_ee, y_ee);

ee1_x = x_ee/sin(a2);
ee1_y = y_ee/cos(a2);

for j = 1:1:length(Bessels)
    Inv1_x(j) = abs(ee1_x-Bessels(j));
    Inv1_y(j) = abs(ee1_y-Bessels(j));
end

% finding the index that corresponds to the correct restricted inverse
% Bessel function of zero order that gives a value as close to zero for the 
% absolute value of [ x1(1)/sin(a2) - J0(sqrt(a1^2+a2^2)) ].
Inverse_ind = find(Inv1_x == min(Inv1_x));
Inverse_Bessel_x = X(Inverse_ind);
% finding the a1 modal participation function!
a1 = sqrt(Inverse_Bessel_x^2-a2^2);

%%
%hardcoding a1 participation factor:
a = [a1; a2];
S = 0:0.00001:1;
s_iterations = length(S);

for k = 1:s_iterations
    phi(1,k) = 2*pi*cos(2*pi*S(k));
    phi(2,k) = 2*pi*sin(2*pi*S(k));
end

% for every increments along S calculating the curvature function
for r = 1:s_iterations
   Curv(r) = curvature(a,phi(:,r))/(2*pi);
   scaling_fact(r) = a(1)*phi(1,r);
end

for o = 1:1:s_iterations
    j_x(o) = sin(sum(Curv(1:o))/s_iterations);
    j_y(o) = cos(sum(Curv(1:o))/s_iterations);
end

for s = 1:1:s_iterations
   X1(s) = sum(j_x(1:s))/s_iterations;
   X2(s) = sum(j_y(1:s))/s_iterations;
end

X1 = X1-X1(1);
X2 = X2-X2(1);

plot(X1,X2)
xlabel('x')
ylabel('y')
title('Two-Mode Inverse Kinematics')
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Utility function(s)

function Y = curvature(a, phi)
y(1) = a(1)*phi(1);
y(2) = a(2)*phi(2);
Y = y(1)+y(2);
end