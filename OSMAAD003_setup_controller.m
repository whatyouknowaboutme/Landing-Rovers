%Mechatronics 2 Lander Controller Design
%Author: Aadam Osman (OSMAAD003)
%Date:20 April 2019

clear;
clc;

syms x y theta dx dy dtheta F1 F2;

%mass and gravity
g=9.3;
m=4000;

%from EOM code
ddx = -(sin(theta)*(F1+F2))/4000;
ddy =  (F1+F2)*(cos(theta))/4000 - 93/10;
ddtheta = (9*(F2-F1))/163480;

%non-linear state space model
f_x_u = [dx;ddx;dy;ddy;dtheta;ddtheta];

%linearize non-linear system
A = jacobian(f_x_u,[x;dx;y;dy;theta;dtheta]);
B= jacobian(f_x_u,[F1;F2]);

%assign intial condition theta = 0, F1 = mg/2 and F2=mg/2
lin_force = m*g/2;
A_lin = double(subs(A,[theta F1 F2],[0 lin_force lin_force]));
B_lin = double(subs(B,[theta F1 F2],[0 lin_force lin_force]));

%definition of cost matrices for controller 1
Q= zeros(6,6);
R =zeros(2,2);

%Assigning relevant cost weightings
Q(1,1) = 9.5E9;%x  
Q(2,2) = 7.5E9;%dx
Q(3,3) = 1.9E5;%y
Q(4,4) = 1.71E6;%dy
Q(5,5) = 1E5;%th
Q(6,6) = 1E13;%dth

R(1,1) = 1; %F1 
R(2,2) = R(1,1); %F2

%controller design via LQR
K = lqr (A_lin,B_lin,Q,R);


%controller 2
%non-linear state space model
f_x_u = [dtheta;ddtheta];

%linearise system about operating point theta = 0, F1 = 0 and F2 =0
A = jacobian(f_x_u,[theta;dtheta]);
B= jacobian(f_x_u,[F1;F2]);

A_lin = double(subs(A,[theta F1 F2],[0 0 0]));
B_lin = double(subs(B,[theta F1 F2],[0 0 0]));

%defining cost matrices
Q = zeros(2,2);
R= zeros(2,2);

%assigning the associated weightings
Q(1,1) = 1E10;%theta
Q(2,2) = 1E7;%dtheta

R(1,1) = 1000;%F1
R(2,2) = R(1,1);%F2

K_180 = lqr(A_lin,B_lin,Q,R);






