% Modified DH
% ABB robot
clear;
clc;
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = 0; d(2) = 0; a(2) = 0.320; alp(2) = pi/2;   
th(3) = 0; d(3) = 0; a(3) = 0.975; alp(3) = 0;
th(4) = 0; d(4) = 0.887; a(4) = 0.2; alp(4) = pi/2;
th(5) = 0; d(5) = 0; a(5) = 0; alp(5) = -pi/2;
th(6) = 0; d(6) = 0; a(6) = 0; alp(6) = pi/2;
% DH parameters  th     d    a    alpha  sigma
L1 = Link([th(1), d(1), a(1), alp(1), 0], 'modified');
L2 = Link([th(2), d(2), a(2), alp(2), 0], 'modified');
L3 = Link([th(3), d(3), a(3), alp(3), 0], 'modified');
L4 = Link([th(4), d(4), a(4), alp(4), 0], 'modified');
L5 = Link([th(5), d(5), a(5), alp(5), 0], 'modified');  
L6 = Link([th(6), d(6), a(6), alp(6), 0], 'modified');

L1.m = 7.36; L2.m = 36.26; L3.m = 12.44;
L4.m = 1.21; L5.m = 1.21; L6.m = 1;
% L1.r = 
L1.I = [0 0 0; 0 0 0; 0 0 0.6];
L2.I = [0 0 0; 0 0 0; 0 0 0.5];
L3.I = [0 0 0; 0 0 0; 0 0 0.4];
L4.I = [0 0 0; 0 0 0; 0 0 0.3];
L5.I = [0 0 0; 0 0 0; 0 0 0.2];
L6.I = [0 0 0; 0 0 0; 0 0 0.1];

T01 = MDHTrans(alp(1), a(1), d(1), th(1));
T12 = MDHTrans(alp(2), a(2), d(2), th(2));
T23 = MDHTrans(alp(3), a(3), d(3), th(3));
T34 = MDHTrans(alp(4), a(4), d(4), th(4));
T45 = MDHTrans(alp(5), a(5), d(5), th(5));
T56 = MDHTrans(alp(6), a(6), d(6), th(6));
% 各关节p及各link质心pc的距离(假设质心在几何中心)
p10 = T01(1: 3, 4); p21 = T12(1: 3, 4); p32 = T23(1: 3, 4);
p43 = T34(1: 3, 4); p54 = T45(1: 3, 4); p65 = T56(1: 3, 4); p76 = [0, 0, 0]';
pc11 = 0.5 * p21
pc22 = 0.5 * p32
pc33 = 0.5 * p43
pc44 = 0.5 * p54
pc55 = 0.5 * p65 
pc66 = 0.5 * p76
% 连杆质心位置
L1.r = pc11; L2.r = pc22; L3.r = pc33;
L4.r = pc44; L5.r = pc55; L6.r = pc66;

robot = SerialLink([L1, L2, L3, L4, L5, L6]); 
robot.name='ABBRobot-6-dof';
robot.display() 

theta = [-20, 120, -15, 30, 20, 10]*pi/180;
robot.teach();
robot.plot(theta); 
% t = robot.fkine(theta)    %末端执行器位姿
% [~, T] = My_Forward_Kinematics_MDH(theta, d, a, alp)
qd = [6 5 4 3 2 1]; qdd = [6 5 4 3 2 1];
tau = robot.rne(theta, qd, qdd, [0, 0, 9.8])
TAU = myNewtonEuler(theta*180/pi, qd', qdd')