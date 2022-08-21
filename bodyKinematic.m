% Indicate the index of the leg
FL = 1;
FR = 2;
BL = 3;
BR = 4;

TESTING for body ik
%setCanvas(100);
scatter3(0,0,0,'wo','linewidth',1);
xlim([-200, 200]);
ylim([-200, 200]);
zlim([-200, 200]);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;


omega = pi/4; %roll
phi = 0; %yaw
psi = 0; % pitch
x=0;
y=0;
z=0;
[T1, T2, T3, T4] = bodyIK(omega, phi, psi, x, y, z);

a = [0 0 0 1]';
A1 = T1*a;
A2 = T2*a;
A3 = T3*a;
A4 = T4*a;
A = [A1 A2 A3 A4]'

%original plot , reversed y-z coordinate
plot3([A(1,1), A(2,1), A(4,1), A(3,1), A(1,1)],[A(1,3), A(2,3), A(4,3), A(3,3), A(1,3)],[A(1,2), A(2,2), A(4,2), A(3,2), A(1,2)],'black','LineWidth',5)
% %self corrdinate plot
% %plot3([A(1,1), A(2,1), A(4,1), A(3,1), A(1,1)],[A(1,2), A(2,2), A(4,2), A(3,2), A(1,2)],[A(1,3), A(2,3), A(4,3), A(3,3), A(1,3)],'black','LineWidth',5)

% body forward kinematic solver
%parameters are body orientation and position for COM
%this function returns for homogenous transform matrix for four hip points
function [T1, T2, T3, T4] = bodyIK(omega, phi, psi, xm, ym, zm)
    L = 120;
    W = 90;
    Tx = [1 0 0 0;
        0 cos(omega) -sin(omega) 0;
        0 sin(omega) cos(omega) 0;
        0 0 0 1];
    Ty = [cos(phi) 0 sin(phi) 0;
        0 1 0 0;
        -sin(phi) 0 cos(phi) 0;
        0 0 0 1];
    Tz = [cos(psi) -sin(psi) 0 0;
        sin(psi) cos(psi) 0 0;
        0 0 1 0;
        0 0 0 1];

    Txyz = Tx*Ty*Tz;
    T = [0 0 0 xm;
        0 0 0 ym;
        0 0 0 zm;
        0 0 0 0];
    Tm = T + Txyz;
    % calculate each hip point homogenous transform matrix
%     r = [0 0 1;
%         0 1 0;
%         -1 0 0;
%         0 0 0];
    r = [1 0 0;
        0 1 0;
        0 0 1;
        0 0 0];
    p1 = [L/2; 0; W/2; 1];
    p2 = [L/2; 0; -W/2; 1];
    p3 = [-L/2; 0; W/2; 1];
    p4 = [-L/2; 0; -W/2; 1];

    T1 = Tm*[r p1];
    T2 = Tm*[r p2];
    T3 = Tm*[r p3];
    T4 = Tm*[r p4];
    
%     T1 = Tm*[0 0 1 L/2;
%         0 1 0 0;
%         -1 0 0 W/2;
%         0 0 0 1];
%     T2 = Tm*[0 0 1 L/2;
%         0 1 0 0;
%         -1 0 0 -W/2;
%         0 0 0 1];
%     T3 = Tm*[0 0 1 -L/2;
%         0 1 0 0;
%         -1 0 0 W/2;
%         0 0 0 1];
%     T4 = Tm*[0 0 1 -L/2;
%         0 1 0 0;
%         -1 0 0 -W/2;
%         0 0 0 1];
end
