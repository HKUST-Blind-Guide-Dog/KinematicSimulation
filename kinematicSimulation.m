setCanvas(200);
%parameters
body = [0.3 0.1 -0.4 0 0 0];
%four leg hip points
legs = [100 -100 100 1;
    100 -100 -100 1;
    -100 -100 100 1;
    -100 -100 -100 1];

dogVisualization(legs, body);

% draw whole robot dog
function dogVisualization(Legs, body)
    [T1, T2, T3, T4] = bodyIK(body(1,1), body(1,2), body(1,3), body(1,4), body(1,5), body(1,6));
    a = [0 0 0 1]';
    A1 = T1*a;
    A2 = T2*a;
    A3 = T3*a;
    A4 = T4*a;
    A = [A1 A2 A3 A4]';
    
    %draw body
    plot3([A(1,1), A(2,1), A(4,1), A(3,1), A(1,1)],[A(1,3), A(2,3), A(4,3), A(3,3), A(1,3)],[A(1,2), A(2,2), A(4,2), A(3,2), A(1,2)],'green','LineWidth',5)
    hold on;
    %rotate 180 degree about x-axis
    Tx = [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];

    %draw legs
    Q = inv(T1)*(Legs(1,1:4))'
    legPoints = calLegPoints(legIK(Q(1),Q(2),Q(3)));
    leg = zeros(5,4);
    for i = 1:5
        x = (legPoints(i,1:4))';
        leg(i,1:4) = (T1*x)';
    end
    drawLeg(leg);

    Q = inv(T2)*(Legs(2,1:4))';
    legPoints = calLegPoints(legIK(Q(1),Q(2),Q(3)));
    leg = zeros(5,4);
    for i = 1:5
        x = (legPoints(i,1:4))';
        leg(i,1:4) = (T2*Tx*x)';
    end
    drawLeg(leg);

    Q = inv(T3)*(Legs(3,1:4))';
    legPoints = calLegPoints(legIK(Q(1),Q(2),Q(3)));
    leg = zeros(5,4);
    for i = 1:5
        x = (legPoints(i,1:4))';
        leg(i,1:4) = (T3*x)';
    end
    drawLeg(leg);

    Q = inv(T4)*(Legs(4,1:4))';
    legPoints = calLegPoints(legIK(Q(1),Q(2),Q(3)));
    leg = zeros(5,4);
    for i = 1:5
        x = (legPoints(i,1:4))';
        leg(i,1:4) = (T4*Tx*x)';
    end
    drawLeg(leg);

end

% body inverse kinematic solver
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

% leg inverse kinematic solver
% jointAngles is a 3x1 vector
function jointAngles = legIK(x,y,z)
    %
    % leg information
    shoulder_length = 25;
    shoulder_hip_vertical_offset = 0; %一般都是0
    elbow_length = 80;
    wrist_length = 80;
    F = sqrt(x^2+y^2-shoulder_length^2);
    G = F - shoulder_hip_vertical_offset;
    H = sqrt(G^2 + z^2);
    
    jointAngles(1,1) = -atan2(y,x) -atan2(F,-shoulder_length);
    % why not divide by -1
    D = (H^2 - elbow_length^2 - wrist_length^2)/(2*elbow_length*wrist_length);
    jointAngles(3,1) = acos(D);
    jointAngles(2,1) = atan2(z,G) - atan2(wrist_length*sin(jointAngles(3,1)),elbow_length+wrist_length*cos(jointAngles(3,1)));
end

% jointPositions is a 5x4 matrix
function jointPositions = calLegPoints(jointAngles)
    % leg information
    shoulder_length = 25;
    shoulder_hip_vertical_offset = 0; %一般都是0
    elbow_length = 80;
    wrist_length = 80;
    theta23 = jointAngles(2,1) + jointAngles(3,1);

    jointPositions(1, 1:4) = [0 0 0 1];
    jointPositions(2, 1:4) = jointPositions(1,1:4)+[-shoulder_length*cos(jointAngles(1,1)), shoulder_length*sin(jointAngles(1,1)), 0, 0];
    jointPositions(3, 1:4) = jointPositions(2,1:4)+[-shoulder_hip_vertical_offset*sin(jointAngles(1,1)), -shoulder_hip_vertical_offset*cos(jointAngles(1,1)), 0, 0];
    jointPositions(4, 1:4) = jointPositions(3,1:4)+[-elbow_length*sin(jointAngles(1,1))*cos(jointAngles(2,1)), -elbow_length*cos(jointAngles(1,1))*cos(jointAngles(2,1)), elbow_length*sin(jointAngles(2,1)), 0];
    jointPositions(5, 1:4) = jointPositions(4,1:4)+[-wrist_length*sin(jointAngles(1,1))*cos(theta23), -wrist_length*cos(jointAngles(1,1))*cos(theta23),wrist_length*sin(theta23), 0];
end

%set the canvas of the plot
function setCanvas(limit)
    scatter3(0,0,0,'wo','linewidth',1);
    xlim([-limit, limit]);
    ylim([-limit, limit]);
    zlim([-limit, limit]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    hold on;
end

%draw legs
function drawLeg(jointPosition)
    % draw lines
    plot3([jointPosition(1,1),jointPosition(2,1),jointPosition(3,1),jointPosition(4,1),jointPosition(5,1)], [jointPosition(1,3),jointPosition(2,3),jointPosition(3,3),jointPosition(4,3),jointPosition(5,3)],[jointPosition(1,2),jointPosition(2,2),jointPosition(3,2),jointPosition(4,2),jointPosition(5,2)],'k-','LineWidth',4);
    % drwa hip and foot point
    plot3(jointPosition(1,1), jointPosition(1,3),jointPosition(1,2),'bo','LineWidth',6);
    plot3(jointPosition(5,1), jointPosition(5,3),jointPosition(5,2),'ro','LineWidth',6);
end