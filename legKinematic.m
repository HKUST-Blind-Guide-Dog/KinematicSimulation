%testing for legik
setCanvas(110);
angles = legIK(-55,-100,20)
degreeAngles = angles*180/pi
positions = calLegPoints(angles)
drawLeg(calLegPoints(legIK(-55,-100,20)));

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

function drawLeg(jointPosition)
    % draw lines
    plot3([jointPosition(1,1),jointPosition(2,1),jointPosition(3,1),jointPosition(4,1),jointPosition(5,1)], [jointPosition(1,3),jointPosition(2,3),jointPosition(3,3),jointPosition(4,3),jointPosition(5,3)],[jointPosition(1,2),jointPosition(2,2),jointPosition(3,2),jointPosition(4,2),jointPosition(5,2)],'k-','LineWidth',4);
    % drwa hip and foot point
    plot3(jointPosition(1,1), jointPosition(1,3),jointPosition(1,2),'bo','LineWidth',6);
    plot3(jointPosition(5,1), jointPosition(5,3),jointPosition(5,2),'ro','LineWidth',6);
end
