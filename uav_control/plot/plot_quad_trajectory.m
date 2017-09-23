clc; clear all; close all;

pose = rossubscriber('/mavros/local_position/pose');
pause(0.5);
globPose = rossubscriber('/mavros/global_position/local');
pause(0.5);
%primitive = rossubscriber('/mavros/motion_primitive');
%pause(0.5);

posX = [];
posY = [];
posZ = [];

globPosX = [];
globPosY = [];
globPosZ = [];

waypointX = [];
waypointY = [];
waypointZ = [];

countWaypoint = 0;

i = 1
while(1)
Odommsg = rostopic('echo', '/mavros/local_position/pose');
OdomX(i)=Odommsg.Pose.Pose.Position.X;
OdomY(i)=Odommsg.Pose.Pose.Position.Y;
OdomZ(i)=Odommsg.Pose.Pose.Position.Z;
end
    
    
    
    
while(1)
    countWaypoint = countWaypoint + 1
    
    msgPose = receive(pose);
    random = 25
    msgGlobPose = receive(globPose);
    random = 50
%     msgPrimitive = receive(primitive);
    
%     s = sprintf('robot position: (%.2f, %.2f, %.2f)', msgPose.Pose.Position.X, msgPose.Pose.Position.Y, msgPose.Pose.Position.Z);
%     disp(s);
    
    posX = [posX msgPose.Pose.Position.X];
    posY = [posY msgPose.Pose.Position.Y];
    posZ = [posZ msgPose.Pose.Position.Z];
    
    if countWaypoint == 1
        initGlobPosX = msgGlobPose.Pose.Pose.Position.X;
        initGlobPosY = msgGlobPose.Pose.Pose.Position.Y;
        initGlobPosZ = msgGlobPose.Pose.Pose.Position.Z;
    end
    
    globPosX = [globPosX msgGlobPose.Pose.Pose.Position.X-initGlobPosX];
    globPosY = [globPosY msgGlobPose.Pose.Pose.Position.Y-initGlobPosY];
    globPosZ = [globPosZ msgGlobPose.Pose.Pose.Position.Z-initGlobPosZ];
    
%     if strcmp(msgPrimitive.Data, 'forward')
%         s = sprintf('data: %s', msgPrimitive.Data);
%         disp(s);
%         waypointX(countWaypoint,:) = [msgPose.Pose.Position.X msgPose.Pose.Position.X];
%         waypointY(countWaypoint,:) = [msgPose.Pose.Position.Y msgPose.Pose.Position.Y+5];
%         waypointZ(countWaypoint,:) = [msgPose.Pose.Position.Z msgPose.Pose.Position.Z];
%     elseif strcmp(msgPrimitive.Data, 'backward')
%         s = sprintf('data: %s', msgPrimitive.Data);
%         disp(s);
%         waypointX(countWaypoint,:) = [msgPose.Pose.Position.X msgPose.Pose.Position.X];
%         waypointY(countWaypoint,:) = [msgPose.Pose.Position.Y msgPose.Pose.Position.Y-5];
%         waypointZ(countWaypoint,:) = [msgPose.Pose.Position.Z msgPose.Pose.Position.Z];
%     elseif strcmp(msgPrimitive.Data, 'right')
%         s = sprintf('data: %s', msgPrimitive.Data);
%         disp(s);
%         waypointX(countWaypoint,:) = [msgPose.Pose.Position.X msgPose.Pose.Position.X+5];
%         waypointY(countWaypoint,:) = [msgPose.Pose.Position.Y msgPose.Pose.Position.Y];
%         waypointZ(countWaypoint,:) = [msgPose.Pose.Position.Z msgPose.Pose.Position.Z];
%     elseif strcmp(msgPrimitive.Data, 'left')
%         s = sprintf('data: %s', msgPrimitive.Data);
%         disp(s);
%         waypointX(countWaypoint,:) = [msgPose.Pose.Position.X msgPose.Pose.Position.X-5];
%         waypointY(countWaypoint,:) = [msgPose.Pose.Position.Y msgPose.Pose.Position.Y];
%         waypointZ(countWaypoint,:) = [msgPose.Pose.Position.Z msgPose.Pose.Position.Z];
%     else
%         continue;
%     end
%     
%     if countWaypoint == 8
%         break;
%     end
%     if countWaypoint == 970
%         break;
%     end
    if countWaypoint == 5
        break;
    end
end

figure(1)
plot3(posX, posY, posZ);
hold on;

% for i = 1:countWaypoint
%     plot3(waypointX(i,:), waypointY(i,:), waypointZ(i,:), 'r');
%     hold on;
% end

title('Trajectory of Arya')
xlabel('x')
ylabel('y')
zlabel('z')

figure(2)
plot3(globPosX, globPosY, globPosZ);
hold on;

xlabel('x')
ylabel('y')
zlabel('z')

