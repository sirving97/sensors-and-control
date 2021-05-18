%% Sensors and Control Script
clear all;
clc;


%% Initialise ROS
rosinit

%% initialise robot
dobot = DobotControl();

%% Dobot initialise
dobot.HomeDobot();

%% Open Gripper 
dobot.OpenGripperDobot();

%% Close Gripper
dobot.CloseGripperDobot();

%% Get end effector
dobot.GetEndEffectorPosition();

%% test move robot
pose = [0.1402; 0; 0];
dobot.MoveToCartesianPoint(pose);


%% The rest

clc;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
plotHandle = trplot(eye(4));
hold on;
desiredTagPose = eye(4);
while (true)
    tagMsg = receive(ARTagSub);
    numTags = size(tagMsg.Poses);
    while numTags(1) < 1
        pause(1);
        tagMsg = receive(ARTagSub);
        numTags = size(tagMsg.Poses);
        disp("no visible tags");
    end
    currentPose = tagMsg.Poses(1);
    currentPosition = currentPose.Position
    currentOrientation = currentPose.Orientation;
    rotMatrix = quat2rotm([currentOrientation.X currentOrientation.Y currentOrientation.Z currentOrientation.W]);
    homMatrix = rotMatrix;
    homMatrix(1,4) = currentPosition.X;
    homMatrix(2,4) = currentPosition.Y;
    homMatrix(3,4) = currentPosition.Z;
    homMatrix(4,4) = 1;
    %homMatrix is the pose of the tag from the camera in the camera frame
    
    %below calculates the difference between the current camera pose and
    %the desired camera pose
    %trplot(plotHandle,homMatrix);
    if desiredTagPose == eye(4)
        desiredTagPose = homMatrix;
        desiredInvert = HomInvert(desiredTagPose);
    else
        poseDiff = homMatrix * desiredInvert
        trplot(plotHandle,poseDiff);
        %currentEndEffector = dobot.GetEndEffectorPosition();

        x = currentEndEffector(1) + poseDiff(1,4); %camera z is robot x
        y = currentEndEffector(2) + poseDiff(2,4); %camera y is robot y
        z = currentEndEffector(3) + poseDiff(3,4); %camera x is robot z 
    
        newPose = [x; y; z]
        
        dobot.MoveToCartesianPoint(newPose);
        
        pause(1);
    end
    
end






%% The rest option 2

clc;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');

startX = 0;
startY = 0;
startZ = 0;

iterator = 0;


while (true)
    tagMsg = receive(ARTagSub);
    numTags = size(tagMsg.Poses);
    while numTags(1) < 1
        pause(1);
        tagMsg = receive(ARTagSub);
        numTags = size(tagMsg.Poses);
        disp("no visible tags");
    end
    currentPose = tagMsg.Poses(1);
    currentPosition = currentPose.Position;

    if iterator < 1
        startX = currentPosition.X;
        startY = currentPosition.Y;
        startZ = currentPosition.Z;  
    else
        currentEndEffector = dobot.GetEndEffectorPosition();
        
        xDiff = currentPosition.X - startX;
        yDiff = currentPosition.Y - startY;
        zDiff = currentPosition.Z - startZ;

        x = currentEndEffector(1) + xDiff;
        y = currentEndEffector(2) + yDiff; 
        z = currentEndEffector(3) + zDiff; 
    
        newPose = [x; y; z]
        
        dobot.MoveToCartesianPoint(newPose);
        
        pause(0.2);
    end
    
     iterator = iterator + 1;
    
end


%% test robot control

for x = 1:1:10
    i = x * 0.01;
    newPose2 = [0.1408, i, 0];
    dobot.MoveToCartesianPoint(newPose2);
    disp("run");
end



%% functions and shit

function result = HomInvert(transform)
    result = eye(4);
    rot = t2r(transform)';
    result(1:3,4) = -rot * transform(1:3,4);
    result(1:3, 1:3) = rot;
end