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


%% Hand eye calibration 

%This part of the code finds the transform between the camera and the dobot
%end effector by looking at an ar tag with a known transform from the robot
%base.

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
tagMsg = receive(ARTagSub);

%get the tag pose and convert to a homogeneous transform called homMatrix
currentPose = tagMsg.Poses(1);
currentPosition = currentPose.Position;
currentOrientation = currentPose.Orientation;
rotMatrix = quat2rotm([currentOrientation.X currentOrientation.Y currentOrientation.Z currentOrientation.W]);
homMatrix = rotMatrix;
homMatrix(1,4) = currentPosition.X;
homMatrix(2,4) = currentPosition.Y;
homMatrix(3,4) = currentPosition.Z;
homMatrix(4,4) = 1;

%invert the transform so that we have the camera pose in the coordinate
%frame of the tag
tr_tag2Camera = inv(homMatrix);

%known offset between the tag and the robot base in meters [x; y; z]
%converted to a homogeneous transform with no rotation.
offset = [0.1; 0; 0];

%in this case we have manually lined the rotation axis of the tag with the 
%axis of the robot so that we don't need to add the rotation

%get the camera pose in the robot base frame by adding the offset 
tr_base2Camera = tr_tag2Camera;
tr_base2Camera(1,4) = tr_base2Camera(1,4) + offset(1); % X
tr_base2Camera(2,4) = tr_base2Camera(2,4) + offset(2); % Y
tr_base2Camera(3,4) = tr_base2Camera(3,4) + offset(3); % Z

%get the current end effector position
currentEndEffector = dobot.GetEndEffectorPosition();

%now we just need to compare the two to find the difference
%note that in our case the camera orientation and the robot end effector
%are the same so we only need to compare x, y and z
diffX = tr_base2Camera(1,4) - currentEndEffector(1);
diffY = tr_base2Camera(2,4) - currentEndEffector(2);
diffZ = tr_base2Camera(3,4) - currentEndEffector(3);

%now convert to transform between the end effector and camera 
tr_EEF2Camera = eye(4);
tr_EEF2Camera(1,4) = diffX;
tr_EEF2Camera(2,4) = diffY;
tr_EEF2Camera(3,4) = diffZ;




%% Control loop using transforms

%This method controls the robot by computing the difference in the
%transform between the current tag pose and the desired tag pose.
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
    currentPosition = currentPose.Position;
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

        %x = currentEndEffector(1) + poseDiff(1,4); 
        %y = currentEndEffector(2) + poseDiff(2,4); 
        %z = currentEndEffector(3) + poseDiff(3,4); 
    
        %newPose = [x; y; z]
        
        %dobot.MoveToCartesianPoint(newPose);
        
        %pause(0.2);
    end
    
end






%% Robot control using just cartesian points 

%Control of the robot using just the x, y, z pose of the tag and the cartesian
%control function in the dobot driver

%We found that this implementation made the code much easier to understand
%when compared to the method above using transforms

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




%% Functions

%Function to invert a homogeneous transform
function result = HomInvert(transform)
    result = eye(4);
    rot = t2r(transform)';
    result(1:3,4) = -rot * transform(1:3,4);
    result(1:3, 1:3) = rot;
end