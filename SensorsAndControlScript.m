%% Sensors and Control Script

%This is the main script for performing the hand eye calibration of the
%dobot robot and running the control loop to make the robot follow the ar
%tag. 

% The sections have been numbered to show the order that they should be
% run. There are still a lot of sections that have been kept in the code
% and used for debugging etc. You only need to run the numbered sections in
% order to make the dobot follow an ar tag. Make sure you have all of the
% required rose nodes and topics running in the background.
clear all;
clc;

%% Simulating the dobot in matlab

%creates and object of the dobotSimulation class to model a simulated dobot
%in matlab. This part was only used for visulaising the dobot motion and
%understainding the control better

clc;
clf;

dobot = DobotSimulation();


%% 1. Initialise ROS
rosinit

%% 2. Initialise robot
dobot = DobotControl();

%% Dobot initialise

%Returns the dobot to its home position. Not necessary to run everytime but
%a good starting point for the tag following. 
dobot.HomeDobot();

%% Open Gripper 

%used for holding the camera
dobot.OpenGripperDobot();

%% Close Gripper

%used for holding the camera
dobot.CloseGripperDobot();

%% Get end effector
dobot.GetEndEffectorPosition();

%% test move robot
pose = [0.1402; 0; 0];
dobot.MoveToCartesianPoint(pose);


%% 3. Hand eye calibration 

%This part of the code finds the transform between the camera and the dobot
%end effector by looking at an ar tag with a known transform from the robot
%base.

%Due to the fact that the realsense camera is pre-calibrated from the
%factory, this is not necessary to run for the ar tag following but useful
%for any other applications where the gripper-camera transform is needed

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

%this is option 1 of 2 for controlling the dobot
clc;

%get ar tag information from ar_track_alvar_msgs/AlvarMarkers
ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
plotHandle = trplot(eye(4));
hold on;
desiredTagPose = eye(4);
while (true)
    
    %recieve tag data
    tagMsg = receive(ARTagSub);
    numTags = size(tagMsg.Poses);
    
    %loop to inform us if there are no tags visible
    while numTags(1) < 1
        pause(1);
        tagMsg = receive(ARTagSub);
        numTags = size(tagMsg.Poses);
        disp("no visible tags");
    end
    
    %convert tag pose to transform
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

    %first part of if statement stores the transform the first time the
    %camera sees the tag and uses that as the desired pose for comparison
    if desiredTagPose == eye(4)
        desiredTagPose = homMatrix;
        desiredInvert = HomInvert(desiredTagPose);
    else
        
        %calculate the error and plot for visualisation
        poseDiff = homMatrix * desiredInvert
        trplot(plotHandle,poseDiff);
        
        %gets the current end effector pose
        currentEndEffector = dobot.GetEndEffectorPosition();

        x = currentEndEffector(1) + poseDiff(1,4); 
        y = currentEndEffector(2) + poseDiff(2,4); 
        z = currentEndEffector(3) + poseDiff(3,4); 
    
        %format so that the robot can read it
        newPose = [x; y; z]
        
        %move the robot to the new pose 
        dobot.MoveToCartesianPoint(newPose);
        
        %the dobot requires a small delay so that it can process the
        %commands
        pause(0.2);
    end
    
end






%% 4. Robot control using just cartesian points 

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
    
    %recieve the tag pose
    tagMsg = receive(ARTagSub);
    numTags = size(tagMsg.Poses);
    
    %inform us if no visible tags
    while numTags(1) < 1
        pause(1);
        tagMsg = receive(ARTagSub);
        numTags = size(tagMsg.Poses);
        disp("no visible tags");
    end
    currentPose = tagMsg.Poses(1);
    currentPosition = currentPose.Position;

    if iterator < 1
        
        %desired tag pose is stored on the first run of the loop
        startX = currentPosition.X;
        startY = currentPosition.Y;
        startZ = currentPosition.Z;  
        
    else
        
        %get end effector pose
        currentEndEffector = dobot.GetEndEffectorPosition();
        
        %below calculates the error in the x, y and z directions separately
        %and then creates the new pose for the robot to move to
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
    
     iterator = iterator + 1; %increment iterator for tracking the loop iterations
    
end




%% Other Functions

%Function to invert a homogeneous transform
function result = HomInvert(transform)
    result = eye(4);
    rot = t2r(transform)';
    result(1:3,4) = -rot * transform(1:3,4);
    result(1:3, 1:3) = rot;
end