classdef DobotSimulation < handle

       properties(Access = private)
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-2 2 -2 2 -2 2];   
        
        %> Arm lengths for Dobot
        armLengths = [0.057, 0.135, 0.147, 0, 0];

       end
       
       methods(Access = public)
           %% Class Initialisation
           function self = DobotSimulation()
               %initialising the simulation
               self.getDobot();
               q = zeros(1,5);
               plot(self.model,q);
           end
           
           %% Get Dobot Parameters and create seriallink
           function getDobot(Static)
               
               name = getstr(now, 'yyyy');
               
               L1 = Link('d',0,'a',0.057,'alpha',0,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
               L2 = Link('d',0,'a',0.135,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(5),deg2rad(80)]);
               L3 = Link('d',0,'a',0.147,'alpha',0,'offset',0,'qlim',[deg2rad(15),deg2rad(170)]);
               L4 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[pi/2,pi/2]);
               L5 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-85),deg2rad(85)]);
               
               self.model = SerialLink([L1 L2 L3 L4 L5], 'name', name);
           end
           
           %% Dobot Fkine
           function translation = DobotFkine(q)
               
               x = self.armLengths(2)*cos(q(1))*sin(q(2))+self.armLengths(3)*cos(q(1))*cos(q(3));
               y = self.armLengths(2)*sin(q(1))*sin(q(2))+self.armLengths(3)*sin(q(1))*sin(q(3));
               z = self.armLengths(2)*cos(q(2)) - self.armLengths(3)*sin(q(3))-self.armLengths(1);
               
               translation = [x, y, z];
           end
           %% Dobot Ikine
           function jointAngles = DobotIkine(translation)
               %let translation = [x, y, z]
               [x, y, z] = [translation(1), translation(2), translation(3)];
               length = sqrt(x^2+y^2);
               distance = sqrt(length^2+z^2);
               t1 = atan(z/l);
               t2 = acosd(((armLength(1)^2)+distance^2-(armLength(2))^2)/(2*armLength(1)*distance));
               alpha = t1 + t2;
               beta = acosd(((armLength(1)^2)+(armLength(2)^2)-distance^2)/(2*armLength(1)*armLength(2)));
               jointAngles = [atan(y/x), (pi/2 - alpha), (pi - beta - alpha)];
           end
           
           %% Collision Detection
           function hasCollided = checkForCollision(self, q)
               %%plot cube in workspace to check for collisions
               [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
               sizeMat = size(Y);
               X = repmat(0.75,sizeMat(1),sizeMat(2));
               oneSideOfCube_h = surf(X,Y,Z);

               % Combine one surface as a point cloud
               cubePoints = [X(:),Y(:),Z(:)];

               % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
               cubePoints = [ cubePoints ...
                           ; cubePoints * rotz(pi/2)...
                           ; cubePoints * rotz(pi) ...
                           ; cubePoints * rotz(3*pi/2) ...
                           ; cubePoints * roty(pi/2) ...
                           ; cubePoints * roty(-pi/2)];         
         
               % Plot the cube's point cloud         
               cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
               cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
               cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
               axis equal
               
               robot = self.model;

               % New values for the ellipsoid 
               centerPoint = [0,0,0];
               radii = [1,0.5,0.5];
               [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
               for i = 1:4
                  robot.points{i} = [X(:),Y(:),Z(:)];
                  warning off
                  robot.faces{i} = delaunay(robot.points{i});    
                  warning on;
               end

               %robot.plot3d(q);
               %axis equal
               %camlight

               tr = zeros(4,4,robot.n+1);
               tr(:,:,1) = robot.base;
               L = robot.links;
               for i = 1 : robot.n
                  tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
               end

               % Go through each ellipsoid
               for i = 1: size(tr,3)
                 cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                 updatedCubePoints = cubePointsAndOnes(:,1:3);
                 algebraicDist = self.GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
                 pointsInside = find(algebraicDist < 1);
                 %display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                 if pointsInside > 0
                     hasCollided = true;
                 end
               end
           end
           
           %% Get algabraic distance
           function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

             algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                          + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                          + ((points(:,3)-centerPoint(3))/radii(3)).^2;
           end
       end
end
