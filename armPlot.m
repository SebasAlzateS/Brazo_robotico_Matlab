function  armGraph = armPlot(x,y,z,q1,q2,q3,q4,scale)

global Arm

b = 0.065*scale;
l1 = 0.09*scale;
l2 = 0.15*scale;

robotPatch = Arm.parte1Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x;
robotPatch(2,:) = robotPatch(2,:)*scale+y;
robotPatch(3,:) = robotPatch(3,:)*scale+z;

armGraph(1) = patch('Faces',Arm.parte1Faces,'Vertices',robotPatch','FaceColor',[0.3 0.3 0.3],'EdgeColor','none');

robotPatch = Arm.parte2Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x;
robotPatch(2,:) = robotPatch(2,:)*scale+y;
robotPatch(3,:) = robotPatch(3,:)*scale+z;

armGraph(2) = patch('Faces',Arm.parte2Faces,'Vertices',robotPatch','FaceColor','y','EdgeColor','none');

Rz = [cos(q1),-sin(q1), 0; sin(q1), cos(q1), 0; 0, 0, 1];

robotPatch = Rz*Arm.parte3Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x;
robotPatch(2,:) = robotPatch(2,:)*scale+y;
robotPatch(3,:) = robotPatch(3,:)*scale+z+b;

armGraph(3) = patch('Faces',Arm.parte3Faces,'Vertices',robotPatch','FaceColor','y','EdgeColor','none');

Ry = [cos(-q2), 0, sin(-q2); 0, 1, 0;-sin(-q2), 0 , cos(-q2)];

robotPatch = Rz*Ry*Arm.parte4Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x;
robotPatch(2,:) = robotPatch(2,:)*scale+y;
robotPatch(3,:) = robotPatch(3,:)*scale+z+b;

armGraph(4) = patch('Faces',Arm.parte4Faces,'Vertices',robotPatch','FaceColor',[0.3 0.3 0.3],'EdgeColor','none');

Ry = [cos(-q2-q3), 0, sin(-q2-q3); 0, 1, 0;-sin(-q2-q3), 0 , cos(-q2-q3)];

robotPatch = Rz*Ry*Arm.parte5Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x+l1*cos(q2)*cos(q1);
robotPatch(2,:) = robotPatch(2,:)*scale+y+l1*cos(q2)*sin(q1);
robotPatch(3,:) = robotPatch(3,:)*scale+z+b+l1*sin(q2);

armGraph(5) = patch('Faces',Arm.parte5Faces,'Vertices',robotPatch','FaceColor','y','EdgeColor','none');

robotPatch = Rz*Ry*Arm.parte6Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x+l1*cos(q2)*cos(q1);
robotPatch(2,:) = robotPatch(2,:)*scale+y+l1*cos(q2)*sin(q1);
robotPatch(3,:) = robotPatch(3,:)*scale+z+b+l1*sin(q2);

armGraph(6) = patch('Faces',Arm.parte6Faces,'Vertices',robotPatch','FaceColor',[0.3 0.3 0.3],'EdgeColor','none');

Ry = [cos(-q2-q3-q4), 0, sin(-q2-q3-q4); 0, 1, 0;-sin(-q2-q3-q4), 0 , cos(-q2-q3-q4)];

robotPatch = Rz*Ry*Arm.parte7Vertices;
robotPatch(1,:) = robotPatch(1,:)*scale+x+l1*cos(q2)*cos(q1)+l2*cos(q2+q3)*cos(q1);
robotPatch(2,:) = robotPatch(2,:)*scale+y+l1*cos(q2)*sin(q1)+l2*cos(q2+q3)*sin(q1);
robotPatch(3,:) = robotPatch(3,:)*scale+z+b+l1*sin(q2)+l2*sin(q2+q3);

armGraph(7) = patch('Faces',Arm.parte7Faces,'Vertices',robotPatch','FaceColor',[0.3 0.3 0.3],'EdgeColor','none');