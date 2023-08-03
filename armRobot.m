function armRobot

    global Arm
    
    parte1 = stlRead('1_base_brazo_robotico_4dof.stl');
    parte2 = stlRead('2_base_brazo_robotico_4dof.stl');
    parte3 = stlRead('3_joint1_brazo_robotico_4dof.stl');
    parte4 = stlRead('4_joint1_brazo_robotico_4dof.stl');
    parte5 = stlRead('5_joint2_brazo_robotico_4dof.stl');
    parte6 = stlRead('6_joint2_brazo_robotico_4dof.stl');
    parte7 = stlRead('7_joint3_brazo_robotico_4dof.stl');
    parte8 = stlRead('8_joint3_brazo_robotico_4dof.stl');
    
    Arm.parte1Vertices = parte1.vertices';
    Arm.parte1Faces = parte1.faces;
    
    Arm.parte2Vertices = parte2.vertices';
    Arm.parte2Faces = parte2.faces;
    
    Arm.parte3Vertices = parte3.vertices';
    Arm.parte3Faces = parte3.faces;
    
    Arm.parte4Vertices = parte4.vertices';
    Arm.parte4Faces = parte4.faces;
    
    Arm.parte5Vertices = parte5.vertices';
    Arm.parte5Faces = parte5.faces;
    
    Arm.parte6Vertices = parte6.vertices';
    Arm.parte6Faces = parte6.faces;
    
    Arm.parte7Vertices = parte7.vertices';
    Arm.parte7Faces = parte7.faces;
    
    Arm.parte8Vertices = parte8.vertices';
    Arm.parte8Faces = parte8.faces;
    
end