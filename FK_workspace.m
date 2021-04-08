% Workspace plotting code using max and min possible angles of the servos

%Lengths in mm
L1 = 235;
L2 = 160;
L3 = 80;
L4 = 103;
X = [];
Y = [];
Z = [];
%Forward Kinematics equations for x,y,z

for t1 = -0.87: 0.01 : 0.87
    for t2 = -0.61: 0.01: 0.61
        for t3 = -0.26: 0.01: 0.78
            x = -(L4*(cos(t2)*sin(t3)+sin(t2)*cos(t3))+L3*sin(t2));
            y = (cos(t1)*cos(t2)*cos(t3) - cos(t1)*sin(t2)*sin(t3))*L4 + L3*cos(t1)*cos(t2) + L2*cos(t1) + L1;
            z = ((-sin(t1)*sin(t2)*sin(t3) + sin(t1)*cos(t2)*cos(t3))*L4 + L3*sin(t1)*cos(t2) + L2*sin(t1));
            X(end+1) = x; 
            Y(end+1) = y;
            Z(end+1) = z; 
      
            
        end
        
    end
    
end

