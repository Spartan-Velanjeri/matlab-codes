robot = importrobot('horizonforce.urdf');
robot.DataFormat = 'column';
showdetails(robot)

randConfig = robot.randomConfiguration;
tform = getTransform(robot,randConfig,'link4','base_link')

show(robot,randConfig);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];
initialguess = robot.homeConfiguration;

[configSoln,solnInfo] = ik('link4',tform,weights,initialguess);

show(robot,configSoln)