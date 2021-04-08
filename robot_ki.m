robot = importrobot('horizondawn.urdf');
robot.DataFormat = 'column';
numJoints = numel(homeConfiguration(robot));
tSpan = 0:0.01:0.5;
q0 = zeros(numJoints,1);
q0(2) = pi/4;
qd0 = zeros(numJoints,1);
initialState = [q0;qd0];


%Set up Joint Control targets
targetJointPosition = [0 0 0]';
targetJointVelocity = zeros(numJoints,1);
targetJointAcceleration = zeros(numJoints,1);

show(robot,targetJointPosition);
%Computed-torque control uses an inverse-dynamics 
%computation to compensate for the robot dynamics.
%The controller drives the closed-loop error dynamics of
%each joint based on a second-order response. 

computedTorqueMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType","ComputedTorqueControl");
updateErrorDynamicsFromStep(computedTorqueMotion,0.2,0.1);
qDesComputedTorque = [targetJointPosition; targetJointVelocity; targetJointAcceleration];

IndepJointMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType","IndependentJointMotion");
updateErrorDynamicsFromStep(IndepJointMotion,0.2,0.1);
qDesIndepJoint = [targetJointPosition; targetJointVelocity; targetJointAcceleration];

pdMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType","PDControl");
pdMotion.Kp = diag(300*ones(1,3));
pdMotion.Kd = diag(10*ones(1,3));
qDesPD = [targetJointPosition; targetJointVelocity];

[tComputedTorque,yComputedTorque] = ode45(@(t,y)derivative(computedTorqueMotion,y,qDesComputedTorque),tSpan,initialState);
[tIndepJoint,yIndepJoint] = ode45(@(t,y)derivative(IndepJointMotion,y,qDesIndepJoint),tSpan,initialState);
[tPD,yPD] = ode15s(@(t,y)derivative(pdMotion,y,qDesPD),tSpan,initialState);

% Computed Torque Control
figure
subplot(2,1,1)
plot(tComputedTorque,yComputedTorque(:,1:numJoints)) % Joint position
hold all
plot(tComputedTorque,targetJointPosition*ones(1,length(tComputedTorque)),'--') % Joint setpoint
title('Computed Torque Motion: Joint Position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(2,1,2)
plot(tComputedTorque,yComputedTorque(:,numJoints+1:end)) % Joint velocity
title('Joint Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')

% Independent Joint Motion
figure
subplot(2,1,1)
plot(tIndepJoint,yIndepJoint(:,1:numJoints))
hold all
plot(tIndepJoint,targetJointPosition*ones(1,length(tIndepJoint)),'--')
title('Independent Joint Motion: Position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(2,1,2);
plot(tIndepJoint,yIndepJoint(:,numJoints+1:end))
title('Joint Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')

% PD with Gravity Compensation
figure
subplot(2,1,1)
plot(tPD,yPD(:,1:numJoints))
hold all
plot(tPD,targetJointPosition*ones(1,length(tPD)),'--')
title('PD Controlled Joint Motion: Position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(2,1,2)
plot(tPD,yPD(:,numJoints+1:end))
title('Joint Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')


