dhparams = [0   	0	.333   	0;
            0	pi/2       0       0;
            0	-pi/2	.316	0;
            0.088   	pi/2	0	0;
            -0.088   	-pi/2	0.384	0;
            0       pi/2	0   	0;
            0.088       pi/2       0       0;
            0   0   .107    0];

robot = rigidBodyTree;

bodies = cell(8,1);
joints = cell(8,1);
for i = 1:8
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)

figure(Name="Panda Robot")
show(robot);

%figure(Name="Interactive GUI")
%gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);