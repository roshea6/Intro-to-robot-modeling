dhparams = [0   	pi/2	.128   	pi;
            -0.6127	pi       0       -pi/2;
            -0.5716	pi	0	0;
            0   	-pi/2	0.1639	pi/2;
            0       pi/2	0.1157   	0;
            0       0       0.0922       0];

robot = rigidBodyTree;

bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
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

figure(Name="UR-10 Robot")
show(robot);

%figure(Name="Interactive GUI")
%gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
