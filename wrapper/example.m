model = rbdlCreateModel()

joint_frame.r = [1.3, 5.1, 1.4];
joint_frame.E = [[1.0, 0.0, 0.0]; [0.0, 1.0, 0.0]; [0.0, 0.0, 1.0]];
joint = [ 
	[0., 0., 0., 1., 0., 0.]; 
	[0., 0., 0., 0., 1., 0.];
	[0., 0., 1., 0., 0., 0.];
		]

body.mass = 93.;
body.com = [1.1, 0.2, 0.3];
body.inertia = [[1.0, 0.1, 0.2]; [0.1, 2., 0.3]; [0.2, 0.3, 3.]]

rbdlSetGravity (model, [0., 9.81, 0.]);
gravity = rbdlGetGravity (model)

rbdlAddBody (model, 0, joint_frame, joint, body, "test_body");
rbdlAppendBody (model, joint_frame, joint, body, "test_body_appended");

dof_count = rbdlGetDofCount(model)

q = ones(dof_count, 1);
qdot = ones(dof_count, 1);
qddot = ones(dof_count, 1);
tau = ones(dof_count, 1);

qddot = rbdlForwardDynamics (model, q, qdot, tau)
tau = rbdlInverseDynamics (model, q, qdot, qddot)

rbdlDestroyModel(model)
