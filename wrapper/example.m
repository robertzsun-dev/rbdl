model = rbdlCreateModel()

joint_frame.r = [1.3, 5.1, 1.4];
joint_frame.E = [[0.0, 0.1, 0.2]; [1.0, 1.1, 1.2]; [2.0, 2.1, 2.2]];
joint = [ 
	[0., 0., 0., 1., 0., 0.]; 
	[0., 0., 0., 0., 1., 0.];
	[0., 0., 1., 0., 0., 0.];
		]

body.mass = 93.;
body.com = [1.1, 0.2, 0.3];
body.inertia = [[0.0, 0.1, 0.2]; [1.0, 1.1, 1.2]; [2.0, 2.1, 2.2]]

rbdlAddBody (model, 0, joint_frame, joint, body, "test_body");
rbdlAppendBody (model, joint_frame, joint, body, "test_body_appended");

dof_count = rbdlGetDofCount(model)

rbdlDestroyModel(model)
