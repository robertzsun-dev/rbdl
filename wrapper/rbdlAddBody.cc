#include "mex.h"
#include "rbdl_mex.h"
#include "ObjectHandle.h"

#include <iostream>
#include <string>

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs < 5) {
		mexErrMsgTxt ("usage: rbdlAddBody(model, parent_id, joint_frame, joint, body [, body_name]");
	}

	ObjectHandle<Model> *handle = ObjectHandle<Model>::from_mex_handle(prhs[0]);
	Model *model = &(handle->get_object());

	// parent_id

	if (!mxIsNumeric(prhs[1])) {
		mexErrMsgTxt("Argument 2 has to be the parent id (i.e. a number!");
		return;
	}

	double value = *mxGetPr(prhs[1]);
	unsigned int parent_id = static_cast<unsigned int>(roundf(value));

	// joint_frame

	Math::SpatialTransform joint_frame;

	for (int fi = 0; fi < mxGetNumberOfFields (prhs[2]); fi++) {
		std::string field_name = mxGetFieldNameByNumber(prhs[2], fi);
		mxArray *field = mxGetField (prhs[2], 0, field_name.c_str());

		if (field_name == "r") {
			if (!GetVector3d (field, joint_frame.r)) {
				mexErrMsgTxt("Error parsing joint_frame.r.");
				return;
			}
		} else if (field_name == "E") {
			if (!GetMatrix3d (field, joint_frame.E)) {
				mexErrMsgTxt("Error parsing joint_frame.E.");
				return;
			}
		} else {
			mexPrintf ("Warning: unknown field in joint transformation: %s!\n", mxGetFieldNameByNumber (prhs[2], fi));
		}
	}

	// joint
	
	Joint joint;

	if (!mxIsDouble(prhs[3])) {
		mexErrMsgTxt("Error: expected parameter joint to be a double matrix");
		return;
	}

	mwSize *dimensions = mxGetDimensions (prhs[3]);
	if (dimensions[0] == 0) {
		joint = Joint(JointTypeFixed);
	} else if (dimensions[1] != 6) {
		mexErrMsgTxt("Error: expected parameter joint to be a n x 6 matrix");
		return;
	}

	unsigned int joint_dof_count = static_cast<unsigned int >(dimensions[0]);
	Math::SpatialVector *axes = NULL;

	if (joint_dof_count == 0) {
		joint = Joint(JointTypeFixed);
	} else {
		axes = new Math::SpatialVector[joint_dof_count];

		double *value_ptr = mxGetPr (prhs[3]);

		for (unsigned int i = 0; i < joint_dof_count; i++) {
			for (unsigned int j = 0; j < 6; j++) {
				axes[i][j] = value_ptr[i + j * joint_dof_count];
			}
		}
	}

	if (joint_dof_count == 1) {
		joint = Joint (axes[0]);
	} else if (joint_dof_count == 2) {
		joint = Joint (axes[0], axes[1]);
	} else if (joint_dof_count == 3) {
		joint = Joint (axes[0], axes[1], axes[2]);
	} else if (joint_dof_count == 4) {
		joint = Joint (axes[0], axes[1], axes[2], axes[3]);
	} else if (joint_dof_count == 5) {
		joint = Joint (axes[0], axes[1], axes[2], axes[3], axes[4]);
	} else if (joint_dof_count == 6) {
		joint = Joint (axes[0], axes[1], axes[2], axes[3], axes[4], axes[5]);
	}

	if (joint_dof_count > 0)
		delete[] axes;

	// body
	
	Body body;
	double mass;
	RigidBodyDynamics::Math::Vector3d com;
	RigidBodyDynamics::Math::Matrix3d inertia;

	for (int fi = 0; fi < mxGetNumberOfFields (prhs[4]); fi++) {
		std::string field_name = mxGetFieldNameByNumber(prhs[4], fi);
		mxArray *field = mxGetField (prhs[4], 0, field_name.c_str());

		if (field_name == "mass") {
			if (!mxIsDouble(field)) {
				mexErrMsgTxt("Error: number expected for body.mass!");
				return;
			}
			mass = *mxGetPr(field);
		} else if (field_name == "com") {
			if (!GetVector3d (field, com)) {
				mexErrMsgTxt("Error parsing body.com");
				return;
			}
		} else if (field_name == "inertia") {
			if (!GetMatrix3d (field, inertia)) {
				mexErrMsgTxt("Error parsing body.inertia.");
				return;
			}
		} else {
			mexPrintf ("Warning: unknown field in body structure: %s!\n", mxGetFieldNameByNumber (prhs[4], fi));
		}
	}

	body = Body (mass, com, inertia);

	// body name (optional)
	
	std::string body_name;

	if (nrhs == 6) {
		if (!mxIsChar(prhs[5])) {
			mexErrMsgTxt("Error: string expected for parameter body_name!");
			return;
		}

		mxChar *param = mxGetChars (prhs[5]);
		mwSize *dimensions = mxGetDimensions (prhs[5]);

		for (int i = 0; i < dimensions[0] * dimensions[1]; i++) {
			body_name = body_name + param[i];
		}
	}

#ifdef RBDL_MEX_DEBUG
	mexPrintf ("parent id is %d\n", parent_id);
	mexPrintf ("joint_frame.r = %f, %f, %f\n", joint_frame.r[0], joint_frame.r[1], joint_frame.r[2]);
	mexPrintf ("joint_frame.E = \n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n",
			joint_frame.E(0,0), joint_frame.E(0,1), joint_frame.E(0,2),
			joint_frame.E(1,0), joint_frame.E(1,1), joint_frame.E(1,2),
			joint_frame.E(2,0), joint_frame.E(2,1), joint_frame.E(2,2)
			);

	mexPrintf ("joint.mDoFCount = %d\n", joint.mDoFCount);
	for (unsigned int i = 0; i < joint.mDoFCount; i++) {
		mexPrintf ("joint.axes[%d] = %f, %f, %f, %f, %f, %f\n", i,
				joint.mJointAxes[i][0], joint.mJointAxes[i][1], joint.mJointAxes[i][2],
				joint.mJointAxes[i][3], joint.mJointAxes[i][4], joint.mJointAxes[i][5]
				);
	}

	mexPrintf ("body.mMass = %f\n", body.mMass);
	mexPrintf ("body.mCenterOfMass = %f, %f, %f\n", body.mCenterOfMass[0], body.mCenterOfMass[1], body.mCenterOfMass[2]);
	mexPrintf ("body.mInertia = \n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n",
			body.mInertia(0,0), body.mInertia(0,1), body.mInertia(0,2),
			body.mInertia(1,0), body.mInertia(1,1), body.mInertia(1,2),
			body.mInertia(2,0), body.mInertia(2,1), body.mInertia(2,2)
			);

	mexPrintf ("body_name = %s\n", body_name.c_str());
#endif

	model->AddBody (parent_id, joint_frame, joint, body, body_name);
}
