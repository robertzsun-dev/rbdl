#include "mex.h"
#include "rbdl_mex.h"
#include "ObjectHandle.h"

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 2) {
		mexErrMsgTxt ("usage: rbdlSetGravity(model, gravity)");
	}

	ObjectHandle<Model> *handle = ObjectHandle<Model>::from_mex_handle(prhs[0]);
	Model* model = &(handle->get_object());

	Math::Vector3d gravity;
	if (!GetVector3d (prhs[1], gravity)) {
		mexErrMsgTxt ("Error parsing gravity argument (expected 3 x 1 vector)");
		return;
	}

	model->gravity = gravity;
}
