#include "mex.h"
#include "rbdl_mex.h"
#include "ObjectHandle.h"

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 1) {
		mexErrMsgTxt ("usage: gravity = rbdlGetGravity(model)");
	}

	ObjectHandle<Model> *handle = ObjectHandle<Model>::from_mex_handle(prhs[0]);
	Model* model = &(handle->get_object());

	mwSize result_dim[2];
	result_dim[0] = static_cast<mwSize> (3);
	result_dim[1];

	plhs[0] = (mxArray*) mxCreateNumericArray (
			1,
			result_dim,
			mxDOUBLE_CLASS,
			mxREAL);

	double *value_ptr = mxGetPr (plhs[0]);

	for (unsigned int i = 0; i < 3; i++) {
		value_ptr[i] = model->gravity[i];
	}
}
