#include "mex.h"
#include "rbdl.h"
#include "ObjectHandle.h"

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 0) {
		mexErrMsgTxt ("rbdlCreateModel() does not take any arguments");
	}

	Model *model = new Model;
	model->Init();

	ObjectHandle<Model> *handle = new ObjectHandle<Model>(model);
	plhs[0] = handle->to_mex_handle();
}
