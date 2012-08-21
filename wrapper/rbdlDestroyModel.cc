#include "mex.h"
#include "rbdl.h"
#include "ObjectHandle.h"

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 1) {
		mexErrMsgTxt ("rbdlGetDofCount(model) needs the model as argument.");
	}

	ObjectHandle<Model> *handle = ObjectHandle<Model>::from_mex_handle(prhs[0]);
	delete handle;
}
