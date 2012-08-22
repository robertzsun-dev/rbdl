#include "mex.h"
#include "rbdl_mex.h"
#include "ObjectHandle.h"

#include <string>

using namespace RigidBodyDynamics;

void mexFunction (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 4) {
		mexErrMsgTxt ("usage: tau = rbdlInverseDynamics(model, q, qdot, qddot");
		return;
	}

	ObjectHandle<Model> *handle = ObjectHandle<Model>::from_mex_handle(prhs[0]);
	Model *model = &(handle->get_object());

	if (!mxIsNumeric(prhs[1])) {
		mexErrMsgTxt("Argument 2 has to be the parent id (i.e. a number!");
		return;
	}

	RigidBodyDynamics::Math::VectorNd q (model->dof_count), 
					 qdot(model->dof_count), 
					 qddot (model->dof_count),
					 tau (model->dof_count);

	if (!GetVectorNd (prhs[1], q)) {
		mexErrMsgTxt ("Error parsing argument 1 (q)");
		return;
	}

	if (!GetVectorNd (prhs[2], qdot)) {
		mexErrMsgTxt ("Error parsing argument 2 (qdot)");
		return;
	}

	if (!GetVectorNd (prhs[3], qddot)) {
		mexErrMsgTxt ("Error parsing argument 3 (qddot)");
		return;
	}

	RigidBodyDynamics::InverseDynamics (*model, q, qdot, qddot, tau);

	mwSize result_dim[2];
	result_dim[0] = static_cast<mwSize> (tau.size());
	result_dim[1];

	plhs[0] = (mxArray*) mxCreateNumericArray (
			1,
			result_dim,
			mxGetClassID (prhs[1]),
			mxREAL);

	double *value_ptr = mxGetPr (plhs[0]);

	for (unsigned int i = 0; i < tau.size(); i++) {
		value_ptr[i] = tau[i];
	}
}
