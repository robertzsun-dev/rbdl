#include "rbdl.h"

bool GetVectorNd (const mxArray *array, RigidBodyDynamics::Math::VectorNd &vector_out) {
	if (!mxIsDouble(array)) {
		mexErrMsgTxt("Error: expected an array of double values!");
		return false;
	}

	mwSize *dimensions = mxGetDimensions (array);

	if (dimensions[1] != 1) {
		mexErrMsgTxt ("Error: expected a column vector for variable size vector!");
		return false;
	}

	unsigned int dim = static_cast<unsigned int> (dimensions[0] * dimensions[1]);

	if (vector_out.size() != dim)
		vector_out = RigidBodyDynamics::Math::VectorNd(dim);

	double *value_ptr = mxGetPr (array);

	for (unsigned int i = 0; i < dim; i++)
		vector_out[i] = value_ptr[i];

	return true;
}

bool GetVector3d (const mxArray *array, RigidBodyDynamics::Math::Vector3d &vector3d_out) {
	mwSize *dimensions = mxGetDimensions (array);

	if (!mxIsDouble(array)) {
		mexErrMsgTxt("Error: expected an array of double values!");
		return false;
	}

	if (dimensions[0] * dimensions[1] != 3) {
		mexErrMsgTxt("Error: expected an array of size 3!\n");
		return false;
	}

	double *value_ptr = mxGetPr (array);

	vector3d_out[0] = value_ptr[0];
	vector3d_out[1] = value_ptr[1];
	vector3d_out[2] = value_ptr[2];

	return true;
}

bool GetMatrix3d (const mxArray *array, RigidBodyDynamics::Math::Matrix3d &matrix3d_out) {
	mwSize *dimensions = mxGetDimensions (array);

	if (!mxIsDouble(array)) {
		mexErrMsgTxt("Error: expected an array of double values!");
		return false;
	}

	if (dimensions[0] * dimensions[1] != 9) {
		mexErrMsgTxt("Error: expected an array of size 9!\n");
		return false;
	}

	double *value_ptr = mxGetPr (array);

	matrix3d_out(0,0) = value_ptr[0];
	matrix3d_out(1,0) = value_ptr[1];
	matrix3d_out(2,0) = value_ptr[2];

	matrix3d_out(0,1) = value_ptr[3];
	matrix3d_out(1,1) = value_ptr[4];
	matrix3d_out(2,1) = value_ptr[5];

	matrix3d_out(0,2) = value_ptr[6];
	matrix3d_out(1,2) = value_ptr[7];
	matrix3d_out(2,2) = value_ptr[8];

	return true;
}
