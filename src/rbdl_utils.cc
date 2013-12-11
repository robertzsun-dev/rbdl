#include "rbdl/rbdl_utils.h"

#include "rbdl/rbdl_math.h"
#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

#include <sstream>
#include <iomanip>

namespace RigidBodyDynamics {

namespace Utils {

using namespace std;
using namespace Math;

string get_dof_name (const SpatialVector &joint_dof) {
	if (joint_dof == SpatialVector (1., 0., 0., 0., 0., 0.)) 
		return "RX";
	else if (joint_dof == SpatialVector (0., 1., 0., 0., 0., 0.))
		return "RY";
	else if (joint_dof == SpatialVector (0., 0., 1., 0., 0., 0.))
		return "RZ";
	else if (joint_dof == SpatialVector (0., 0., 0., 1., 0., 0.))
		return "TX";
	else if (joint_dof == SpatialVector (0., 0., 0., 0., 1., 0.))
		return "TY";
	else if (joint_dof == SpatialVector (0., 0., 0., 0., 0., 1.))
		return "TZ";

	ostringstream dof_stream(ostringstream::out);
	dof_stream << "custom (" << joint_dof.transpose() << ")";
	return dof_stream.str();
}

string get_body_name (const RigidBodyDynamics::Model &model, unsigned int body_id) {
	if (model.mBodies[body_id].mMass == 0.) {
		// this seems to be a virtual body that was added by a multi dof joint
		unsigned int child_index = 0;

		// if there is not a unique child we do not know what to do...
		if (model.mu[body_id].size() != 1)
			return "";

		unsigned int child_id = model.mu[body_id][0];

		return get_body_name (model, model.mu[body_id][0]);
	}

	return model.GetBodyName(body_id);
}

RBDL_DLLAPI std::string GetDofDescription (const Model &model, unsigned int dof_index) {
	unsigned int joint_index = 0;
	unsigned int axis_index = 0;

	if (dof_index >= model.dof_count) {
		// handle w component of a quaternion
		axis_index = 3;
		int quat_index = model.dof_count- dof_index + 1;
		joint_index = 0;
		do {
			if (model.mJoints[joint_index].mJointType == JointTypeSpherical) {
				quat_index --;
			}
			joint_index ++;
		} while (quat_index > 0);
		joint_index --;
	} else {
		for (joint_index = 0; joint_index < model.mJoints.size(); joint_index++) {
			unsigned int q_index = model.mJoints[joint_index].q_index;
			if (q_index <= dof_index  && q_index + model.mJoints[joint_index].mDoFCount > dof_index) {
				axis_index = dof_index - q_index;
				break;
			}
		}
	}

	if (model.mJoints[joint_index].mJointType == JointTypeSpherical) {
		string dof_name;
		switch (axis_index) {
			case 0: dof_name = dof_name + "_QX"; break;
			case 1: dof_name = dof_name + "_QY"; break;
			case 2: dof_name = dof_name + "_QZ"; break;
			case 3: dof_name = dof_name + "_QW"; break;
			default: break;
		}
		
		return get_body_name(model, joint_index) + dof_name;
	}

	return get_body_name(model, joint_index) + std::string("_") + get_dof_name (model.mJoints[joint_index].mJointAxes[axis_index]);
}

RBDL_DLLAPI std::string GetModelDofOverview (const Model &model) {
	stringstream result ("");

	for (unsigned int i = 0; i < model.q_size; i++) {
		result << setfill(' ') << setw(3) << i << ": " << GetDofDescription(model, i) << endl;
	}

	return result.str();
}

std::string print_hierarchy (const RigidBodyDynamics::Model &model, unsigned int body_index = 0, int indent = 0) {
	stringstream result ("");

	for (int j = 0; j < indent; j++)
		result << "  ";

	result << get_body_name (model, body_index);

	if (body_index == 0) {
		result << endl;
		result << print_hierarchy (model, 1, 1);
		return result.str();
	} 

	// print the dofs
	result << " [ ";

	while (model.mBodies[body_index].mMass == 0.) {
		if (model.mu[body_index].size() == 0) {
			result << " end";
			break;
		} else if (model.mu[body_index].size() > 1) {
			cerr << endl << "Error: Cannot determine multi-dof joint as massless body with id " << body_index << " (name: " << model.GetBodyName(body_index) << ") has more than one child:" << endl;
			for (unsigned int ci = 0; ci < model.mu[body_index].size(); ci++) {
				cerr << "  id: " << model.mu[body_index][ci] << " name: " << model.GetBodyName(model.mu[body_index][ci]) << endl;
			}
			abort();
		}

		result << get_dof_name(model.S[body_index]) << ", ";

		body_index = model.mu[body_index][0];
	}
	result << get_dof_name(model.S[body_index]);

	result << " ]" << endl;

	unsigned int child_index = 0;
	for (child_index = 0; child_index < model.mu[body_index].size(); child_index++) {
		result << print_hierarchy (model, model.mu[body_index][child_index], indent + 1);
	}

	// print fixed children
	for (unsigned int fbody_index = 0; fbody_index < model.mFixedBodies.size(); fbody_index++) {
		if (model.mFixedBodies[fbody_index].mMovableParent == body_index) {
			for (int j = 0; j < indent + 1; j++)
				result << "  ";

			result << model.GetBodyName(model.fixed_body_discriminator + fbody_index) << " [fixed]" << endl;
		}
	}


	return result.str();
}

RBDL_DLLAPI std::string GetModelHierarchy (const Model &model) {
	stringstream result ("");

	result << print_hierarchy (model);

	return result.str();
}

RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model) {
	stringstream result ("");

	VectorNd Q (VectorNd::Zero(model.dof_count));
	UpdateKinematicsCustom (model, &Q, NULL, NULL);

	for (unsigned int body_id = 0; body_id < model.mBodies.size(); body_id++) {
		std::string body_name = model.GetBodyName (body_id);

		if (body_name.size() == 0) 
			continue;

		Vector3d position = CalcBodyToBaseCoordinates (model, Q, body_id, Vector3d (0., 0., 0.), false);

		result << body_name << ": " << position.transpose() << endl;
	}

	return result.str();
}

}
}
