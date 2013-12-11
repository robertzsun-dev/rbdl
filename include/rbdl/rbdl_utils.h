#ifndef _RBDL_UTILS_H
#define _RBDL_UTILS_H

#include <string>
#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace Utils {

	/** \brief Describes a Dof using the body name and transformation type */
	RBDL_DLLAPI std::string GetDofDescription (const Model &model, unsigned int dof_index);
	/** \brief Creates a human readable overview of the model. */
	RBDL_DLLAPI std::string GetModelHierarchy (const Model &model);
	/** \brief Creates a human readable overview of the Degrees of Freedom. */
	RBDL_DLLAPI std::string GetModelDofOverview (const Model &model);
	/** \brief Creates a human readable overview of the locations of all bodies that have names. */
	RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model);
}
}

/* _RBDL_UTILS_H */
#endif
