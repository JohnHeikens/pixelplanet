#pragma once
#include <math/vector/vectn.h>
struct RigidBody {
	vec3 centerOfMass;
	fp mass;
	RigidBody() : centerOfMass{}, mass{} {}
	RigidBody(cvec3& centerOfMass, cfp& mass) : centerOfMass(centerOfMass), mass(mass) {

	}
};