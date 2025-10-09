#pragma once
#include <MoleculeType.h>
#include <math/physics/RigidBody.h>
#include <array/arrayFunctions/arrayFunctions.h>
#include <math/graphics/color/color.h>
struct MolecularJoint;
struct Molecule : public RigidBody {
	using RigidBody::RigidBody;
	Molecule(MoleculeType type, cvec3& centerOfMass, cfp& mass, cfp& radius, ccolor& color, cvec3& velocity = vec3(), Quaternion rotation = Quaternion::identity()) : RigidBody(centerOfMass, mass, velocity, rotation), radius(radius), color(color), type(type) {

	}
	//true if this molecule has a joint determining it's second axis. when it hasn't, we can still rotate the joint over the up axis to find a second axis.
	int axesSet = 0;

	//to prevent velocities being 'passed around' when rigidbodies collide
	vec3 oldVelocity = vec3();
	color color{};
	std::mutex mutex{};
	std::vector<Molecule*> collidedWith{};
	bool shouldDelete = false;
	std::vector<MolecularJoint*> joints{};
	MoleculeType type;
	fp radius;
	fp getDensity() const;
	~Molecule();
};