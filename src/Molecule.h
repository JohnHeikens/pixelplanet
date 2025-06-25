#pragma once
#include <MoleculeType.h>
#include <math/physics/RigidBody.h>
#include <array/arrayFunctions/arrayFunctions.h>
#include <math/graphics/color/color.h>
constexpr fp moleculeRadius = 0.5;
constexpr fp doubleMoleculeRadius = moleculeRadius * 2;
struct MolecularJoint;
struct Molecule : public RigidBody {
	using RigidBody::RigidBody;
	Molecule(MoleculeType type, cvec3& centerOfMass, cfp& mass, ccolor& color, cvec3& velocity = vec3()) : RigidBody(centerOfMass, mass), color(color), velocity(velocity) {

	}
	//true if this molecule has a joint determining it's second axis. when it hasn't, we can still rotate the joint over the up axis to find a second axis.
	int axesSet = 0;
	vec3 velocity = vec3();
	vec3 acceleration = vec3();
	//to prevent velocities being 'passed around' when molecules collide
	vec3 newVelocity = vec3();

	color color{};
	std::mutex mutex{};
	std::vector<Molecule*> collidedWith{};
	bool shouldDelete = false;
	std::vector<MolecularJoint*> joints{};
	MoleculeType Type;
	~Molecule();
};