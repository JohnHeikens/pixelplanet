#pragma once
#include "RigidBody.h"
struct Molecule : public RigidBody {
	using RigidBody::RigidBody;
	Molecule(cvec3& centerOfMass, cfp& mass, ccolor& color, cvec3& velocity = vec3()): RigidBody(centerOfMass, mass), color(color), velocity(velocity) {

	}
	vec3 velocity = vec3();
	vec3 acceleration = vec3();
	color color{};
	//to prevent velocities being 'passed around' when molecules collide
	vec3 newVelocity = vec3();
	std::mutex mutex{};
	std::vector<Molecule*> collidedWith{};
	bool shouldDelete = false;
};