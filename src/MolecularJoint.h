#include <Molecule.h>
#pragma once
struct MolecularJoint {
	Molecule* molecule;
	vec3 relativePosition;
};
constexpr fp checkAngleMargin = math::fpepsilon;
constexpr fp angleMargin = 0.1;
//the axes at which a molecule will have to figure out to find it's orientation.
constexpr vec3 axis1 = vec3(1, 0, 0);
constexpr vec3 axis2 = vec3(0, 1, 0);

//potentialAxis has to be normalized!
//will return vec3() when the it's not orthogonal
constexpr vec3 getOrthogonalVector(cvec3& potentialAxis, cfp& maxDifference = math::fpepsilon) {
	for (int axis = 0; axis < 3; axis++) {
		vec3 axisVector{};
		axisVector[axis] = 1;
		fp difference = vec3::dot(potentialAxis, axisVector);
		if (difference > maxDifference) {
			if (difference > 1 - maxDifference) {
				return axisVector;
			}
			break;
		}
		else if (difference < -maxDifference) {
			if (difference < maxDifference - 1)
				axisVector[axis] = -1;
			return axisVector;
			break;
		}
	}
	return vec3();
}
//opposite or the same direction
constexpr bool isColinear(cfp& angleInRadians) {
	return (angleInRadians < angleMargin) || (angleInRadians > (math::PI * (1 - angleMargin)));
};
constexpr bool isPerpendicular(cfp& angleInRadians) {
	return angleInRadians > (math::PI * (0.5 - angleMargin)) && angleInRadians < (math::PI * (0.5 + angleMargin));
};

constexpr vec3 getOrthogonalVector(Molecule* m, vec3 difference) {
	if (m->axesSet < 2)return vec3();
	else return getOrthogonalVector(m->rotation.rotateInverse(difference).normalized(), std::acos(angleMargin));
}

//rotationDifference:
//the quaternion to multiply with to go from the old to the new rotation
static inline void adjustRotation(Molecule* m, Quaternion rotationDifference) {
	//first, increase axesSet to prevent infinite recursion
	m->axesSet++;
	for (MolecularJoint* joint : m->joints) {
		//then, find all quaternions who don't have this axis set (to prevent infinite recursion
		joint->relativePosition = rotationDifference.rotateInverse(joint->relativePosition);
		vec3 orthogonalAxis = getOrthogonalVector(joint->relativePosition.normalized());
		if (orthogonalAxis != vec3())
		{
			joint->relativePosition = orthogonalAxis * (m->radius + joint->molecule->radius);
			if (joint->molecule->axesSet < m->axesSet)
				adjustRotation(joint->molecule, rotationDifference);
		}
	}
	//rotationaldifference first, because we're rotating the internal joints
	m->rotation = m->rotation * rotationDifference;
}

static inline void Join(Molecule* m1, Molecule* m2, cvec3& m1Tom2) {
	m1->joints.push_back(new MolecularJoint{ m2, m1->rotation.rotateInverse(m1Tom2) });
	m2->joints.push_back(new MolecularJoint{ m1, m2->rotation.rotateInverse(-m1Tom2) });
}
static inline void Join(Molecule* m1, Molecule* m2) {
	Join(m1, m2, m2->centerOfMass - m1->centerOfMass);
}