#include <Molecule.h>
#pragma once
struct MolecularJoint {
	Molecule* molecule;
	vec3 relativePosition;
};
constexpr fp checkAngleMargin = math::fpepsilon;
constexpr fp angleMargin = 0.1;
constexpr vec3 axis1 = vec3(1, 0, 0);

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
//rotationDifference:
//the quaternion to multiply with to go from the old to the new rotation
static inline void setNewRotation(Molecule* m, Quaternion rotationDifference) {
	//first, increase axesSet to prevent infinite recursion
	m->axesSet++;
	for (MolecularJoint* joint : m->joints) {
		//then, find all quaternions who don't have this axis set (to prevent infinite recursion
		joint->relativePosition = rotationDifference.rotateInverse(joint->relativePosition);
		vec3 orthogonalAxis = getOrthogonalVector(joint->relativePosition.normalized());
		if (orthogonalAxis != vec3())
		{
			joint->relativePosition = orthogonalAxis * doubleMoleculeRadius;
			if (joint->molecule->axesSet < m->axesSet)
				setNewRotation(joint->molecule, rotationDifference);
		}
	}
	//rotationaldifference first, because we're rotating the internal joints
	m->rotation = m->rotation * rotationDifference;
}

static inline void Join(Molecule* mOld, Molecule* mNew, cvec3& m1Tom2) {
	mOld->joints.push_back(new MolecularJoint{ mNew, mOld->rotation.rotateInverse(m1Tom2) });
	mNew->joints.push_back(new MolecularJoint{ mOld, mNew->rotation.rotateInverse(-m1Tom2) });
}
static inline void Join(Molecule* m1, Molecule* m2) {
	Join(m1, m2, m2->centerOfMass - m1->centerOfMass);
}