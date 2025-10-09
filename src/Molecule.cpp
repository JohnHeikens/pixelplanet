#include "stdafx.h"
#include "Molecule.h"
#include "MolecularJoint.h"
fp Molecule::getDensity() const
{
	return mass / (radius * radius * radius * 8);
}
Molecule::~Molecule() {
	for (MolecularJoint* joint : joints) {
		Molecule* otherMolecule = joint->molecule;
		for (auto it = otherMolecule->joints.begin(); it < otherMolecule->joints.end(); ) {
			if ((*it)->molecule == this) {
				delete* it;
				it = otherMolecule->joints.erase(it);
			}
			else it++;
		}
		delete joint;
	}
}