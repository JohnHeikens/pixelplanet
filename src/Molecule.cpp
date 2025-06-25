#include "stdafx.h"
#include "Molecule.h"
#include "MolecularJoint.h"
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