#pragma once
#include "Molecule.h"
struct Player :Molecule {
	Player(cvec3& position) :Molecule(MoleculeType::Organic,position, 0.1, 0.5, colorPalette::purple){}
};