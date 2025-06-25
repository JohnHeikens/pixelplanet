#pragma once
#include "Molecule.h"
struct Player :Molecule {
	Player(cvec3& position) :Molecule(MoleculeType::Organic,position, 1, colorPalette::purple){}
};