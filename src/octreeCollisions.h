#pragma once
#include <Molecule.h>
#include <math/physics/gravity/BarnesHutTile.h>
#include <math/collisions.h>

typedef std::function<void(Molecule*, Molecule*)> collisionCallBackType;

inline void processCollidingTilesDownwards(Molecule& checkFor, const std::vector<BarnesHutTile*>& tiles, BarnesHutTile& currentTile, const collisionCallBackType& collisionCallBack, size_t recursion, cSquare3& checkBounds) {
	if (currentTile.bodies.size()) {
		for (const RigidBody* body : currentTile.bodies) {
			if ((checkFor.centerOfMass - body->centerOfMass).lengthSquared() < math::squared(checkFor.radius + ((Molecule*)body)->radius)
				&& body != &checkFor) {
				collisionCallBack(&checkFor, (Molecule*)body);
			}
		}
	}
	else
		for (BarnesHutTile* tile : currentTile.occupiedChildren) {
			if ((recursion + 1 >= tiles.size() || tile != tiles[recursion + 1])) {
				if (collides(checkBounds, tile->bounds)) {
					processCollidingTilesDownwards(checkFor, tiles, *tile, collisionCallBack, recursion + 1, checkBounds);
				}
			}
		}
}
inline void processCollidingTilesUpwards(Molecule& checkFor, const std::vector<BarnesHutTile*>& tiles, const collisionCallBackType& collisionCallBack, size_t recursion, cSquare3& checkBounds) {
	BarnesHutTile& currentTile = *tiles[recursion];
	processCollidingTilesDownwards(checkFor, tiles, currentTile, collisionCallBack, recursion, checkBounds);
	if (recursion && !contains(currentTile.bounds, checkBounds))
		processCollidingTilesUpwards(checkFor, tiles, collisionCallBack, recursion - 1, checkBounds);
}
inline void processTile(std::vector<BarnesHutTile*>& tiles, const collisionCallBackType& collisionCallBack, size_t recursion = 0) {
	BarnesHutTile& currentTile = *tiles[recursion];
	if (currentTile.bodies.size()) {
		for (const RigidBody* body : currentTile.bodies) {
			Molecule* m = (Molecule*)body;
			//check for collisions in a bounding box with radius r + r. this way, the biggest molecules will check for the smaller ones.
			cfp& checkRadius = m->radius + m->radius;
			cSquare3& checkBounds = Square3(m->centerOfMass, checkRadius);
			//check around. can't move this to begin of processCollidingTilesUpwards function, because currentTile has changed there
			//also check the current tile, because multiple bodies might be in here.
			processCollidingTilesUpwards(*m, tiles, collisionCallBack, recursion, checkBounds);
		}
	}
	else for (BarnesHutTile* child : currentTile.occupiedChildren) {
		tiles.push_back(child);
		processTile(tiles, collisionCallBack, recursion + 1);
		tiles.pop_back();
	}
};

//returns the path of barnes hut tiles that lead to the most specific tile at the place of the molecule. won't guarantee that the molecule is there, though.
inline std::vector<BarnesHutTile*> findMolecule(Molecule* searchFor, BarnesHutTile& mainTile) {
	std::vector<BarnesHutTile*> tilesTraversed = {  };
	int recursion = 0;
	BarnesHutTile* currentTile = &mainTile;
	while (currentTile) {
		tilesTraversed.push_back(currentTile);
		if (currentTile->bodies.size()) {
			return tilesTraversed;
		}
		else {

			currentTile = currentTile->children[currentTile->getChildIndex(searchFor)];
			recursion++;
		}
	}
}

inline std::vector<Molecule*> getNearMolecules(Molecule* searchAround, fp radius, BarnesHutTile& mainTile) {
	auto path = findMolecule(searchAround, mainTile);
	std::vector<Molecule*> moleculesFound;
	processCollidingTilesUpwards(*searchAround, path, [&moleculesFound, &radius](RigidBody* m1, RigidBody* m2) {
		if ((m1->centerOfMass - m1->centerOfMass).lengthSquared() < radius * radius) {
			moleculesFound.push_back((Molecule*)m2);
		}
		}, path.size() - 1ULL, Square3(searchAround->centerOfMass, radius));
	return moleculesFound;
}

inline void processCollidingTiles(Molecule* molecule, BarnesHutTile& mainTile, const collisionCallBackType& collisionCallBack) {
	auto path = findMolecule(molecule, mainTile);
	processCollidingTilesUpwards(*molecule, path, collisionCallBack, path.size() - 1ULL, Square3(molecule->centerOfMass, molecule->radius));

}