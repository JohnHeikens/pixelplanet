#include <math/vector/vectn.h>
#include "Molecule.h"
#include <math/vector/vectorfunctions.h>
#include <math/rectangle/rectangletn.h>
#include "include/array/arrayFunctions/arrayFunctions.h"
#pragma once

struct GridContainer {
	std::unordered_map<veci3, fastList< Molecule*>> cells{};
	std::vector<Molecule*> toRemove = std::vector<Molecule*>();

	inline veci3 getCellCoordinates(Molecule* value) {
		return floorVector(value->centerOfMass);
	}

	inline void addValue(Molecule* value) {
		auto& cell = cells[getCellCoordinates(value)];
		cell.push_back(value);
		cell.update();
	}
	//only call removevalue after updatecells!
	inline void removeValue(Molecule* value) {
		cells[getCellCoordinates(value)].erase(value);
	}
	template<typename processFunction>
	inline void processNearCells(Molecule* value, fp radius, const processFunction& f) {
		veci3 minCellCoords = floorVector(value->centerOfMass - radius);
		veci3 maxCellCoords = ceilVector(value->centerOfMass + radius);
		for (veci3 pos : rectanglei3(minCellCoords, maxCellCoords - minCellCoords)) {
			const auto& cell = cells.find(pos);
			if (cell != cells.end()) {
				for (Molecule*& m : cell->second) {
					f(*m);
				}
			}
		}
	}
	template<typename DeleteFunctionType>
	inline void updateCells(DeleteFunctionType deleteFunction) {
		std::vector<std::pair<veci3, Molecule*>> movingMolecules = std::vector<std::pair<veci3, Molecule*>>();
		for (auto& cell : cells) {
			for (const auto& molecule : cell.second) {
				if (deleteFunction(molecule)) {
					cell.second.erase(&molecule);
				}
				else {
					cveci3& cellCoordinates = getCellCoordinates(molecule);
					if (cellCoordinates != cell.first) {
						//move
						movingMolecules.push_back(std::pair(cellCoordinates, molecule));
						cell.second.erase(&molecule);
					}
				}
			}
		}
		for (auto& movingMolecule : movingMolecules) {
			cells[movingMolecule.first].push_back(movingMolecule.second);
		}
		for (auto it = cells.begin(); it != cells.end();) {
			if (it->second.newSize) {
				it->second.update();
				it++;
			}
			else {
				it = cells.erase(it);
			}
		}
	}
};