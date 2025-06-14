#include <math/vector/vectn.h>
#include "Molecule.h"
#include <math/vector/vectorfunctions.h>
#include <math/rectangle/rectangletn.h>
#include "include/array/arrayFunctions/arrayFunctions.h"
#pragma once
namespace std {
	template<>
	struct hash<veci3> {
		size_t operator()(const veci3& v) const noexcept {
			constexpr size_t prime = 92821ULL;
			return (size_t)v[0] + v[1] * prime + v[2] * (prime * prime);
		}
	};
};
struct GridContainer {
	std::unordered_map<veci3, std::vector< Molecule*>> cells{};

	inline veci3 getCell(Molecule* value) {
		return floorVector(value->centerOfMass);
	}

	inline void addValue(Molecule* value) {
		cells[getCell(value)].push_back(value);
	}
	inline std::vector<Molecule*> findNearMolecules(Molecule* value, int radius) {
		veci3 cell = getCell(value);
		std::vector<Molecule*> moleculesFound{};
		for (veci3 pos : rectanglei3(cell - veci3(radius), veci3(radius * 2 + 1))) {
			addArrayToArray(moleculesFound, cells[pos]);
		}
		return moleculesFound;
	}
};