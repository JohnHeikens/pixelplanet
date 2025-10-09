#pragma once
#include <math/graphics/texture.h>
struct PlanetBlueprint {
	std::wstring planetName = L"earth";
	bool hasAthmosphere = false;
	texture surface = texture(veci2(2,1));
	texture atmosphere = texture(veci2(2, 1));
	fp atmosphereTreshold = 0.9;
};