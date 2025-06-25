#pragma once
#include <math/mattnxn.h>
struct Camera {
	mat3x3 rotationTransform{};
	vec3 position{};
	fp smoothSpeed = 0.25;
	mat4x4 worldToScreen{};
	fp followDistance = 5;
	void follow(cvec3& positionToFollow, cvec3& speedToFollow) {
		vec3 lerpTowards = positionToFollow + rotationTransform.multPointMatrix(vec3(0, -followDistance, 0));
		this->position += speedToFollow;
		this->position = math::lerp(this->position, lerpTowards, smoothSpeed);
	}
};