#include <math/rectangle/rectangletn.h>
#include "RigidBody.h"
#include <math/physics/gravity/gravity.h>
/// <summary>
/// this gravity simulator makes use of the barnes-hut gravity algorithm
/// http://arborjs.org/docs/barnes-hut
/// </summary>
struct BarnesHutTile : public RigidBody
{
	static constexpr fp gravitationalConstant = 0.0001;
	static constexpr fp theta = 0.5;
	BarnesHutTile* children[8]{};
	crectangle3 bounds;
	int bodyCount = 0;
	inline BarnesHutTile(crectangle3& bounds = rectangle3()) :bounds(bounds)
	{
	}
	inline void AddQuadrant(RigidBody* body)
	{
		vec3 relative = body->centerOfMass - bounds.getCenter();
		int index = 0;

		vec3 childNodeSize = bounds.size * 0.5f;
		vec3 childNodePos = bounds.pos0;

		for (int i = 0; i < 3; i++)
		{
			if (relative[i] > 0)
			{
				childNodePos[i] += childNodeSize[i];
				index += 1 << i;
			}
		}

		if (children[index] == nullptr)
		{
			children[index] = new BarnesHutTile(rectangle3(childNodePos, childNodeSize));
		}
		children[index]->AddBodyUnsafe(body);

	}
	inline void AddBody(RigidBody* body) {
		if (bounds.contains(body->centerOfMass))
			AddBodyUnsafe(body);
	}
	inline void AddBodyUnsafe(RigidBody* body)
	{
		bodyCount++;
		if (bodyCount > 1)
		{
			if (bodyCount == 2)
			{
				if (this->centerOfMass == body->centerOfMass)
				{
					//simply dont add the molecule. the mass has been added.
					return;
				}
				else
				{
					//separate both molecules into different quadrants
					AddQuadrant(this);
					this->centerOfMass = vec3();
					this->mass = 0;
				}
			}
			AddQuadrant(body);
			//choose quadrant
		}
		else
		{
			this->centerOfMass = body->centerOfMass;
			this->mass = body->mass;
		}
	}
	inline void CalculateMassDistribution()
	{
		if (bodyCount > 1)
		{
			for (int i = 0; i < 0x8; i++)
			{
				if (children[i] != nullptr)
				{
					children[i]->CalculateMassDistribution();
					mass += children[i]->mass;
					centerOfMass += children[i]->mass * children[i]->centerOfMass;
				}
			}
			centerOfMass /= mass;
		}
	}
	inline vec3 CalculateForce(vec3 targetPos)
	{
		constexpr fp radius = 1;
		if (bodyCount == 1 ||
			//s / r < t
			//square everything
			//(s * s) / (r * r) < (t * t)
			math::squared(bounds.size.x) / (centerOfMass - targetPos).lengthSquared() < math::squared(theta)
			)
		{
			return calculateAcceleration(targetPos, centerOfMass, mass, gravitationalConstant, radius);
		}
		else
		{
			vec3 totalForce{};
			for (int i = 0; i < 0x8; i++)
			{
				if (children[i] != nullptr)
				{
					totalForce += children[i]->CalculateForce(targetPos);
				}
			}
			return totalForce;
		}
	}
	//static vec3 CalculateAcceleration(vec3 pos0, vec3 pos1, double m1, double gravityConstant)
	//{
	//	return calculateAcceleration(pos0, pos1, m1, gravityConstant, )
	//	return calculateGravityMass((pos1 - pos0).lengthSquared(), m1, gravityConstant, 1);
	//	//vec3 difference = pos1 - pos0;
	//	//double distanceSquared = difference.lengthSquared();
	//	//if (distanceSquared == 0)
	//	//{
	//	//	return vec3();
	//	//}
	//	////normal = difference / sqrt (distanceSquared)
	//	////normal = diff / d
	//	////return gravityConstant / distanceSquared * m1 * normal
	//	////replace for normal
	//	////g / d * d * m1 * norm
	//	////g / d * d * m1 * (diff / d)
	//	////diff * ((m1 * g) / (d * d * d))
	//	//return difference * ((m1 * gravityConstant) / (distanceSquared * std::sqrt(distanceSquared)));
	//}
	inline ~BarnesHutTile() {
		for (BarnesHutTile* child : children) {
			delete child;
		}
	}
	inline void clear() {
		for (BarnesHutTile*& child : children) {
			delete child;
			child = nullptr;
		}
		bodyCount = 0;
		centerOfMass = vec3();
		mass = 0;
	}
};