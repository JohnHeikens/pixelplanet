#include <math/rectangle/rectangletn.h>
#include "RigidBody.h"
#include <math/physics/gravity/gravity.h>
/// <summary>
/// this gravity simulator makes use of the barnes-hut gravity algorithm
/// http://arborjs.org/docs/barnes-hut
/// </summary>
struct BarnesHutTile
{
	static constexpr fp gravitationalConstant = 0.001;
	static constexpr fp theta = 0.5;
	BarnesHutTile* children[8]{};
	crectangle3 bounds;
	vec3 centerOfMass = vec3();
	fp summedMass = 0;
	RigidBody* body = nullptr;
	int bodyCount = 0;
	inline BarnesHutTile(crectangle3& bounds = rectangle3()) :bounds(bounds)
	{
	}
	void AddQuadrant(RigidBody* body)
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
	void AddBody(RigidBody* body) {
		if (bounds.contains(body->centerOfMass))
			AddBodyUnsafe(body);
	}
	void AddBodyUnsafe(RigidBody* body)
	{
		bodyCount++;
		if (bodyCount > 1)
		{
			if (this->body != nullptr)
			{
				if (this->body->centerOfMass == body->centerOfMass)
				{
					//simply dont add the molecule. the mass has been added.
					return;
				}
				else
				{
					//separate both molecules into different quadrants
					AddQuadrant(this->body);
					this->body = nullptr;
				}
			}
			AddQuadrant(body);
			//choose quadrant
		}
		else
		{
			this->body = body;
		}
	}
	void CalculateMassDistribution()
	{
		if (body)
		{
			centerOfMass = body->centerOfMass;
			summedMass = body->mass;
		}
		else
		{
			for (int i = 0; i < 0x8; i++)
			{
				if (children[i] != nullptr)
				{
					children[i]->CalculateMassDistribution();
					summedMass += children[i]->summedMass;
					centerOfMass += children[i]->summedMass * children[i]->centerOfMass;
				}
			}
			centerOfMass /= summedMass;
		}
	}
	vec3 CalculateForce(vec3 targetPos)
	{
		constexpr fp radius = 1;
		if (bodyCount == 1)
		{
			return calculateAcceleration(targetPos, centerOfMass, summedMass, gravitationalConstant, radius);
		}
		else
		{
			//h / r < t
			//h / r * r < t * r
			//TODO: Optimize
			double rr = (centerOfMass - targetPos).lengthSquared();
			//s / r < t
			//square everything
			//(s * s) / (r * r) < (t * t)
			if (math::squared(bounds.size.x) / rr < math::squared(theta))
			{
				return calculateAcceleration(targetPos, centerOfMass, summedMass, gravitationalConstant, radius);
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
	~BarnesHutTile() {
		for (BarnesHutTile* child : children) {
			delete child;
		}
	}
	void clear() {
		for (BarnesHutTile*& child : children) {
			delete child;
			child = nullptr;
		}
		bodyCount = 0;
		body = nullptr;
	}
};