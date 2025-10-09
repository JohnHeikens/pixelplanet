#include <math/graphics/brush/brushes/DepthBufferBrush.h>
#include <math/sphere/sphere.h>
#include <math/sphere/sphereCollisions.h>
#include <math/vector/vectorfunctions.h>
#include <RayCastHit.h>
#include <math/random/shufflerandom.h>
#pragma once
ShuffleRandom<uint> shuffleRandom{ 0x1233 };
template<ValidBrush brush0Type, typename depthBrush0Type, ValidBrush brush1Type, typename depthBrush1Type>
struct TransparentSphereBrush : public DepthBufferBrush<brush0Type, depthBrush0Type, brush1Type, depthBrush1Type> {
	typedef DepthBufferBrush<brush0Type, depthBrush0Type, brush1Type, depthBrush1Type> base;

	vec3 relativeCameraPosition;
	mat3x3 cameraTransform;
	RotatedPixelOrientation orientation;
	fp c;
	const Sphere& sphere;
	constexpr TransparentSphereBrush(const brush0Type& brush0, const depthBrush0Type& depthBuffer0, const brush1Type& brush1, const depthBrush1Type& depthBuffer1,
		const Sphere& sphere, cvec3& cameraPosition, cmat3x3& cameraTransform, cfp& verticalFOV) :base(brush0, depthBuffer0, brush1, depthBuffer1), relativeCameraPosition((cameraPosition - sphere.center) / sphere.radius), orientation(brush1.size, verticalFOV, cameraTransform), sphere(sphere) {
		c = relativeCameraPosition.lengthSquared() - 1;
	}
	struct Iterator :RowIterator<TransparentSphereBrush> {
		typedef RowIterator<TransparentSphereBrush> base;
		vec3 rayDirection;
		Iterator(const TransparentSphereBrush& brush, cvect2<fsize_t>& pos) :base(brush, pos), rayDirection(brush.orientation.getRayDirection((vec2)pos)) {
		}
		constexpr void operator++() {
			base::operator++();
			rayDirection += base::brush.orientation.Step.x;
		}
		constexpr color operator*() {
			//cfp& depth = base::depthBuffer0.getValue(pos);
			cfp& a = rayDirection.lengthSquared();
			cfp& b = 2.0 * vec3::dot(base::brush.relativeCameraPosition, rayDirection);
			cfp& discriminant = b * b - 4 * a * base::brush.c;
			cfp& division2a = (2.0 * a);
			cfp& middle = -b / division2a;
			cfp& radiusatintersection = sqrt(discriminant) / division2a;
			fp tIn = middle - radiusatintersection;
			fp tOut = middle + radiusatintersection;
			fp depth = tIn * base::brush.sphere.radius;

			if (depth < base::brush.depthBuffer1.getValueUnsafe(base::position)) {
				color c = base::brush.brush0.getValue(base::position);
				if (c.a()) {
					//if (c.a() == color::maxValue) {

						//calculate normal
						vec3 intersectionPoint = base::brush.relativeCameraPosition + rayDirection * tIn;
						
						//fp multiplier = (intersectionPoint.z + 1) * 0.5;//based on z
						fp multiplier = (tOut - tIn) / 2.0;
						c.r() = (colorChannel)(c.r() * multiplier);
						c.g() = (colorChannel)(c.g() * multiplier);
						c.b() = (colorChannel)(c.b() * multiplier);
						//return color(c, 0xff);
						if (c.a() == color::maxValue)
						{
							base::brush.depthBuffer1.setValueUnsafe(base::position, depth);
							return c;
						}
						else {
							c.a() = (colorChannel)(c.a() * ((tOut - tIn) * 0.5));
							return transitionColor(c, base::brush.brush1.getValue(base::position));
						}
					//}
					//else {
					//	//add air to depthbuffer
					//	base::brush.depthBuffer1.setValueUnsafe(base::position, tOut - tIn);
					//}
				}
			}
			return base::brush.brush1.getValue(base::position);
		}
	};
	Iterator getIterator(cvect2<fsize_t>& pos) const {
		return Iterator(*this, pos);
	}
};

