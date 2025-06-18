#include "application/initialize.h"
#include "application/control/form/form.h"
#include "application/application.h"
#include "gameInfo.h"
#include "RigidBody.h"
#include "BarnesHutTile1.h"
#include "Molecule.h"
#include "include/math/vector/vectorrandom.h"
#include "include/math/graphics/graphicsFunctions.h"
#include <GridContainer.h>
#include <math/physics.h>
#include <math/graphics/brush/brushes/DepthBufferBrush.h>
#include <application/control/Graph.h>
#include <math/graphics/video/videoWriter.h>
#include <execution>
#include <array/arrayFunctions/arrayFunctions.h>

constexpr rectangle3 bounds = rectangle3().expanded(0x1000);
constexpr int startingMoleculeCount = isDebugging ? 0x10 : 0x1000;
constexpr fp moleculeRadius = 0.5;
constexpr fp doubleMoleculeRadius = moleculeRadius * 2;
constexpr rectangle3 startingPlanetRect = rectangle3().expanded((fp)math::cbrt(startingMoleculeCount) * moleculeRadius);
constexpr vec3 initialCameraPosition = vec3(0, -startingPlanetRect.size.getX(), 0);

BarnesHutTile mainTile = BarnesHutTile(bounds);
fastList<Molecule*> molecules{};
std::mt19937 currentRandom = getRandomFromSeed(getmicroseconds());


struct {
	//https://leanrada.com/notes/sweep-and-prune/
	//use this for sorting the molecules to optimize for collision detection
	inline bool operator ()(Molecule* m1, Molecule* m2) {
		return m1->centerOfMass.x < m2->centerOfMass.x;
	}
} moleculeSorter;
GridContainer closeMolecules{};

struct gameForm : public form
{
	videoWriter* writer{};
	//absolute
	vec3 cameraPosition = initialCameraPosition;
	mat3x3 rotationTransform = mat3x3();
	vec3 cameraVelocity = vec3();
	bool shift = false;

	//relative to the camera
	vec3 acceleration = vec3();
	vec3 angularVelocity = vec3();
	//unchanging
	static constexpr vec3 forward = vec3(0, 1, 0);
	static constexpr vec3 up = vec3(0, 0, 1);
	static constexpr vec3 right = vec3(1, 0, 0);
	gameForm()
	{
		resetSimulation();
	}
	void resetSimulation() {
		molecules.clear();
		closeMolecules = GridContainer();
		//fill with molecules
		for (int i = 0; i < startingMoleculeCount; i++) {
			vec3 startingPos = getRandomPointInSphere(currentRandom, startingPlanetRect);
			Molecule* m = new Molecule(startingPos, 1, hsv2rgb(colorf(startingPos.normalized().getRotation() * math::radiansToDegrees, (startingPos.z - startingPlanetRect.pos0.z) / startingPlanetRect.size.z, 1)));
			molecules.push_back(m);
			closeMolecules.addValue(m);
		}


		//molecules.push_back(new Molecule(vec3(0, 0, 0), 1, colorPalette::blue));
		//molecules.push_back(new Molecule(vec3(1, 0, 0), 1, colorPalette::red));


		molecules.update();
	}
	~gameForm() {
		delete writer;
	}
	virtual void layout(crectanglei2& newRect) override;
	void startRecording() {
		delete writer;
		writer = new videoWriter(rect.size, 60, workingDirectory / ("video " + timeToString("{:%Y-%m-%d_%H-%M-%S}") + ".mp4"));
	}
	void updateStream()
	{
	}
	virtual void keyDown(cvk& keyCode) override
	{
		if (keyCode == vk::R) {
			resetSimulation();
		}
		else if (keyCode == vk::V) {
			startRecording();
		}
		else
			processKey(keyCode, 1);

	}
	virtual void keyUp(cvk& keyCode) override
	{
		processKey(keyCode, -1);
	}
	void processKey(cvk& keyCode, fp mult) {
		cfp& speedMultiplier = mult * 0.01;
		cfp& angleMultiplier = mult * 0.1;
		if (keyCode == vk::W)
		{
			acceleration += forward * speedMultiplier;
		}
		else if (keyCode == vk::S)
		{
			acceleration -= forward * speedMultiplier;
		}
		else if (keyCode == vk::A)
		{
			acceleration -= right * speedMultiplier;
		}
		else if (keyCode == vk::D)
		{
			acceleration += right * speedMultiplier;
		}
		else if (keyCode == vk::Q)
		{
			acceleration += up * speedMultiplier;
		}
		else if (keyCode == vk::E)
		{
			acceleration -= up * speedMultiplier;
		}
		else if (keyCode == vk::Right) {
			angularVelocity.z -= angleMultiplier;
		}
		else if (keyCode == vk::Left) {
			angularVelocity.z += angleMultiplier;
		}
		else if (keyCode == vk::Up) {
			angularVelocity.x += angleMultiplier;
		}
		else if (keyCode == vk::Down) {
			angularVelocity.x -= angleMultiplier;
		}
		else if (keyCode == vk::LShift || keyCode == vk::RShift) {
			shift = mult > 0;
		}
	}
	virtual void render(cveci2& position, const texture& renderTarget) override
	{
		array2d<fp> depthBuffer = array2d<fp>(renderTarget.size);
		depthBuffer.fill(INFINITY);
		constexpr fp simulationStep = 1;
		cameraVelocity += rotationTransform.multPointMatrix(acceleration);
		cameraPosition += cameraVelocity;
		cameraVelocity *= 0.99;

		constexpr fp framesPerFullRotation = 20.0f;
		cvec3& multipliedAngularVelocity = angularVelocity * (math::PI2 / framesPerFullRotation);
		//yaw
		rotationTransform = mat3x3::cross(rotationTransform, mat3x3::rotate3d(up, multipliedAngularVelocity.z));
		rotationTransform = mat3x3::cross(rotationTransform, mat3x3::rotate3d(forward, multipliedAngularVelocity.y));
		rotationTransform = mat3x3::cross(rotationTransform, mat3x3::rotate3d(right, multipliedAngularVelocity.x));

		mat4x4 test = mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1)));
		mat4x4 worldToScreen = mat4x4::combine({
			mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1))),
			mat4x4::perspectiveFov(90 * math::degreesToRadians, renderTarget.size, 0.1, 0x1000),
			mat4x4::lookat(cameraPosition, cameraPosition + rotationTransform.multPointMatrix(forward), rotationTransform.multPointMatrix(up))
			}
		);
		mainTile.clear();

		for (Molecule* const& m : molecules) {
			m->velocity = m->newVelocity;
			//m->color = colorPalette::green;
			predictBehavior(m->acceleration, m->velocity, m->centerOfMass, simulationStep);
			m->newVelocity = m->velocity;
			m->shouldDelete = !mainTile.bounds.contains(m->centerOfMass);
			if (m->shouldDelete)
			{
				molecules.erase(&m);
			}
			else
			{
				mainTile.AddBodyUnsafe(m);
				if (m->collidedWith.size() > 0x10) {
					m->color = colorPalette::red;
				}
				m->collidedWith.clear();
				//m->color = colorPalette::green;
			}
		}
		closeMolecules.updateCells([](Molecule* m) {return m->shouldDelete; });
		molecules.update();
		mainTile.CalculateMassDistribution();
		renderTarget.fill(colorPalette::black);

		std::for_each(std::execution::par_unseq, molecules.begin(), molecules.end(), [&worldToScreen, this, &renderTarget, &depthBuffer](Molecule*& m) {
			m->acceleration = mainTile.CalculateForce(m->centerOfMass);
			closeMolecules.processNearCells(m, doubleMoleculeRadius, [&m](Molecule& otherMolecule) {
				if (&otherMolecule != m) {
					//from othermolecule to m. this makes a positive amount repel from otherMolecule
					vec3 difference = m->centerOfMass - otherMolecule.centerOfMass;
					cfp& distanceSquared = difference.lengthSquared();

					if (distanceSquared < math::squared(doubleMoleculeRadius) && distanceSquared > 0) {
						//make sure we're only colliding once
						m->mutex.lock();
						//we don't have to add the collision to m->collidedWith, since m only gets processed once this frame
						cbool& contains = arrayContains(m->collidedWith, &otherMolecule);
						m->mutex.unlock();
						if (!contains) {
							otherMolecule.mutex.lock();
							otherMolecule.collidedWith.push_back(m);
							otherMolecule.mutex.unlock();
							//m->color = colorPalette::red;
							//otherMolecule.color = colorPalette::red;
							//just swap velocities for now. wouldn't work when 3 molecules collide at the same time
							//when exchange = 1, all energy is preserved.
							//push bodies apart
							fp exchange = 0.5;
							vec3 v1 = m->newVelocity;
							vec3 v2 = otherMolecule.newVelocity;
							cfp& distance = sqrt(distanceSquared);
							vec3 pushForce = (difference / distance) * math::squared(1 - distance) * 0.2;

							otherMolecule.newVelocity = v1 * exchange + v2 * (1 - exchange) - pushForce;
							m->newVelocity = v2 * exchange + v1 * (1 - exchange) + pushForce;
						}
					}
				}
				});


			vec4 multipliedPosition = worldToScreen.multPointMatrix<4>(m->centerOfMass);
			cfp& distanceFromScreen = (m->centerOfMass - cameraPosition).length();
			multipliedPosition /= multipliedPosition.w;
			if (multipliedPosition.z > -1 && multipliedPosition.z < 1) {
				solidBrush<fp, vect2<fsize_t>> depthBrush{ distanceFromScreen };
				fillTransformedSphere(renderTarget, sphere(m->centerOfMass, moleculeRadius), cameraPosition, rotationTransform, 90 * math::degreesToRadians, DepthBufferBrush(
					//top
					solidColorBrush(m->color), depthBrush,
					//bottom
					renderTarget, depthBuffer
				));
				//fillEllipse(renderTarget, rectangle2(vec2(multipliedPosition), vec2()).expanded(renderTarget.size.x / distanceFromScreen),
				//	DepthBufferBrush(
				//		//top
				//		solidColorBrush(m->color), depthBrush,
				//		//bottom
				//		renderTarget, depthBuffer
				//	));
				//renderTarget.setValue(veci2(multipliedPosition), colorPalette::red);
			}
			});
		//fillLine(renderTarget, vec2(0, renderTarget.size.y / 2), vec2(renderTarget.size.x, renderTarget.size.y / 2), brushes::green);
		//fillLine(renderTarget, vec2(renderTarget.size.x / 2, 0), vec2(renderTarget.size.x / 2, renderTarget.size.y), brushes::green);
		renderChildren(position, renderTarget);
		if (writer)
			writer->addFrame(renderTarget);
	}
};
gameForm* mainForm = new gameForm();
int main(int argc, char* argv[])
{
	// execute this function before you do anything,
	initialize();
	return application(mainForm, gameName).run();
}

void gameForm::layout(crectanglei2& newRect)
{
	if (writer) {
		startRecording();
	}
	control::layout(newRect);
}
