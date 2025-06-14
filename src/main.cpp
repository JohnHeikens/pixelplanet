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

constexpr rectangle3 bounds = rectangle3().expanded(0x1000);
constexpr int startingMoleculeCount = isDebugging ? 0x100 : 0x100;
BarnesHutTile mainTile = BarnesHutTile(bounds);
fastList<Molecule*> molecules{};
std::mt19937 currentRandom = getRandomFromSeed(getmicroseconds());
//constexpr fp targetRadius = 0.2;


//https://en.wikipedia.org/wiki/Lennard-Jones_potential
//epsilon = depth of the potential well energy scale
//sigma = the distance where the potential energy = 0 (no attraction or repulsion)
//+ = repulsion, - = attraction
static constexpr fp lennartJonesForce(cfp& distance, cfp& epsilon, cfp& sigma) {
	cfp& divided = sigma / distance;
	cfp& divided2 = divided * divided;
	cfp& divided6 = divided2 * divided2 * divided2;
	cfp& divided12 = divided6 * divided6;
	return 4 * epsilon * (2 * divided12 - divided6) / distance;
}

fp distancePart = 5.0 / 6.0;
fp b = -0.25;
fp squareMultiplier = 9.0;

static constexpr fp safeCollisionForce(cfp& distance) {
	return distance > 1 ? 0 : math::squared(distance - distancePart) * squareMultiplier - b;
}

//other arguments are filled in already
//proof till compression force of 0.3
static constexpr fp orbitForce(fp distance) {
	return safeCollisionForce(distance) * 0.1;
}

static constexpr fp runTests(cfp compressionForce, fp distance, fp acceleration, fp speed, fp interval) {
	//a constant compression force between the molecules, pulling them together
	for (int iteration = 0; iteration < 0x100 / interval; iteration++) {
		//*2 because both particles get drawn to eachother
		acceleration = compressionForce + orbitForce(distance) * 2;
		predictBehavior(acceleration, speed, distance, interval);
	}
	return distance;
}
constexpr fp formulaTest = runTests(0.05, 3, 0, 0, 1);
struct gameForm : public form
{
	videoWriter* writer{};
	Graph* lennartJonesForceGraph;
	//absolute
	vec3 cameraPosition = vec3(0, -0x200, 0);
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
	gameForm() : lennartJonesForceGraph(new Graph(orbitForce, rectangle2(0, -10, 20, 20)))
	{
		this->children = { lennartJonesForceGraph };
		resetSimulation();
	}
	void resetSimulation() {

		constexpr rectangle3 startingPlanetRect = rectangle3().expanded(0x10);
		molecules.clear();
		//fill with molecules
		for (int i = 0; i < startingMoleculeCount; i++) {
			vec3 startingPos = getRandomPointInSphere(currentRandom, startingPlanetRect);
			molecules.push_back(new Molecule(startingPos, 1, hsv2rgb(colorf(startingPos.normalized().getRotation() * math::radiansToDegrees, (startingPos.z - startingPlanetRect.pos0.z) / startingPlanetRect.size.z, 1))));
		}
		//molecules.push_back(new Molecule(vec3(0, 0, 0), 1, colorPalette::blue));
		//molecules.push_back(new Molecule(vec3(1, 0, 0), 1, colorPalette::red));


		molecules.update();
	}
	~gameForm() {
		delete writer;
	}
	virtual void layout(crectanglei2& newRect) override;
	void updateStream()
	{
	}
	virtual void keyDown(cvk& keyCode) override
	{
		fp mult = shift ? 1.1 : 1.0 / 1.1;
		if (keyCode == vk::Z) {
			distancePart *= mult;
		}
		else if (keyCode == vk::X) {
			squareMultiplier *= mult;
		}
		else if (keyCode == vk::C) {
			b += shift ? 0.1 : -0.1;
		}
		else if (keyCode == vk::R) {
			resetSimulation();
		}
		else
			processKey(keyCode, 1);

	}
	virtual void keyUp(cvk& keyCode) override
	{
		processKey(keyCode, -1);
	}
	void processKey(cvk& keyCode, fp mult) {
		if (keyCode == vk::W)
		{
			acceleration += forward * mult;
		}
		else if (keyCode == vk::S)
		{
			acceleration -= forward * mult;
		}
		else if (keyCode == vk::A)
		{
			acceleration -= right * mult;
		}
		else if (keyCode == vk::D)
		{
			acceleration += right * mult;
		}
		else if (keyCode == vk::Q)
		{
			acceleration -= up * mult;
		}
		else if (keyCode == vk::E)
		{
			acceleration += up * mult;
		}
		else if (keyCode == vk::Right) {
			angularVelocity.z += mult;
		}
		else if (keyCode == vk::Left) {
			angularVelocity.z -= mult;
		}
		else if (keyCode == vk::Up) {
			angularVelocity.x += mult;
		}
		else if (keyCode == vk::Down) {
			angularVelocity.x -= mult;
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

		GridContainer closeMolecules{};
		for (Molecule* const& m : molecules) {
			predictBehavior(m->acceleration, m->velocity, m->centerOfMass, simulationStep);
			if (!mainTile.bounds.contains(m->centerOfMass))
				molecules.erase(&m);
			else
			{
				mainTile.AddBodyUnsafe(m);
				closeMolecules.addValue(m);
			}
		}
		molecules.update();
		mainTile.CalculateMassDistribution();
		renderTarget.fill(colorPalette::black);
		lennartJonesForceGraph->dots.clear();
		for (Molecule* const& m : molecules) {
			m->acceleration = mainTile.CalculateForce(m->centerOfMass);
			m->color = colorPalette::red;
			for (Molecule* otherMolecule : closeMolecules.findNearMolecules(m, 1)) {
				if (otherMolecule != m) {
					m->color = colorPalette::green;
					//from othermolecule to m. this makes a positive amount repel from otherMolecule
					vec3 difference = m->centerOfMass - otherMolecule->centerOfMass;
					cfp& distanceSquared = difference.lengthSquared();

					if (distanceSquared < 1) {
						cfp& distance = sqrt(distanceSquared);
						cvec3& normal = difference / distance;
						cfp& lennartJonesForce = orbitForce(distance);
						lennartJonesForceGraph->dots.push_back(vec2(distance, lennartJonesForce));
						//lennard jones potential
						m->acceleration += lennartJonesForce * normal;
					}
				}
			}
			vec4 multipliedPosition = worldToScreen.multPointMatrix<4>(m->centerOfMass);
			cfp& distanceFromScreen = (m->centerOfMass - cameraPosition).length();
			multipliedPosition /= multipliedPosition.w;
			if (multipliedPosition.z > -1 && multipliedPosition.z < 1) {
				solidBrush<fp, vect2<fsize_t>> depthBrush{ distanceFromScreen };
				fillEllipseCentered(renderTarget, rectangle2(vec2(multipliedPosition), vec2()).expanded(renderTarget.size.x / distanceFromScreen),
					DepthBufferBrush(
						//top
						solidColorBrush(m->color), depthBrush,
						//bottom
						renderTarget, depthBuffer
					));
				//renderTarget.setValue(veci2(multipliedPosition), colorPalette::red);
			}
		}
		renderChildren(position, renderTarget);
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
	delete writer;
	writer = new videoWriter(newRect.size, 60, workingDirectory / ("video " + timeToString("{:%Y-%m-%d_%H-%M-%S}") + ".mp4"));
	control::layout(newRect);
	lennartJonesForceGraph->layout(rectanglei2(0, 0, 0x200, 0x100));
}
