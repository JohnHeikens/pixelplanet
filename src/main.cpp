#include "application/initialize.h"
#include "application/control/form/form.h"
#include "application/application.h"
#include "gameInfo.h"
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
#include <Player.h>
#include <Camera.h>
#include <math/physics/gravity/BarnesHutTile.h>
#include <MolecularJoint.h>
#include <math/sphere/sphereCollisions.h>
#include <application/control/colorPicker.h>
#include <application/control/label.h>
#include <math/graphics/brush/font/baseFont.h>
#include <PlacementMode.h>

constexpr rectangle3 bounds = rectangle3().expanded(0x1000);
constexpr int startingMoleculeCount = isDebugging ? 0x10 : 0x1000;
constexpr vec3 startingPlanetPos = vec3();
constexpr fp startingPlanetRadius = (fp)math::cbrt(startingMoleculeCount) * 0.4;
constexpr rectangle3 startingPlanetRect = rectangle3(startingPlanetPos, vec3()).expanded(startingPlanetRadius);
constexpr vec3 initialCameraPosition = vec3(0, -startingPlanetRect.size.getX(), 0);
constexpr fp simulationStep = 1;

BarnesHutTile mainTile = BarnesHutTile(bounds);
fastList<Molecule*> molecules{};
std::mt19937 currentRandom = getRandomFromSeed(getmicroseconds());

Player* player = nullptr;

struct {
	//https://leanrada.com/notes/sweep-and-prune/
	//use this for sorting the molecules to optimize for collision detection
	inline bool operator ()(Molecule* m1, Molecule* m2) {
		return m1->centerOfMass.x < m2->centerOfMass.x;
	}
} moleculeSorter;
GridContainer closeMolecules{};

constexpr vk PlacementModeKey = vk::Z;
constexpr vk AttachmentModeKey = vk::X;

struct gameForm : public form
{
	videoWriter* writer{};
	colorPicker* picker = new colorPicker();
	Camera camera;

	bool shift = false;

	//relative to the player
	vec3 acceleration = vec3();
	vec3 angularVelocity = vec3();

	veci2 mousePos{};

	Molecule* selectedMolecule{};
	vec3 exactIntersection{};

	PlacementMode placementMode{};
	AttachmentMode attachmentMode{};

	//unchanging
	static constexpr vec3 forward = vec3(0, 1, 0);
	static constexpr vec3 up = vec3(0, 0, 1);
	static constexpr vec3 right = vec3(1, 0, 0);
	gameForm()
	{
		children = { picker };
		resetSimulation();
	}
	inline void addMolecule(Molecule* m) {
		molecules.push_back(m);
		closeMolecules.addValue(m);
	}
	void resetSimulation() {
		molecules.clear();
		closeMolecules = GridContainer();
		//fill with molecules
		for (int i = 0; i < startingMoleculeCount; i++) {
			vec3 startingPos = getRandomPointInSphere(currentRandom, startingPlanetRect);
			cfp& distanceFromCenter = (startingPos - startingPlanetPos).length() / startingPlanetRadius;
			MoleculeType type = distanceFromCenter < 0 ? MoleculeType::Stone : MoleculeType::Air;
			ccolor moleculeColor = type == MoleculeType::Air ?
				(color)hsv2rgb(colorf(startingPos.normalized().getRotation() * math::radiansToDegrees, (startingPos.z - startingPlanetRect.pos0.z) / startingPlanetRect.size.z, 1)) :
				colorPalette::brown;

			Molecule* m = new Molecule(type, startingPos, 1.0, moleculeColor);
			addMolecule(m);
		}
		delete player;
		player = new Player(initialCameraPosition);
		addMolecule(player);
		//create connections between molecules
		for (Molecule* m : molecules) {
			if (m->Type == MoleculeType::Stone) {
				closeMolecules.processNearCells(m, doubleMoleculeRadius, [m](Molecule& other) {
					if (other.Type == m->Type) {
						for (const auto& joint : other.joints) {
							if (joint->molecule == m)return;
						}
						Join(m, &other);
					}
					});
			}
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
		else if (keyCode == PlacementModeKey) {
			placementMode = (PlacementMode)((((int)placementMode) + 1) % (int)PlacementMode::count);
		}
		else if (keyCode == AttachmentModeKey) {
			attachmentMode = (AttachmentMode)((((int)attachmentMode) + 1) % (int)AttachmentMode::count);
		}
		else
			processKey(keyCode, 1);

	}
	virtual void keyUp(cvk& keyCode) override
	{
		processKey(keyCode, -1);
	}
	virtual void mouseMove(cveci2& position, cmb& button) override {
		control::mouseMove(position, button);
		mousePos = position;
	}
	virtual void mouseDown(cveci2& position, cmb& button) override {
		form::mouseDown(position, button);
		if (selectedMolecule) {
			if (button == mb::Right) {
				//closestMolecule->color = colorPalette::white;
				
				//the local normalized click direction
				vec3 localClickDirection = selectedMolecule->rotation.rotateInverse((exactIntersection - selectedMolecule->centerOfMass).normalized());
				//the local normalized attachment direction vector. by default, it's just the place which got clicked
				vec3 attachmentDirection = localClickDirection;
				constexpr fp angleMargin = 0.1;
				//place and attach orthogonally, from the perspective of the molecules attached to.
				if (placementMode == PlacementMode::Orthogonal && selectedMolecule->axesSet) {
					if (selectedMolecule->axesSet == 1) {
						//try to get a perpendicular axis
						fp angleDifference = angleBetween(localClickDirection, axis1);
						if (isColinear(angleDifference)) {
							//we cannot deduce a second axis from a colinear molecule.
							attachmentDirection = angleDifference < angleMargin ? axis1 : -axis1;
						}
						else if(isPerpendicular(angleDifference)) {
							//we found a second axis! this means our axes are now complete!
							attachmentDirection = makePerpendicular(localClickDirection, axis1);
						}
					}
					else if(selectedMolecule->axesSet == 2) {
						//since we already found both axes, focus on getting it orthogonal.
						attachmentDirection = getOrthogonalVector(localClickDirection, std::cos(angleMargin));
						if (attachmentDirection == vec3()) {
							//it's not orthogonal. just use the direction clicked.
							attachmentDirection = localClickDirection;
						}
					}
				}
				if (attachmentDirection == vec3()) {
					attachmentDirection = (exactIntersection - selectedMolecule->centerOfMass).normalized();
				}
				cvec3& newMoleculeCenter = selectedMolecule->centerOfMass + attachmentDirection * doubleMoleculeRadius;
				Molecule* newMolecule = new Molecule(MoleculeType::Stone, newMoleculeCenter, 1, picker->currentColor, player->velocity);
				addMolecule(newMolecule);
				if (attachmentMode != AttachmentMode::None) {
					Join(selectedMolecule, newMolecule);
					if (newMolecule->axesSet > selectedMolecule->axesSet) {
						//adopt same axes
						setNewRotation(selectedMolecule, Quaternion::between(selectedMolecule->rotation, newMolecule->rotation));
						selectedMolecule->axesSet = newMolecule->axesSet;
					}
					else if (selectedMolecule->axesSet == 0) {
						//rotate m1 to have the joint at it's first axis
						vec3 currentUpAxis = selectedMolecule->rotation.rotate(axis1);
						setNewRotation(selectedMolecule, Quaternion::moveDirection(currentUpAxis, attachmentDirection));
						vec3 newRelativePosition = selectedMolecule->rotation.rotate(axis1);
						vec3 newRelativePosition2 = newMolecule->rotation.rotate(-axis1);
					}
					else if (selectedMolecule->axesSet == 1) {
						//check if second axis can be set
						if (isPerpendicular(angleBetween(attachmentDirection, axis1))) {
							vec3 perpendicularAxis = makePerpendicular(attachmentDirection, axis1);
							//rotate over x axis towards y axis
							Quaternion rotation = Quaternion::axisAngle(axis1, vec2(perpendicularAxis.y, perpendicularAxis.z).getRotation());
							setNewRotation(selectedMolecule, selectedMolecule->rotation * rotation);
						}
					}
				}
				//attach near molecules
				if (attachmentMode == AttachmentMode::Near) {
					constexpr fp joinRadius = 1.2;
					closeMolecules.processNearCells(newMolecule, joinRadius, [newMolecule, &attachmentDirection, this](Molecule& otherMolecule) {
						//check for relative position
						vec3 relativePosition = newMolecule->rotation.rotateInverse(otherMolecule.centerOfMass - newMolecule->centerOfMass);
						cfp& distance = relativePosition.length();
						if (distance < joinRadius) {
							cvec3& otherMoleculeDirection = relativePosition / distance;
							if (placementMode == PlacementMode::Orthogonal) {
								vec3 orthogonalVector = getOrthogonalVector(otherMoleculeDirection, std::cos(angleMargin));
								if (orthogonalVector == vec3())
									orthogonalVector = otherMoleculeDirection;

							}
							else {
								Join(newMolecule, &otherMolecule);
							}
						}
						});
				}
			}
			else if (button == mb::Left) {
				selectedMolecule->shouldDelete = true;
			}
		}
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
	void updateSelection() {
		//raycast position
		cvec2 rectCenter = rectangle2(rect).getCenter();
		vec2 dirXY = (mousePos - rectCenter) / rectCenter.y;
		//not normalized! doesn't have to be
		vec3 rayDirection = camera.rotationTransform.multPointMatrix(vec3(dirXY.x, 1, dirXY.y));
		selectedMolecule = nullptr;
		fp closestMoleculeDistance = INFINITY;
		for (Molecule* m : molecules) {
			fp t0, t1;
			if (collideraysphere(camera.position, rayDirection, sphere(m->centerOfMass, moleculeRadius), t0, t1)) {
				if (t0 < closestMoleculeDistance) {
					closestMoleculeDistance = t0;
					selectedMolecule = m;
				}
			}
		}
		if (selectedMolecule) {
			exactIntersection = camera.position + rayDirection * closestMoleculeDistance;
		}
	}
	virtual void render(cveci2& position, const texture& renderTarget) override
	{
		array2d<fp> depthBuffer = array2d<fp>(renderTarget.size);
		depthBuffer.fill(INFINITY);
		player->newVelocity += camera.rotationTransform.multPointMatrix(acceleration);
		camera.follow(player->centerOfMass, player->velocity);
		//cameraPosition += cameraVelocity;
		//cameraVelocity *= 0.99;

		constexpr fp framesPerFullRotation = 20.0f;
		cvec3& multipliedAngularVelocity = angularVelocity * (math::PI2 / framesPerFullRotation);
		//yaw
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(up, multipliedAngularVelocity.z));
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(forward, multipliedAngularVelocity.y));
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(right, multipliedAngularVelocity.x));

		mat4x4 test = mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1)));
		mat4x4 worldToScreen = mat4x4::combine({
			mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1))),
			mat4x4::perspectiveFov(90 * math::degreesToRadians, renderTarget.size, 0.1, 0x1000),
			mat4x4::lookat(camera.position, player->centerOfMass, camera.rotationTransform.multPointMatrix(up))
			}
		);
		mainTile.clear();

		for (Molecule* const& m : molecules) {
			m->velocity = m->newVelocity;
			m->centerOfMass += m->velocity * simulationStep + m->acceleration * math::calculateIterativeAddition(simulationStep);
			m->velocity += m->acceleration * simulationStep;
			m->rotation = m->rotation * m->angularVelocity;
			//m->color = colorPalette::green;
			m->newVelocity = m->velocity;
			m->shouldDelete = m->shouldDelete || !mainTile.bounds.contains(m->centerOfMass);
			if (m->shouldDelete)
			{
				molecules.erase(&m);
				if (selectedMolecule == m)
					selectedMolecule = nullptr;
			}
			else
			{
				mainTile.AddBodyUnsafe(m);
				//if (m->collidedWith.size() > 0x10) {
				//	m->color = colorPalette::red;
				//}
				//else {
				//	m->color = colorPalette::white;
				//}
				m->collidedWith.clear();
				//m->color = colorPalette::green;
			}
		}
		closeMolecules.updateCells([](Molecule* m) {
			bool shouldDelete = m->shouldDelete;
			if (shouldDelete)
				delete m;
			return shouldDelete;
			});
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
							//m->ApplyForce(v2, otherMolecule.centerOfMass);
							//otherMolecule.ApplyForce(v1, m->centerOfMass);
						}
					}
				}
				});
			static fp powerMultiplier = 1 - std::pow(0.9, simulationStep);
			for (const MolecularJoint* joint : m->joints) {
				//try to get the other in a relatively correct position
				//
				cvec3& currentRelativePosition = (joint->molecule->centerOfMass - m->centerOfMass);
				vec3 adder = (m->rotation.rotate(joint->relativePosition) - currentRelativePosition) * powerMultiplier;
				//always add double velocities so they cancel out!
				joint->molecule->newVelocity += adder;
				m->newVelocity -= adder;
			}

			//vec4 multipliedPosition = worldToScreen.multPointMatrix<4>(m->centerOfMass);
			cfp& distanceFromScreen = (m->centerOfMass - camera.position).length();
			//multipliedPosition /= multipliedPosition.w;
			//if (multipliedPosition.z > -1 && multipliedPosition.z < 1) {
			solidBrush<fp, vect2<fsize_t>> depthBrush{ distanceFromScreen };

			fillTransformedSphere(renderTarget, sphere(m->centerOfMass, moleculeRadius), camera.position, camera.rotationTransform, 90 * math::degreesToRadians, DepthBufferBrush(
				//top
				solidColorBrush(selectedMolecule == m ?
					lerpColor(m->color,
						//when it's a light color, it should lerp towards black. else white
						((m->color.r() + m->color.g() + m->color.b()) / 0x180) ?
						colorPalette::black :
						colorPalette::white, 0.2) :
					m->color), depthBrush,
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
		//}
			});
		if (selectedMolecule) {
			renderAxes(mat4x4::combine({
				worldToScreen, //then to screen
				mat4x4::translate(selectedMolecule->centerOfMass), //then translate
				mat4x4::rotate(selectedMolecule->rotation)//first rotate
				}), renderTarget);
			constexpr color axisColors[] = {
	colorPalette::red,
	colorPalette::green,
	colorPalette::blue
			};
			//render axes
			for (int axis = 0; axis < 3; axis++) {
				vec3 directionVector = vec3();
				directionVector[axis] = 1;
				fillTransformedLine(selectedMolecule->centerOfMass, selectedMolecule->centerOfMass + selectedMolecule->rotation.rotate(directionVector), worldToScreen, renderTarget, solidColorBrush(axisColors[axis]));
			}
		}
		renderAxes(worldToScreen, renderTarget);
		//fillLine(renderTarget, vec2(0, renderTarget.size.y / 2), vec2(renderTarget.size.x, renderTarget.size.y / 2), brushes::green);
		//fillLine(renderTarget, vec2(renderTarget.size.x / 2, 0), vec2(renderTarget.size.x / 2, renderTarget.size.y), brushes::green);
		currentFont->DrawString(std::format(L"position:\t{:.2f},\t{:.2f},\t{:.2f}\nplacement mode:\t{}\nattachment mode:\t{}", player->centerOfMass.x, player->centerOfMass.y, player->centerOfMass.z, (int)placementMode, (int)attachmentMode), renderTarget.getClientRect(), renderTarget);
		renderChildren(position, renderTarget);
		if (writer)
			writer->addFrame(renderTarget);
		updateSelection();

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
	picker->layout(rectanglei2(0, 0, 0x200, 0x100));
}
