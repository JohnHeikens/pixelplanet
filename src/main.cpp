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
#include <TransparentSphereBrush.h>
#include <RayCastHit.h>
#include <octreeCollisions.h>
#include <PlanetBlueprint.h>
#include <fstream>
#include <filesystem/textfile.h>
#include <filesystem/jsonReader.h>

constexpr Square3 simulationBounds = Square3(vec3(), 0x1000);
constexpr int startingMoleculeCount = isDebugging ? 0x100 : 0x1000;
constexpr vec3 startingPlanetPos = vec3();
constexpr fp startingPlanetRadius = (fp)math::cbrt(startingMoleculeCount);
constexpr rectangle3 startingPlanetRect = rectangle3(startingPlanetPos, vec3()).expanded(startingPlanetRadius);
constexpr vec3 initialCameraPosition = vec3(0, -startingPlanetRect.size.getX(), 0);
constexpr fp simulationStep = 0.1;
constexpr fp gravitationalConstant = 0.2 / (fp)startingMoleculeCount;

BarnesHutTile mainTile = BarnesHutTile(simulationBounds);
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
constexpr vk PlacementModeKey = vk::Z;
constexpr vk AttachmentModeKey = vk::X;
constexpr vk RenderPlayerKey = vk::C;
sf::Music backgroundMusic = sf::Music(L"data/music/silent-universe.mp3");
sf::SoundBuffer popSoundBuffer = sf::SoundBuffer(L"data/sound/pop.mp3");

template<typename T>
constexpr auto toSF(vectn<T, 2> vector) {
	return sf::Vector2<T>(vector.x, vector.y);
}
template<typename T>
constexpr auto toSF(vectn<T, 3> vector) {
	return sf::Vector3<T>(vector.x, vector.y, vector.z);
}
template<typename T, fsize_t N>
constexpr auto toFloatSF(vectn<T, N> vector) {
	return toSF(vectn<float, N>(vector));
}

struct gameForm : public form
{
	videoWriter* writer{};
	colorPicker* picker = new colorPicker();
	textBox* planetNameTextBox = new textBox();
	Label* planetNameLabel = new Label(L"Planet Name:");
	Camera camera{};

	bool shift = false;
	bool playMode = true;

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
	sf::Sound popSound;
	PlanetBlueprint blueprint;
	gameForm() : popSound(popSoundBuffer)
	{
		backgroundMusic.setVolume(10.0f);
		backgroundMusic.play();
		backgroundMusic.setSpatializationEnabled(false);
		children = { picker, planetNameTextBox, planetNameLabel };
		resetSimulation();
	}
	inline void addMolecule(Molecule* m) {
		molecules.push_back(m);
	}
	void resetSimulation() {
		for (Molecule* m : molecules) {
			delete m;
		}
		mainTile.reset();
		molecules.clear();
		selectedMolecule = nullptr;
		vec3 pos1 = startingPlanetRect.pos1();
		//fill with molecules
			//the x y and z strides of stacked spheres (see documentation powerpoint)
		//constexpr vec3 stride{
		//	doubleMoleculeRadius,
		//	doubleMoleculeRadius * math::sqrt(0.75),
		//	doubleMoleculeRadius * math::sqrt((fp)2 / (fp)3)
		//};
		//constexpr vec2 zOffset = vec2{
		//	0.5,
		//	math::sqrt(0.75) / 3
		//} *doubleMoleculeRadius;
		//constexpr fp yOffset = 0.5 * doubleMoleculeRadius;
		//int yUneven = 0;
		//int zUneven = 0;
		//vec3 moleculePos{};
		//for (moleculePos.z = startingPlanetRect.pos0.z; moleculePos.z < pos1.z; moleculePos.z += stride.z, zUneven = 1 - zUneven) {
		//	for (moleculePos.y = startingPlanetRect.pos0.y + zUneven * zOffset.y; moleculePos.y < pos1.y; moleculePos.y += stride.y, yUneven = 1 - yUneven) {
		//		for (moleculePos.x = startingPlanetRect.pos0.x + math::mod(zUneven * zOffset.x + yUneven * yOffset, doubleMoleculeRadius); moleculePos.x < pos1.x; moleculePos.x += stride.x) {
		//			fp distanceFromCenter = moleculePos.length() / startingPlanetRadius;
		//			if (distanceFromCenter < 1) {
		//				MoleculeType type = distanceFromCenter < 0.7 ? MoleculeType::Stone : MoleculeType::Air;
		//				color moleculeColor = //type == MoleculeType::Air ?
		//					//colorPalette::blue:
		//					(color)hsv2rgb(colorf(moleculePos.normalized().getRotation() * math::radiansToDegrees, (moleculePos.z - startingPlanetRect.pos0.z) / startingPlanetRect.size.z, 1));
		//				moleculeColor.a() = type == MoleculeType::Air ? 0x40 : 0xff;
		//
		//				Molecule* m = new Molecule(type, moleculePos, type == MoleculeType::Air ? 0.1 : 1.0, moleculeColor);
		//				addMolecule(m);
		//			}
		//		}
		//	}
		//}
		stdPath planetPath = stdPath(L"data") / L"planet" / planetNameTextBox->text;
		stdPath testPath = planetPath / L"properties.json";
		std::ifstream f{ testPath };
		if (f.good()) {
			f.close();
			jsonContainer container = readJson(stringToWString(readAllText(testPath)));
			if (auto atmosphereChild = container.getChild(L"atmosphere")) {
				blueprint.hasAthmosphere = true;
				convertToDouble((*atmosphereChild)[L"treshold"].children[0].value, blueprint.atmosphereTreshold);
				if (auto textureChild = atmosphereChild->getChild(L"texture")) {
					blueprint.atmosphere = texture(planetPath / textureChild->children[0].value, true);
				}
			}
			else {
				blueprint.hasAthmosphere = false;
			}
			if (auto surfaceChild = container.getChild(L"surface")) {
				if (auto textureChild = surfaceChild->getChild(L"texture")) {
					blueprint.surface = texture(planetPath / textureChild->children[0].value, true);
				}
			}
			else {
				blueprint.hasAthmosphere = false;
			}
		}

		for (int i = 0; i < startingMoleculeCount; i++) {
			vec3 startingPos;
			fp distanceFromCenter;
			do {
				startingPos = getRandomPointInSphere(currentRandom, startingPlanetRect);
				distanceFromCenter = (startingPos - startingPlanetPos).length() / startingPlanetRadius;
				//if (distanceFromCenter < stoneRadius && distanceFromCenter > 0.7)
				//	continue;


				MoleculeType type = distanceFromCenter < blueprint.atmosphereTreshold ? MoleculeType::Stone : MoleculeType::Air;
				fp moleculeRadius = type == MoleculeType::Air ? 0.5 : (blueprint.atmosphereTreshold - distanceFromCenter) / blueprint.atmosphereTreshold * (startingPlanetRadius * 0.2) + 0.5;
				//not r ^ 3 * PI because it's relative, so it wouldn't matter
				fp moleculeVolume = moleculeRadius * moleculeRadius * moleculeRadius * 8;
				//to make bigger molecules pack less dense
				if (randFp(moleculeVolume) > 1)
					continue;

				cvec2& rotation = startingPos.normalized().getRotationVector() * math::radiansToDegrees;

				color moleculeColor = //type == MoleculeType::Air ?
					//colorPalette::blue:
					(color)hsv2rgb(colorf(startingPos.normalized().getRotation() * math::radiansToDegrees / 6, (startingPos.z - startingPlanetRect.pos0.z) / startingPlanetRect.size.z, 1));
				//planetTexture.getValue(floorVector(vec2(rotation.x < 0 ? rotation.x + 180 : rotation.x, rotation.y + 90)));
				moleculeColor.a() = type == MoleculeType::Air ? 0x40 : 0xff;

				fp density = type == MoleculeType::Air ? 0.125 : 1;
				fp moleculeMass = moleculeVolume * density;
				cvec3& startingVelocity = vec3();//vec3(startingPos.y * 0.001, -startingPos.x * 0.001, 0);

				Molecule* m = new Molecule(type, startingPos, moleculeMass, moleculeRadius, moleculeColor, startingVelocity);
				addMolecule(m);
				mainTile.AddBodyUnsafe(m);
				break;
				//buffer between air and ground so the air doesn't get launched away
			} while (true);
		}
		//std::vector<BarnesHutTile*> tiles = { &mainTile };
		//processTile(tiles, [](Molecule* m1, Molecule* m2) {
		//	if (!m1->shouldDelete && !m2->shouldDelete) {
		//		cfp& distanceSquared = (m1->centerOfMass - m2->centerOfMass).lengthSquared();
		//		if (distanceSquared < math::squared((m1->radius + m2->radius) * 0.9)) {
		//			//delete one of the colliding molecules
		//			(m1->radius > m2->radius ? m2 : m1)->shouldDelete = true;
		//			//m1->shouldDelete = true;
		//			//m2->shouldDelete = true;
		//		}
		//		else {
		//			for (const auto& joint : m2->joints) {
		//				if (joint->molecule == m1)return;
		//			}
		//			//Join(m1, m2);
		//		}
		//	}
		//
		//	}, 0);

		player = new Player(initialCameraPosition);
		camera = Camera(initialCameraPosition);
		addMolecule(player);
		molecules.update();

		//molecules.push_back(new Molecule(vec3(0, 0, 0), 1, colorPalette::blue));
		//molecules.push_back(new Molecule(vec3(1, 0, 0), 1, colorPalette::red));


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
		form::keyDown(keyCode);
		if (!focusedChild) {
			if (keyCode == vk::R) {
				resetSimulation();
			}
			else if (keyCode == vk::V) {
				startRecording();
			}
			else if (keyCode == vk::B) {
				//http://www.vendian.org/mncharity/dir3/planet_globes/
				//color by planet
				for (Molecule* m : molecules) {
					cvec2& rotation = (m->centerOfMass - mainTile.centerOfMass).normalized().getRotationVector();
					const texture& sampleFrom = m->type == MoleculeType::Air && blueprint.hasAthmosphere ? blueprint.atmosphere : blueprint.surface;
					veci2 pixelPos = floorVector(vec2(rotation.x < 0 ? rotation.x + math::PI2 : rotation.x, rotation.y + math::PI * 0.5) * (sampleFrom.size.y / math::PI));
					m->color = sampleFrom.getValue(pixelPos);
				}
			}
			else if (keyCode == PlacementModeKey) {
				placementMode = (PlacementMode)((((int)placementMode) + 1) % (int)PlacementMode::count);
			}
			else if (keyCode == AttachmentModeKey) {
				attachmentMode = (AttachmentMode)((((int)attachmentMode) + 1) % (int)AttachmentMode::count);
			}
			else if (keyCode == RenderPlayerKey) {
				playMode = !playMode;
			}
			else
				processKey(keyCode, 1);
		}
	}
	virtual void keyUp(cvk& keyCode) override
	{
		if (!focusedChild)
			processKey(keyCode, -1);
		form::keyUp(keyCode);
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
				vec3 localClickDirection = selectedMolecule->rotation.rotateInverse((exactIntersection - selectedMolecule->centerOfMass)).normalized();
				//the local normalized attachment direction vector. by default, it's just the place which got clicked
				vec3 attachmentDirection = localClickDirection;
				constexpr fp angleMargin = 0.1;
				//place and attach orthogonally, from the perspective of the molecules attached to.
				if (placementMode == PlacementMode::Orthogonal) {
					if (selectedMolecule->axesSet == 0 && attachmentMode != AttachmentMode::None) {
						//rotate m1 to have the joint at it's first axis
						vec3 currentUpAxis = selectedMolecule->rotation.rotate(axis1);
						adjustRotation(selectedMolecule, Quaternion::moveDirection(currentUpAxis, attachmentDirection));
						attachmentDirection = axis1;
					}
					else if (selectedMolecule->axesSet == 1 && attachmentMode != AttachmentMode::None) {
						//try to get a perpendicular axis
						fp angleDifference = angleBetween(localClickDirection, axis1);
						if (isColinear(angleDifference)) {
							//we cannot deduce a second axis from a colinear molecule.
							attachmentDirection = angleDifference < angleMargin ? axis1 : -axis1;
						}
						else if (isPerpendicular(angleDifference)) {
							//we found a second axis! this means our axes are now complete!
							vec3 secondAxis = makePerpendicular(localClickDirection, axis1);
							//rotate over x axis towards y axis (0 1 0)
							Quaternion moveOverXToY = Quaternion::axisAngle(axis1,
								//to rotate the y axis to the second axis
								vec2(secondAxis.y, secondAxis.z).getRotation()).normalized();
							adjustRotation(selectedMolecule, moveOverXToY);
							attachmentDirection = vec3(0, 1, 0);
						}
					}
					else if (selectedMolecule->axesSet == 2) {
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
				cfp& newMoleculeRadius = 0.5;
				cfp& doubleMoleculeRadius = selectedMolecule->radius + newMoleculeRadius;
				cvec3& newMoleculeCenter = selectedMolecule->centerOfMass + selectedMolecule->rotation.rotate(attachmentDirection) * doubleMoleculeRadius;
				Molecule* newMolecule = new Molecule(MoleculeType::Stone, newMoleculeCenter, 1, newMoleculeRadius, picker->currentColor, selectedMolecule->velocity, selectedMolecule->rotation);
				newMolecule->axesSet = selectedMolecule->axesSet;
				addMolecule(newMolecule);
				if (attachmentMode != AttachmentMode::None) {
					selectedMolecule->joints.push_back(new MolecularJoint{ newMolecule, attachmentDirection * doubleMoleculeRadius });
					//since the newMolecule has the same rotation as the old molecule, we just swap the attachment direction
					newMolecule->joints.push_back(new MolecularJoint{ selectedMolecule, -attachmentDirection * doubleMoleculeRadius });
				}
				//attach near molecules
				if (attachmentMode == AttachmentMode::Near) {
					constexpr fp joinRadius = 1.2;
					for (Molecule* otherMolecule : getNearMolecules(newMolecule, joinRadius, mainTile)) {
						if (otherMolecule != newMolecule && otherMolecule != selectedMolecule) {
							cvec3& positionDifference = otherMolecule->centerOfMass - newMolecule->centerOfMass;
							//check for relative position
							vec3 relativePosition = newMolecule->rotation.rotateInverse(positionDifference);
							cfp& distance = relativePosition.length();
							if (distance < joinRadius) {
								cvec3& otherMoleculeDirection = relativePosition / distance;
								if (placementMode == PlacementMode::Orthogonal) {
									vec3 orthogonalVector = getOrthogonalVector(newMolecule, relativePosition);
									if (orthogonalVector == vec3())
										Join(newMolecule, otherMolecule);
									else {
										vec3 otherOrthogonalVector = getOrthogonalVector(otherMolecule, -positionDifference);
										if (otherOrthogonalVector == vec3()) {
											Join(newMolecule, otherMolecule);
										}
										else {
											newMolecule->joints.push_back(new MolecularJoint{ otherMolecule, orthogonalVector * doubleMoleculeRadius });
											otherMolecule->joints.push_back(new MolecularJoint{ otherMolecule, orthogonalVector * doubleMoleculeRadius });
										}
									}

								}
								else {
									Join(newMolecule, otherMolecule);
								}
							}
						}
					}
				}
				popSound.setPosition({ (float)newMolecule->centerOfMass.x, (float)newMolecule->centerOfMass.y, (float)newMolecule->centerOfMass.z });
				popSound.setVelocity({ (float)newMolecule->velocity.x, (float)newMolecule->velocity.y, (float)newMolecule->velocity.z });
				popSound.setPitch(randFp<float>(0.9f, 1.1f));
				popSound.play();
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
		else if (keyCode == vk::LShift) {
			angularVelocity.y -= angleMultiplier;
		}
		else if (keyCode == vk::Space) {
			angularVelocity.y += angleMultiplier;
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
			if (m->type != MoleculeType::Air) {
				if (m != player) {
					fp t0, t1;
					if (collideraysphere(camera.position, rayDirection, Sphere(m->centerOfMass, m->radius), t0, t1)) {
						if (t0 < closestMoleculeDistance) {
							closestMoleculeDistance = t0;
							selectedMolecule = m;
						}
					}
				}
			}
		}
		if (selectedMolecule) {
			exactIntersection = camera.position + rayDirection * closestMoleculeDistance;
		}
	}

	inline static void processCollision(Molecule* m1, Molecule* m2) {
		//from m2 to m1. this makes a positive amount repel from m2
		vec3 positionDifference = m1->centerOfMass - m2->centerOfMass;
		cfp& distanceSquared = positionDifference.lengthSquared();

		//if (m1 == player || m2 == player) {
		//m1->color = colorPalette::purple;
		//m2->color = colorPalette::purple;
		//}
		if (distanceSquared > 0) {
			//make sure we're only colliding once
			m1->mutex.lock();
			//we don't have to add the collision to m1->collidedWith, since m1 only gets processed once this frame
			cbool& contains = arrayContains(m1->collidedWith, m2);
			m1->mutex.unlock();
			if (!contains) {
				m2->mutex.lock();
				m2->collidedWith.push_back(m1);
				cfp& distance = sqrt(distanceSquared);
				constexpr fp pushForceMultiplier = 0.1;
				//when a giant molecule touches a small molecule, it shouldn't be sent flying.
				cfp& overlapMultiplier = math::squared((m1->radius + m2->radius) - (distance));
				//desmos graph:
				// p = rho (density)
				//\left(\left(r_{1}+r_{2}\right)-\left(x\right)\right)^{2}\cdot\rho_{1}\cdot\rho_{2}\cdot0.1
				vec3 pushForce = (positionDifference / distance) * (overlapMultiplier * pushForceMultiplier);
				vec3 summedForce = pushForce;


				//when exchange = 1, all energy is preserved. wouldn't work when 3 molecules collide at the same time, as velocities could add up to a very high amount
				// therefore, let's put it at a low number.
				//push bodies apart
				constexpr fp exchangeMultiplier = 0.01;
				cvec3& forceExchange = (m2->oldVelocity * m2->getDensity() - m1->oldVelocity * m1->getDensity()) * (overlapMultiplier * exchangeMultiplier);
				summedForce += forceExchange;
				//if (vec3::dot(velocityDifference, m1->centerOfMass - m2->centerOfMass) < 0) {
					//summedForce += velocityDifference * exchange;

				//}

				//f * mOther * mSelf / mSelf = f * mOther
				m1->applyForce<ForceMode::Force>(summedForce);
				m2->applyForce<ForceMode::Force>(-summedForce);
				//m1->ApplyForce(v2, m2->centerOfMass);
				//m2->ApplyForce(v1, m1->centerOfMass);
				m2->mutex.unlock();
			}
		}
	}

	virtual void render(cveci2& position, const texture& renderTarget) override
	{
		array2d<fp> depthBuffer = array2d<fp>(renderTarget.size);
		depthBuffer.fill(INFINITY);
		player->velocity += camera.rotationTransform.multPointMatrix(acceleration);
		camera.follow(player->centerOfMass, player->velocity);
		sf::Listener::setPosition(toFloatSF(camera.position));
		sf::Listener::setDirection(toFloatSF(camera.rotationTransform.multPointMatrix(vec3(0, 1, 0))));
		sf::Listener::setUpVector(toFloatSF(camera.rotationTransform.multPointMatrix(vec3(0, 0, 1))));
		sf::Listener::setVelocity(toFloatSF(vecf3(player->velocity)));
		//cameraPosition += cameraVelocity;
		//cameraVelocity *= 0.99;

		constexpr fp framesPerFullRotation = 20.0f;
		cvec3& multipliedAngularVelocity = angularVelocity * (math::PI2 / framesPerFullRotation);
		//yaw
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(up, multipliedAngularVelocity.z));
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(forward, multipliedAngularVelocity.y));
		camera.rotationTransform = mat3x3::cross(camera.rotationTransform, mat3x3::rotate3d(right, multipliedAngularVelocity.x));

		mat4x4 test = mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1)));

		mainTile.reset();

		for (Molecule* const& m : molecules) {
			m->oldVelocity = m->velocity;
			m->centerOfMass += m->velocity * simulationStep + m->acceleration * math::calculateIterativeAddition(simulationStep);
			m->velocity += m->acceleration * simulationStep;
			m->rotation = m->rotation * m->angularVelocity;
			m->acceleration = vec3();
			//m->color = colorPalette::green;
			m->shouldDelete |= (!mainTile.inBounds(m->centerOfMass) && m != player);
			if (m->shouldDelete)
			{
				molecules.erase(&m);
				if (selectedMolecule == m)
					selectedMolecule = nullptr;
				delete m;
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
		molecules.update();
		//split in two groups: solid and transparent.
		std::sort(molecules.begin(), molecules.end(),
			//less function to sort ascending for solid and descending for transparent. we want to render the nearest solid spheres first, so we can cull a lot of pixels.
			[this](const Molecule* m1, const Molecule* m2) {

				cbool& solid1 = m1->color.a() == 0xff, solid2 = m2->color.a() == 0xff;

				if (solid1 != solid2)return solid1;
				bool leftCloser = (m1->centerOfMass - camera.position).lengthSquared() <
					(m2->centerOfMass - camera.position).lengthSquared();
				return solid1 ? leftCloser : !leftCloser;
			});
		mainTile.CalculateMassDistribution();
		renderTarget.fill(colorPalette::black);

		Molecule** mid = std::find_if(molecules.begin(), molecules.end(), [](Molecule* m) {
			return m->color.a() != 0xff;
			});
		mat4x4 worldToScreen = mat4x4::combine({
			mat4x4::fromRectToRect(rectangle3(vec3(-1, -1, 0), vec3(2, 2, 1)), rectangle3(vec3(), vec3(renderTarget.size.x, renderTarget.size.y, 1))),
			mat4x4::perspectiveFov(90 * math::degreesToRadians, renderTarget.size, 0.1, 0x1000),
			mat4x4::lookat(camera.position, camera.position + camera.rotationTransform.multPointMatrix(forward), camera.rotationTransform.multPointMatrix(up))
			});
		constexpr int groupCount = 2;
		Molecule** groupStart[groupCount] = { molecules.begin(), mid };
		Molecule** groupEnd[groupCount] = { mid, molecules.end() };
		for (int groupIndex = 0; groupIndex < groupCount; groupIndex++) {
			//render per group so transparency isn't rendered before solids
			std::for_each(std::execution::par_unseq, groupStart[groupIndex], groupEnd[groupIndex], [&worldToScreen, this, &renderTarget, &depthBuffer](Molecule*& m) {
				processCollidingTiles(m, mainTile, &processCollision);
				cvec3& gravityForce = mainTile.CalculateForce(m->centerOfMass, gravitationalConstant);
				m->applyForce<ForceMode::Acceleration>(gravityForce);


				//static fp powerMultiplier = 1 - std::pow(0.9, simulationStep);
				for (const MolecularJoint* joint : m->joints) {
					//try to get the other in a relatively correct position
					//
					cvec3& currentRelativePosition = (joint->molecule->centerOfMass - m->centerOfMass);
					vec3 adder = (m->rotation.rotate(joint->relativePosition) - currentRelativePosition) * 0.01;
					//always add double velocities so they cancel out!
					//TODO: force instead of velocity
					joint->molecule->applyForce<ForceMode::Acceleration>(adder);
					m->applyForce<ForceMode::Acceleration>(-adder);
				}
				if (playMode || m != player) {
					//vec4 multipliedPosition = worldToScreen.multPointMatrix<4>(m->centerOfMass);
					cfp& distanceFromScreen = (m->centerOfMass - camera.position).length();
					//multipliedPosition /= multipliedPosition.w;
					//if (multipliedPosition.z > -1 && multipliedPosition.z < 1) {
					solidBrush<fp> depthBrush{ distanceFromScreen };
					const Sphere sphere{ m->centerOfMass, m->radius };

					fillTransformedSphere(renderTarget, sphere, camera.position, camera.rotationTransform, 90 * math::degreesToRadians,
						TransparentSphereBrush(
							//top
							solidColorBrush(selectedMolecule == m ?
								lerpColor(m->color,
									//when it's a light color, it should lerp towards black. else white
									((m->color.r() + m->color.g() + m->color.b()) / 0x180) ?
									colorPalette::black :
									colorPalette::white, 0.2) :
								m->color), depthBrush,
							//bottom
							renderTarget, depthBuffer,
							sphere, camera.position, camera.rotationTransform, 90 * math::degreesToRadians
						));
					//fillTransformedLine(m->centerOfMass, m->centerOfMass + gravityForce * 60 * 0x10, worldToScreen, renderTarget, brushes::cyan);
				}
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
		}
		if (playMode) {
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
			std::wstring informationString = std::format(L"molecule count: {}\nposition:\t{:.2f},\t{:.2f},\t{:.2f}\nplacement mode:\t{}\nattachment mode:\t{}", molecules.size, player->centerOfMass.x, player->centerOfMass.y, player->centerOfMass.z, (int)placementMode, (int)attachmentMode);
			if (selectedMolecule) {
				informationString += std::format(L"\nselected molecule:\nposition:\t{:.2f},\t{:.2f},\t{:.2f}\nradius:{:.2f}\ncollided molecules:{}", selectedMolecule->centerOfMass.x, selectedMolecule->centerOfMass.y, selectedMolecule->centerOfMass.z, selectedMolecule->radius, selectedMolecule->collidedWith.size());
			}
			currentFont->DrawString(informationString, renderTarget.getClientRect(), renderTarget);
			renderChildren(position, renderTarget);
		}
		if (writer)
			writer->addFrame(renderTarget);
		updateSelection();

	}
};


void gameForm::layout(crectanglei2& newRect)
{
	control::layout(newRect);
	if (writer) {
		startRecording();
	}
	picker->layout(rectanglei2(0, 0, 0x200, 0x100));
	planetNameTextBox->layout(rectanglei2(0, 0x100, 0x200, 0x40));
	planetNameLabel->layout(rectanglei2(0, 0x140, 0x200, 0x40));
}

gameForm* mainForm = new gameForm();
int main(int argc, char* argv[])
{
	// execute this function before you do anything,
	initialize();
	return application(mainForm, gameName).run();
}