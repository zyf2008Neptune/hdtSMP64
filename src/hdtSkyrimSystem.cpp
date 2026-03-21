#include "hdtSkyrimSystem.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <locale>
#include <memory>
#include <numbers>
#include <string>
#include <unordered_map>
#include <utility>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btMinMax.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btTransformUtil.h>
#include <LinearMath/btVector3.h>
#include <RE/B/BSTSmartPointer.h>
#include <RE/N/NiAVObject.h>
#include <RE/N/NiNode.h>
#include <RE/N/NiPoint3.h>
#include <RE/N/NiSkinData.h>
#include <RE/N/NiSkinInstance.h>
#include <RE/N/NiSkinPartition.h>
#include <RE/P/PlayerCamera.h>
#include <RE/P/PlayerCharacter.h>
#include <RE/V/VertexDesc.h>
#include <SKSE/Logger.h>

#include "FrameworkUtils.h"
#include "NetImmerseUtils.h"
#include "PCH.h"
#include "XmlInspector/XmlInspector.hpp"
#include "XmlReader.h"
#include "hdtConvertNi.h"
#include "hdtDefaultBBP.h"
#include "hdtSkinnedMesh/hdtAABB.h"
#include "hdtSkinnedMesh/hdtBulletHelper.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtConstraintGroup.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkyrimBone.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace hdt
{
	// float32
	// Martin Kallman
	//
	// Fast half-precision to single-precision floating point conversion
	//  - Supports signed zero and denormals-as-zero (DAZ)
	//  - Does not support infinities or NaN
	//  - Few, partially pipelinable, non-branching instructions,
	//  - Core opreations ~6 clock cycles on modern x86-64
	static auto float32(float* __restrict out, const uint16_t in) -> void
	{
		uint32_t t1;
		uint32_t t2;
		uint32_t t3;

		t1 = in & 0x7fff; // Non-sign bits
		t2 = in & 0x8000; // Sign bit
		t3 = in & 0x7c00; // Exponent

		t1 <<= 13; // Align mantissa on MSB
		t2 <<= 16; // Shift sign bit into position

		t1 += 0x38000000; // Adjust bias

		t1 = (t3 == 0 ? 0 : t1); // Denormals-as-zero

		t1 |= t2; // Re-insert sign bit

		*(reinterpret_cast<uint32_t*>(out)) = t1;
	};

	static constexpr auto PI = std::numbers::pi_v<float>;

	btEmptyShape SkyrimSystemCreator::BoneTemplate::emptyShape[1];

	auto SkyrimSystem::findBone(IDStr name) -> SkinnedMeshBone*
	{
		for (const auto& i : m_bones) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	auto SkyrimSystem::findBody(IDStr name) -> SkinnedMeshBody*
	{
		for (const auto& i : m_meshes) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	auto SkyrimSystem::findBoneIdx(IDStr name) -> int
	{
		for (auto i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_name == name) {
				return i;
			}
		}

		return -1;
	}

	SkyrimSystem::SkyrimSystem(RE::NiNode* skeleton) :
		m_skeleton(skeleton), m_oldRoot(nullptr)
	{
		m_oldRoot = m_skeleton;
	}

	auto SkyrimSystem::readTransform(float timeStep) -> void
	{
		auto newRoot = m_skeleton.get();
		while (newRoot->parent) {
			newRoot = newRoot->parent;
		}

		if (m_oldRoot != newRoot) {
			timeStep = RESET_PHYSICS;
		}

		if (!m_initialized) {
			timeStep = RESET_PHYSICS;
			m_initialized = true;
		}

		if (timeStep <= RESET_PHYSICS) {
			if (!this->block_resetting) {
				updateTransformUpDown(m_skeleton.get(), true);
			}

			m_lastRootRotation = convertNi(m_skeleton->world.rotate);
		} else if (m_skeleton->parent == RE::PlayerCharacter::GetSingleton()->Get3D2()) {
			if (SkyrimPhysicsWorld::get()->m_resetPc > 0) {
				timeStep = RESET_PHYSICS;
				updateTransformUpDown(m_skeleton.get(), true);
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
				SkyrimPhysicsWorld::get()->m_resetPc -= 1;
			} else if (
				!RE::PlayerCamera::GetSingleton()->GetRuntimeData2().isWeapSheathed ||
				RE::PlayerCamera::GetSingleton()->currentState->id ==
				RE::CameraState::
				kFirstPerson) // isWeaponSheathed or potentially isCameraFree || cameraState is first person
			{
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
			} else {
				btQuaternion newRot = convertNi(m_skeleton->world.rotate);
				btVector3 rotAxis;
				float rotAngle;
				btTransformUtil::calculateDiffAxisAngleQuaternion(m_lastRootRotation, newRot, rotAxis, rotAngle);

				if (SkyrimPhysicsWorld::get()->m_clampRotations) {
					float limit = SkyrimPhysicsWorld::get()->m_rotationSpeedLimit * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						rotAngle = btClamped(rotAngle, -limit, limit);
						btQuaternion clampedRot(rotAxis, rotAngle);
						m_lastRootRotation = clampedRot * m_lastRootRotation;
						m_skeleton->world.rotate = convertBt(m_lastRootRotation);

						const auto& children = m_skeleton->GetChildren();
						for (uint16_t i = 0; i < children.size(); ++i) {
							auto node = castNiNode(children[i].get());
							if (node) {
								updateTransformUpDown(node, true);
							}
						}
					}
				} else if (SkyrimPhysicsWorld::get()->m_unclampedResets) {
					float limit = SkyrimPhysicsWorld::get()->m_unclampedResetAngle * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						timeStep = RESET_PHYSICS;
						updateTransformUpDown(m_skeleton.get(), true);
						m_lastRootRotation = convertNi(m_skeleton->world.rotate);
					}
				}
			}
		}

		SkinnedMeshSystem::readTransform(timeStep);
		m_oldRoot = hdt::make_nismart(newRoot);
	}

	auto SkyrimSystem::writeTransform() -> void { SkinnedMeshSystem::writeTransform(); }

	auto SkyrimSystemCreator::findObjectByName(const IDStr& name) -> RE::NiNode*
	{
		// TODO check it's not a lurker skeleton
		return findNode(m_skeleton, name->cstr());
	}

	auto SkyrimSystemCreator::getOrCreateBone(const IDStr& name) -> SkyrimBone*
	{
		auto bone = dynamic_cast<SkyrimBone*>(m_mesh->findBone(getRenamedBone(name)));
		if (bone) {
			return bone;
		}

		logger::warn("Bone {} used before being created, trying to create it with current default values",
			name->cstr());
		return createBoneFromNodeName(name);
	}

	auto SkyrimSystemCreator::getRenamedBone(IDStr name) -> IDStr
	{
		auto iter = m_renameMap.find(name);
		if (iter != m_renameMap.end()) {
			return iter->second;
		}
		return name;
	}

	auto SkyrimSystemCreator::createOrUpdateSystem(RE::NiNode* skeleton,
		RE::NiAVObject* model, DefaultBBP::PhysicsFile_t* file, std::unordered_map<IDStr, IDStr>&& renameMap,
		SkyrimSystem* old_system) -> RE::BSTSmartPointer<SkyrimSystem>
	{
		const auto& path = file->first;
		if (path.empty()) {
			return nullptr;
		}

		const auto& loaded = readAllFile(path.c_str());
		if (loaded.empty()) {
			return nullptr;
		}

		m_renameMap = std::move(renameMap);
		m_skeleton = skeleton;
		m_model = model;
		m_filePath = path;

		if (!old_system) {
			updateTransformUpDown(m_skeleton, true);
		}

		XMLReader reader(reinterpret_cast<uint8_t*>(const_cast<char*>(loaded.data())), loaded.size());
		m_reader = std::addressof(reader);

		m_reader->nextStartElement();
		if (m_reader->GetName() != "system") {
			return nullptr;
		}

		auto& meshNameMap = file->second;

		m_mesh = RE::make_smart<SkyrimSystem>(skeleton);

		// Store original locale
		auto save_locale = std::locale();

		// Set locale to en_US
		std::locale::global(std::locale("en_US"));

		try {
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "bone") {
						readOrUpdateBone(old_system);
					} else if (name == "bone-default") {
						const auto& clsname = m_reader->getAttribute("name", "");
						const auto& extends = m_reader->getAttribute("extends", "");
						auto defaultBoneInfo = getBoneTemplate(extends);
						readBoneTemplate(defaultBoneInfo);
						m_boneTemplates[clsname] = defaultBoneInfo;
					} else if (name == "per-vertex-shape") {
						auto shape = readPerVertexShape(meshNameMap);
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.emplace_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "per-triangle-shape") {
						auto shape = readPerTriangleShape(std::addressof(meshNameMap));
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.emplace_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "constraint-group") {
						auto constraint = readConstraintGroup();
						if (constraint) {
							m_mesh->m_constraintGroups.push_back(constraint);
						}
					} else if (name == "generic-constraint") {
						auto constraint = readGenericConstraint();
						if (constraint) {
							m_mesh->m_constraints.emplace_back(constraint);
						}
					} else if (name == "stiffspring-constraint") {
						auto constraint = readStiffSpringConstraint();
						if (constraint) {
							m_mesh->m_constraints.emplace_back(constraint);
						}
					} else if (name == "conetwist-constraint") {
						auto constraint = readConeTwistConstraint();
						if (constraint) {
							m_mesh->m_constraints.emplace_back(constraint);
						}
					} else if (name == "generic-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
						readGenericConstraintTemplate(defaultGenericConstraintTemplate);
						m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
					} else if (name == "stiffspring-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
						readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
						m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
					} else if (name == "conetwist-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
						readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
						m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
					} else if (name == "shape") {
						auto attrName = m_reader->getAttribute("name");
						auto shape = readShape();
						if (shape) {
							m_shapeRefs.push_back(shape);
							m_shapes.insert(std::make_pair(attrName, shape));
						}
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
		} catch (const std::string& err) {
			logger::error("xml parse error - {}", err.c_str());
			return nullptr;
		}

		// Restore original locale
		std::locale (saved_locale);

		if (m_reader->GetErrorCode() != Xml::ErrorCode::None) {
			logger::error("xml parse error - {}", m_reader->GetErrorMessage());
			return nullptr;
		}

		m_mesh->m_skeleton = hdt::make_nismart(m_skeleton);
		m_mesh->m_shapeRefs.swap(m_shapeRefs);
		std::sort(m_mesh->m_bones.begin(), m_mesh->m_bones.end(), [](const auto& a, const auto& b) {
			return static_cast<SkyrimBone*>(a.get())->m_depth < static_cast<SkyrimBone*>(b.get())->m_depth;
		});

		return m_mesh->valid() ? m_mesh : nullptr;
	}

	auto SkyrimSystemCreator::readConstraintGroup() -> RE::BSTSmartPointer<ConstraintGroup>
	{
		RE::BSTSmartPointer<ConstraintGroup> ret = RE::make_smart<ConstraintGroup>();

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();

				if (name == "generic-constraint") {
					auto constraint = readGenericConstraint();
					if (constraint) {
						ret->m_constraints.emplace_back(constraint);
					}
				} else if (name == "stiffspring-constraint") {
					auto constraint = readStiffSpringConstraint();
					if (constraint) {
						ret->m_constraints.emplace_back(constraint);
					}
				} else if (name == "conetwist-constraint") {
					auto constraint = readConeTwistConstraint();
					if (constraint) {
						ret->m_constraints.emplace_back(constraint);
					}
				} else if (name == "generic-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
					readGenericConstraintTemplate(defaultGenericConstraintTemplate);
					m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
				} else if (name == "stiffspring-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
					readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
					m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
				} else if (name == "conetwist-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
					readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
					m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
		return ret;
	}

	auto SkyrimSystemCreator::readBoneTemplate(BoneTemplate& cinfo) -> void
	{
		auto clearCollide = true;
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();
				if (name == "mass") {
					cinfo.m_mass = m_reader->readFloat();
				} else if (name == "inertia") {
					cinfo.m_localInertia = m_reader->readVector3();
				} else if (name == "centerOfMassTransform") {
					cinfo.m_centerOfMassTransform = m_reader->readTransform();
				} else if (name == "linearDamping") {
					cinfo.m_linearDamping = m_reader->readFloat();
				} else if (name == "angularDamping") {
					cinfo.m_angularDamping = m_reader->readFloat();
				} else if (name == "friction") {
					cinfo.m_friction = m_reader->readFloat();
				} else if (name == "rollingFriction") {
					cinfo.m_rollingFriction = m_reader->readFloat();
				} else if (name == "restitution") {
					cinfo.m_restitution = m_reader->readFloat();
				} else if (name == "margin-multiplier") {
					cinfo.m_marginMultipler = m_reader->readFloat();
				} else if (name == "shape") {
					auto shape = readShape();
					if (shape) {
						m_shapeRefs.push_back(shape);
						cinfo.m_collisionShape = shape.get();
					} else {
						cinfo.m_collisionShape = BoneTemplate::emptyShape;
					}
				} else if (name == "collision-filter") {
					cinfo.m_collisionFilter = m_reader->readInt();
				} else if (name == "can-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_canCollideWithBone.emplace_back(m_reader->readText());
				} else if (name == "no-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_noCollideWithBone.emplace_back(m_reader->readText());
				} else if (name == "gravity-factor") {
					cinfo.m_gravityFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
	}

	auto SkyrimSystemCreator::readShape() -> std::shared_ptr<btCollisionShape>
	{
		auto typeStr = m_reader->getAttribute("type");
		if (typeStr == "ref") {
			auto shapeName = m_reader->getAttribute("name");
			m_reader->skipCurrentElement();
			auto iter = m_shapes.find(shapeName);
			if (iter != m_shapes.end()) {
				return iter->second;
			}
			logger::warn("unknown shape - {}", shapeName.c_str());
			return nullptr;
		}
		if (typeStr == "box") {
			btVector3 halfExtend(0, 0, 0);
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "halfExtend") {
						halfExtend = m_reader->readVector3();
					} else if (name == "margin") {
						margin = m_reader->readFloat();
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
			auto ret = std::make_shared<btBoxShape>(halfExtend);
			ret->setMargin(margin);
			return ret;
		}
		if (typeStr == "sphere") {
			float radius = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "radius") {
						radius = m_reader->readFloat();
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
			return std::make_shared<btSphereShape>(radius);
		}
		if (typeStr == "capsule") {
			float radius = 0;
			float height = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "radius") {
						radius = m_reader->readFloat();
					} else if (name == "height") {
						height = m_reader->readFloat();
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
			return std::make_shared<btCapsuleShape>(radius, height);
		}
		if (typeStr == "hull") {
			float margin = 0;
			auto ret = std::make_shared<btConvexHullShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "point") {
						ret->addPoint(m_reader->readVector3(), false);
					} else if (name == "margin") {
						margin = m_reader->readFloat();
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
			ret->recalcLocalAabb();
			return ret->getNumPoints() ? ret : nullptr;
		}
		if (typeStr == "cylinder") {
			float height = 0;
			float radius = 0;
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto& name = m_reader->GetName();
					if (name == "height") {
						height = m_reader->readFloat();
					} else if (name == "radius") {
						radius = m_reader->readFloat();
					} else if (name == "margin") {
						margin = m_reader->readFloat();
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}

			if (radius >= 0 && height >= 0) {
				auto ret = std::make_shared<btCylinderShape>(btVector3(radius, height, radius));
				ret->setMargin(margin);
				return ret;
			}
			return nullptr;
		}
		if (typeStr == "compound") {
			auto ret = std::make_shared<btCompoundShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					if (m_reader->GetName() == "child") {
						btTransform tr;
						std::shared_ptr<btCollisionShape> shape;

						while (m_reader->Inspect()) {
							if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
								if (m_reader->GetName() == "transform") {
									tr = m_reader->readTransform();
								} else if (m_reader->GetName() == "shape") {
									shape = readShape();
								} else {
									logger::warn("unknown element - {}", m_reader->GetName().c_str());
									m_reader->skipCurrentElement();
								}
							} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
								break;
							}
						}

						if (shape) {
							ret->addChildShape(tr, shape.get());
							m_shapeRefs.push_back(shape);
						}
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
					break;
				}
			}
			return ret->getNumChildShapes() ? ret : nullptr;
		}
		logger::warn("Unknown shape type {}", typeStr.c_str());
		return nullptr;
	}

	auto SkyrimSystemCreator::readOrUpdateBone(SkyrimSystem* old_system) -> void
	{
		IDStr name = getRenamedBone(m_reader->getAttribute("name"));
		if (m_mesh->findBone(name)) {
			logger::warn("Bone {} already exists, skipped", name->cstr());
			return;
		}

		IDStr cls = m_reader->getAttribute("template", "");
		if (!createBoneFromNodeName(name, cls, true, old_system)) {
			m_reader->skipCurrentElement();
		}
	}

	auto SkyrimSystemCreator::createBoneFromNodeName(const IDStr& bodyName, const IDStr& templateName,
		const bool readTemplate, SkyrimSystem* old_system) -> SkyrimBone*
	{
		auto node = findObjectByName(bodyName);
		if (node) {
			logger::info("Found node named {}, creating bone", bodyName->cstr());
			auto boneTemplate = getBoneTemplate(templateName);
			if (readTemplate) {
				readBoneTemplate(boneTemplate);
			}
			auto bone = new SkyrimBone(node->name.c_str(), node, this->m_skeleton, boneTemplate);
			bone->m_localToRig = boneTemplate.m_centerOfMassTransform;
			bone->m_rigToLocal = boneTemplate.m_centerOfMassTransform.inverse();
			bone->m_marginMultipler = boneTemplate.m_marginMultipler;
			bone->m_gravityFactor = boneTemplate.m_gravityFactor;

			if (old_system) {
				auto old_b = old_system->findBone(bodyName);
				if (old_b) {
					bone->m_currentTransform = convertNi(bone->m_skeleton->world) * old_b->m_origToSkeletonTransform;
					auto dest = bone->m_currentTransform.asTransform() * bone->m_localToRig;
					bone->m_origToSkeletonTransform = old_b->m_origToSkeletonTransform;
					bone->m_origTransform = old_b->m_origTransform;
					bone->m_rig.setWorldTransform(dest);
					bone->m_rig.setInterpolationWorldTransform(dest);
					bone->m_rig.setLinearVelocity(btVector3(0, 0, 0));
					bone->m_rig.setAngularVelocity(btVector3(0, 0, 0));
					bone->m_rig.setInterpolationLinearVelocity(btVector3(0, 0, 0));
					bone->m_rig.setInterpolationAngularVelocity(btVector3(0, 0, 0));
					bone->m_rig.updateInertiaTensor();
				} else {
					bone->readTransform(RESET_PHYSICS);
				}
			} else {
				bone->readTransform(RESET_PHYSICS);
			}

			m_mesh->m_bones.emplace_back(hdt::make_smart(bone));
			return bone;
		}
		logger::warn("Node named {} doesn't exist, skipped, no bone created", bodyName->cstr());
		return nullptr;
	}

	auto SkyrimSystemCreator::generateMeshBody(const std::string name,
		DefaultBBP::NameSet_t* names) -> std::pair<RE::BSTSmartPointer<SkyrimBody>,
		SkyrimSystemCreator::VertexOffsetMap>
	{
		RE::BSTSmartPointer<SkyrimBody> body = RE::make_smart<SkyrimBody>();
		body->m_name = name;

		auto vertexStart = 0;
		auto boneStart = 0;

		VertexOffsetMap vertexOffsetMap;

		for (auto& meshName : *names) {
			// We wouldn't find the trishape here without the ActorManager::fixArmorNameMaps() fix when the related bug happens
			// (for example when doing the smp reset).
			auto* triShape = castBSTriShape(findObject(m_model, meshName.c_str()));
			auto* dynamicShape = castBSDynamicTriShape(findObject(m_model, meshName.c_str()));
			if (!triShape) {
				continue;
			}

			if (!triShape->GetGeometryRuntimeData().skinInstance) {
				continue;
			}

			RE::NiSkinInstance* skinInstance = triShape->GetGeometryRuntimeData().skinInstance.get();
			RE::NiSkinData* skinData = skinInstance->skinData.get();
			for (uint32_t boneIdx = 0; boneIdx < skinData->bones; ++boneIdx) {
				auto node = skinInstance->bones[boneIdx];
				auto boneData = &skinData->boneData[boneIdx];
				auto boundingSphere = BoundingSphere(convertNi(boneData->bound.center), boneData->bound.radius);
				IDStr boneName = node->name.c_str();
				auto bone = m_mesh->findBone(boneName);
				if (!bone) {
					auto defaultBoneInfo = getBoneTemplate("");
					bone = new SkyrimBone(boneName, node->AsNode(), this->m_skeleton, defaultBoneInfo);
					m_mesh->m_bones.push_back(hdt::make_smart(bone));
					logger::info("Created bone {} added to body {}, created without default values", boneName->cstr(),
						name);
				}

				body->addBone(bone, convertNi(boneData->skinToBone), boundingSphere);
			}

			RE::NiSkinPartition* skinPartition = triShape->GetGeometryRuntimeData().skinInstance->skinPartition.get();
			body->m_vertices.resize(vertexStart + skinPartition->vertexCount);

			// vertices data are all the same in every partitions
			auto partition = skinPartition->partitions.data();
			auto vFlags = partition->vertexDesc.GetFlags();
			auto vSize = partition->vertexDesc.GetSize();
			auto vertexBlock = partition->buffData->rawVertexData;

			uint8_t* dynamicVData = nullptr;
			if (dynamicShape) {
				dynamicVData = static_cast<uint8_t*>(dynamicShape->GetDynamicTrishapeRuntimeData().dynamicData);
			}

			uint8_t boneOffset = 0;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_VERTEX) {
				boneOffset += 16;
			}

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV) {
				boneOffset += 4;
			}

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV_2) {
				boneOffset += 4;
			}

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_NORMAL) {
				boneOffset += 4;
			}

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_TANGENT) {
				boneOffset += 4;
			}

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_COLORS) {
				boneOffset += 4;
			}

			for (uint32_t j = 0; j < skinPartition->vertexCount; ++j) {
				RE::NiPoint3* vertexPos;

				if (dynamicShape) {
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&dynamicVData[j * 16]);
				} else {
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&vertexBlock[j * vSize]);
				}

				body->m_vertices[j + vertexStart].m_skinPos = convertNi(*vertexPos);

				auto boneData =
					reinterpret_cast<SkyrimSystem::BoneData*>(&vertexBlock[j * vSize + boneOffset]);

				for (auto k = 0; k < partition->bonesPerVertex && k < 4; ++k) {
					auto localBoneIndex = boneData->boneIndices[k];
					assert(localBoneIndex < body->m_skinnedBones.size());
					body->m_vertices[j + vertexStart].m_boneIdx[k] = localBoneIndex + boneStart;
					float32(&body->m_vertices[j + vertexStart].m_weight[k], boneData->boneWeights[k]);
				}
			}

			vertexOffsetMap.insert({ meshName, vertexStart });
			boneStart = static_cast<int>(body->m_skinnedBones.size());
			vertexStart = static_cast<int>(body->m_vertices.size());
		}

		if (0 == vertexStart) {
			m_reader->skipCurrentElement();
			return { nullptr, {} };
		}

		for (auto& i : body->m_vertices) {
			i.sortWeight();
		}

		return { body, vertexOffsetMap };
	}

	auto SkyrimSystemCreator::readPerVertexShape(DefaultBBP::NameMap_t meshNameMap) -> RE::BSTSmartPointer<SkyrimBody>
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap.find(name);
		auto names = (it == meshNameMap.end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto body = generateMeshBody(name, &names).first;
		if (!body) {
			return nullptr;
		}

		auto shape = RE::make_smart<PerVertexShape>(body.get());

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("piority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "tag") {
					body->m_tags.emplace_back(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone) {
						body->m_canCollideWithBones.push_back(bone);
					}
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone) {
						body->m_noCollideWithBones.push_back(bone);
					}
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (auto i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
							break;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else if (nodeName == "wind-effect") {
					shape->m_windEffect = m_reader->readFloat();
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		shape->autoGen();
		body->finishBuild();

		return body;
	}

	auto SkyrimSystemCreator::readPerTriangleShape(
		DefaultBBP::NameMap_t* meshNameMap) -> RE::BSTSmartPointer<SkyrimBody>
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap->find(name);
		auto names = (it == meshNameMap->end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto bodyData = generateMeshBody(name, &names);
		auto body = bodyData.first;
		auto vertexOffsetMap = bodyData.second;
		if (!body) {
			return nullptr;
		}

		auto shape = RE::make_smart<PerTriangleShape>(body.get());

		for (auto entry : vertexOffsetMap) {
			auto* g = castBSTriShape(findObject(m_model, entry.first.c_str()));
			if (g->GetGeometryRuntimeData().skinInstance) {
				int offset = entry.second;
				RE::NiSkinPartition* skinPartition = g->GetGeometryRuntimeData().skinInstance->skinPartition.get();
				for (auto i = 0; i < skinPartition->partitions.size(); ++i) {
					auto& partition = skinPartition->partitions[i];
					for (auto j = 0; j < partition.triangles; ++j) {
						shape->addTriangle(partition.triList[j * 3] + offset, partition.triList[j * 3 + 1] + offset,
							partition.triList[j * 3 + 2] + offset);
					}
				}
			} else {
				logger::warn("Shape {} has no skin data, skipped", entry.first.c_str());
				return nullptr;
			}
		}

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("priority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "prenetration" || nodeName == "penetration") {
					shape->m_shapeProp.penetration = m_reader->readFloat();
				} else if (nodeName == "tag") {
					body->m_tags.emplace_back(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone) {
						body->m_canCollideWithBones.push_back(bone);
					}
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone) {
						body->m_noCollideWithBones.push_back(bone);
					}
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (auto i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else if (nodeName == "wind-effect") {
					shape->m_windEffect = m_reader->readFloat();
				} else {
					logger::warn("unknown element - {}", nodeName.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		body->finishBuild();

		return body;
	}

	auto SkyrimSystemCreator::readFrameLerp(btTransform& tr) -> void
	{
		tr.setIdentity();
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();
				if (name == "translationLerp") {
					tr.getOrigin().setX(m_reader->readFloat());
				} else if (name == "rotationLerp") {
					tr.getOrigin().setY(m_reader->readFloat());
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
	}

	auto SkyrimSystemCreator::parseFrameType(const std::string& name, FrameType& frameType, btTransform& frame) -> bool
	{
		if (name == "frameInA") {
			frameType = FrameType::FrameInA;
			frame = m_reader->readTransform();
		} else if (name == "frameInB") {
			frameType = FrameType::FrameInB;
			frame = m_reader->readTransform();
		} else if (name == "frameInLerp") {
			frameType = FrameType::FrameInLerp;
			readFrameLerp(frame);
		} else {
			return false;
		}
		return true;
	}

	auto SkyrimSystemCreator::readGenericConstraintTemplate(GenericConstraintTemplate& dest) -> void
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame)) {
					;
				} else if (name == "enableLinearSprings") {
					dest.enableLinearSprings = m_reader->readBool();
				} else if (name == "enableAngularSprings") {
					dest.enableAngularSprings = m_reader->readBool();
				} else if (name == "linearStiffnessLimited") {
					dest.linearStiffnessLimited = m_reader->readBool();
				} else if (name == "angularStiffnessLimited") {
					dest.angularStiffnessLimited = m_reader->readBool();
				} else if (name == "springDampingLimited") {
					dest.springDampingLimited = m_reader->readBool();
				} else if (name == "linearNonHookeanDamping") {
					dest.linearNonHookeanDamping = m_reader->readVector3();
				} else if (name == "angularNonHookeanDamping") {
					dest.angularNonHookeanDamping = m_reader->readVector3();
				} else if (name == "linearNonHookeanStiffness") {
					dest.linearNonHookeanStiffness = m_reader->readVector3();
				} else if (name == "angularNonHookeanStiffness") {
					dest.angularNonHookeanStiffness = m_reader->readVector3();
				} else if (name == "linearMotors") {
					dest.linearMotors = m_reader->readBool();
				} else if (name == "angularMotors") {
					dest.angularMotors = m_reader->readBool();
				} else if (name == "linearServoMotors") {
					dest.linearServoMotors = m_reader->readBool();
				} else if (name == "angularServoMotors") {
					dest.angularServoMotors = m_reader->readBool();
				} else if (name == "linearTargetVelocity") {
					dest.linearTargetVelocity = m_reader->readVector3();
				} else if (name == "angularTargetVelocity") {
					dest.angularTargetVelocity = m_reader->readVector3();
				} else if (name == "linearMaxMotorForce") {
					dest.linearMaxMotorForce = m_reader->readVector3();
				} else if (name == "angularMaxMotorForce") {
					dest.angularMaxMotorForce = m_reader->readVector3();
				} else if (name == "stopERP") {
					dest.stopERP = m_reader->readFloat();
				} else if (name == "stopCFM") {
					dest.stopCFM = m_reader->readFloat();
				} else if (name == "motorERP") {
					dest.motorERP = m_reader->readFloat();
				} else if (name == "motorCFM") {
					dest.motorCFM = m_reader->readFloat();
				} else if (name == "useLinearReferenceFrameA") {
					dest.useLinearReferenceFrameA = m_reader->readBool();
				} else if (name == "linearLowerLimit") {
					dest.linearLowerLimit = m_reader->readVector3();
				} else if (name == "linearUpperLimit") {
					dest.linearUpperLimit = m_reader->readVector3();
				} else if (name == "angularLowerLimit") {
					dest.angularLowerLimit = m_reader->readVector3();
				} else if (name == "angularUpperLimit") {
					dest.angularUpperLimit = m_reader->readVector3();
				} else if (name == "linearStiffness") {
					dest.linearStiffness = m_reader->readVector3();
				} else if (name == "angularStiffness") {
					dest.angularStiffness = m_reader->readVector3();
				} else if (name == "linearDamping") {
					dest.linearDamping = m_reader->readVector3();
				} else if (name == "angularDamping") {
					dest.angularDamping = m_reader->readVector3();
				} else if (name == "linearEquilibrium") {
					dest.linearEquilibrium = m_reader->readVector3();
				} else if (name == "angularEquilibrium") {
					dest.angularEquilibrium = m_reader->readVector3();
				} else if (name == "linearBounce") {
					dest.linearBounce = m_reader->readVector3();
				} else if (name == "angularBounce") {
					dest.angularBounce = m_reader->readVector3();
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
	}

	auto SkyrimSystemCreator::findBones(const IDStr& bodyAName, const IDStr& bodyBName, SkyrimBone*& bodyA,
		SkyrimBone*& bodyB) -> bool
	{
		bodyA = dynamic_cast<SkyrimBone*>(m_mesh->findBone(bodyAName));
		bodyB = dynamic_cast<SkyrimBone*>(m_mesh->findBone(bodyBName));

		if (!bodyA) {
			logger::warn("constraint {} <-> {} : bone for bodyA doesn't exist, will try to create it",
				bodyAName->cstr(), bodyBName->cstr());
			bodyA = createBoneFromNodeName(bodyAName);
			if (!bodyA) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (!bodyB) {
			logger::warn("constraint {} <-> {} : bone for bodyB doesn't exist, will try to create it",
				bodyAName->cstr(), bodyBName->cstr());
			bodyB = createBoneFromNodeName(bodyBName);
			if (!bodyB) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (bodyA == bodyB) {
			logger::warn("constraint between same object {} <-> {}, skipped", bodyAName->cstr(), bodyBName->cstr());
			m_reader->skipCurrentElement();
			return false;
		}

		if (bodyA->m_rig.isKinematicObject() && bodyB->m_rig.isKinematicObject()) {
			logger::warn("constraint between two kinematic object {} <-> {}, skipped", bodyAName->cstr(),
				bodyBName->cstr());
			m_reader->skipCurrentElement();
			return false;
		}

		logger::info("OK: constraint between object {} <-> {}", bodyAName->cstr(), bodyBName->cstr());
		return true;
	}

	static auto rotFromAtoB(const btVector3& a, const btVector3& b) -> btQuaternion
	{
		auto axis = a.cross(b);
		if (axis.fuzzyZero()) {
			return btQuaternion::getIdentity();
		}
		float sinA = axis.length();
		float cosA = a.dot(b);
		float angle = btAtan2(cosA, sinA);
		return btQuaternion(axis, angle);
	}

	auto SkyrimSystemCreator::calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA,
		const btQsTransform& trB, btTransform& frameA, btTransform& frameB) -> void
	{
		btQsTransform frameInWorld;
		switch (type) {
		case FrameType::FrameInA:
			frameA = frame;
			frameInWorld = trA * btQsTransform(frame);
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		case FrameType::FrameInB:
			frameB = frame;
			frameInWorld = trB * btQsTransform(frameB);
			frameA = (trA.inverse() * frameInWorld).asTransform();
			break;
		case FrameType::FrameInLerp:
		{
			auto trans = trA.getOrigin().lerp(trB.getOrigin(), frame.getOrigin().x());
			auto rot = trA.getBasis().slerp(trB.getBasis(), frame.getOrigin().y());
			frameInWorld = btQsTransform(rot, trans);
			frameA = (trA.inverse() * frameInWorld).asTransform();
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		}
		case FrameType::AWithXPointToB:
		{
			btMatrix3x3 matr(trA.getBasis());
			frameInWorld = trA;
			auto old = matr.getColumn(0).normalized();
			auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
			auto q = rotFromAtoB(old, a2b);
			frameInWorld.getBasis() *= q;
			frameA = (trA.inverse() * frameInWorld).asTransform();
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		}
		case FrameType::AWithYPointToB:
		{
			btMatrix3x3 matr(trA.getBasis());
			frameInWorld = trA;
			auto old = matr.getColumn(1).normalized();
			auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
			auto q = rotFromAtoB(old, a2b);
			frameInWorld.getBasis() *= q;
			frameA = (trA.inverse() * frameInWorld).asTransform();
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		}
		case FrameType::AWithZPointToB:
		{
			btMatrix3x3 matr(trA.getBasis());
			frameInWorld = trA;
			auto old = matr.getColumn(2).normalized();
			auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
			auto q = rotFromAtoB(old, a2b);
			frameInWorld.getBasis() *= q;
			frameA = (trA.inverse() * frameInWorld).asTransform();
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		}
		}
	}

	auto SkyrimSystemCreator::readGenericConstraint() -> RE::BSTSmartPointer<Generic6DofConstraint>
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB)) {
			return nullptr;
		}

		const auto& trA = bodyA->m_currentTransform;
		const auto& trB = bodyB->m_currentTransform;

		auto cinfo = getGenericConstraintTemplate(clsname);
		readGenericConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<Generic6DofConstraint> constraint;
		if (cinfo.useLinearReferenceFrameA) {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyB, bodyA, frameB, frameA);
		} else {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyA, bodyB, frameA, frameB);
		}

		constraint->setLinearLowerLimit(cinfo.linearLowerLimit);
		constraint->setLinearUpperLimit(cinfo.linearUpperLimit);
		constraint->setAngularLowerLimit(cinfo.angularLowerLimit);
		constraint->setAngularUpperLimit(cinfo.angularUpperLimit);
		for (auto i = 0; i < 3; ++i) {
			constraint->setStiffness(i, cinfo.linearStiffness[i], cinfo.linearStiffnessLimited);
			constraint->setStiffness(i + 3, cinfo.angularStiffness[i], cinfo.angularStiffnessLimited);
			constraint->setDamping(i, cinfo.linearDamping[i], cinfo.springDampingLimited);
			constraint->setDamping(i + 3, cinfo.angularDamping[i], cinfo.springDampingLimited);

			constraint->setEquilibriumPoint(i, cinfo.linearEquilibrium[i]);
			constraint->setEquilibriumPoint(i + 3, cinfo.angularEquilibrium[i]);

			//constraint->setNonHookeanDamping(i, cinfo.linearNonHookeanDamping[i]);
			//constraint->setNonHookeanDamping(i + 3, cinfo.angularNonHookeanDamping[i]);
			//constraint->setNonHookeanStiffness(i, cinfo.linearNonHookeanStiffness[i]);
			//constraint->setNonHookeanStiffness(i + 3, cinfo.angularNonHookeanStiffness[i]);

			constraint->enableSpring(i, cinfo.enableLinearSprings);
			constraint->enableSpring(i + 3, cinfo.enableAngularSprings);

			constraint->enableMotor(i, cinfo.linearMotors);
			constraint->enableMotor(i + 3, cinfo.angularMotors);
			constraint->setServo(i, cinfo.linearServoMotors);
			constraint->setServo(i + 3, cinfo.angularServoMotors);
			// TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server motor target.  Hard coded to equilibrium right now.
			constraint->setServoTarget(i, cinfo.linearEquilibrium[i]);
			constraint->setServoTarget(i + 3, cinfo.angularEquilibrium[i]);
			constraint->setTargetVelocity(i, cinfo.linearTargetVelocity[i]);
			constraint->setTargetVelocity(i + 3, cinfo.angularTargetVelocity[i]);
			constraint->setMaxMotorForce(i, cinfo.linearMaxMotorForce[i]);
			constraint->setMaxMotorForce(i + 3, cinfo.angularMaxMotorForce[i]);

			constraint->setParam(BT_CONSTRAINT_ERP, cinfo.motorERP, i);
			constraint->setParam(BT_CONSTRAINT_CFM, cinfo.motorCFM, i);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP, cinfo.stopERP, i);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM, cinfo.stopCFM, i);
		}
		constraint->getTranslationalLimitMotor()->m_bounce = cinfo.linearBounce;
		constraint->getRotationalLimitMotor(0)->m_bounce = cinfo.angularBounce[0];
		constraint->getRotationalLimitMotor(1)->m_bounce = cinfo.angularBounce[1];
		constraint->getRotationalLimitMotor(2)->m_bounce = cinfo.angularBounce[2];
		/*constraint->getTranslationalLimitMotor()->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(0)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(1)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(2)->m_limitSoftness = 1;*/

		return constraint;
	}

	auto SkyrimSystemCreator::readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest) -> void
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();
				if (name == "minDistanceFactor") {
					dest.minDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				} else if (name == "maxDistanceFactor") {
					dest.maxDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				} else if (name == "stiffness") {
					dest.stiffness = std::max(m_reader->readFloat(), 0.0f);
				} else if (name == "damping") {
					dest.damping = std::max(m_reader->readFloat(), 0.0f);
				} else if (name == "equilibrium") {
					dest.equilibriumFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
	}

	auto SkyrimSystemCreator::readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest) -> void
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				const auto& name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame)) {
					;
				} else if (name == "swingSpan1" || name == "coneLimit" || name == "limitZ") {
					dest.swingSpan1 = std::max(m_reader->readFloat(), 0.f);
				} else if (name == "swingSpan2" || name == "planeLimit" || name == "limitY") {
					dest.swingSpan2 = std::max(m_reader->readFloat(), 0.f);
				} else if (name == "twistSpan" || name == "twistLimit" || name == "limitX") {
					dest.twistSpan = std::max(m_reader->readFloat(), 0.f);
				} else if (name == "limitSoftness") {
					dest.limitSoftness = btClamped(m_reader->readFloat(), 0.f, 1.f);
				} else if (name == "biasFactor") {
					dest.biasFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				} else if (name == "relaxationFactor") {
					dest.relaxationFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}
	}

	auto SkyrimSystemCreator::getBoneTemplate(const IDStr& name) -> const SkyrimSystemCreator::BoneTemplate&
	{
		auto iter = m_boneTemplates.find(name);
		if (iter == m_boneTemplates.end()) {
			return m_boneTemplates[""];
		}
		return iter->second;
	}

	auto SkyrimSystemCreator::getGenericConstraintTemplate(
		const IDStr& name) -> const SkyrimSystemCreator::GenericConstraintTemplate&
	{
		auto iter = m_genericConstraintTemplates.find(name);
		if (iter == m_genericConstraintTemplates.end()) {
			return m_genericConstraintTemplates[""];
		}
		return iter->second;
	}

	auto SkyrimSystemCreator::getStiffSpringConstraintTemplate(
		const IDStr& name) -> const SkyrimSystemCreator::StiffSpringConstraintTemplate&
	{
		auto iter = m_stiffSpringConstraintTemplates.find(name);
		if (iter == m_stiffSpringConstraintTemplates.end()) {
			return m_stiffSpringConstraintTemplates[""];
		}
		return iter->second;
	}

	auto SkyrimSystemCreator::getConeTwistConstraintTemplate(
		const IDStr& name) -> const SkyrimSystemCreator::ConeTwistConstraintTemplate&
	{
		auto iter = m_coneTwistConstraintTemplates.find(name);
		if (iter == m_coneTwistConstraintTemplates.end()) {
			return m_coneTwistConstraintTemplates[""];
		}
		return iter->second;
	}

	auto SkyrimSystemCreator::readStiffSpringConstraint() -> RE::BSTSmartPointer<StiffSpringConstraint>
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB)) {
			return nullptr;
		}

		StiffSpringConstraintTemplate cinfo = getStiffSpringConstraintTemplate(clsname);
		readStiffSpringConstraintTemplate(cinfo);

		RE::BSTSmartPointer<StiffSpringConstraint> constraint = RE::make_smart<StiffSpringConstraint>(bodyA, bodyB);
		constraint->m_minDistance *= cinfo.minDistanceFactor;
		constraint->m_maxDistance *= cinfo.maxDistanceFactor;
		constraint->m_stiffness = cinfo.stiffness;
		constraint->m_damping = cinfo.damping;
		constraint->m_equilibriumPoint = constraint->m_minDistance * cinfo.equilibriumFactor +
		                                 constraint->m_maxDistance * (1 - cinfo.equilibriumFactor);
		return constraint;
	}

	auto SkyrimSystemCreator::readConeTwistConstraint() -> RE::BSTSmartPointer<ConeTwistConstraint>
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA = nullptr, *bodyB = nullptr;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB)) {
			return nullptr;
		}

		const auto& trA = bodyA->m_currentTransform;
		const auto& trB = bodyB->m_currentTransform;

		auto cinfo = getConeTwistConstraintTemplate(clsname);
		readConeTwistConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<ConeTwistConstraint> constraint =
			RE::make_smart<ConeTwistConstraint>(bodyA, bodyB, frameA, frameB);
		constraint->setLimit(cinfo.swingSpan1, cinfo.swingSpan2, cinfo.twistSpan, cinfo.limitSoftness, cinfo.biasFactor,
			cinfo.relaxationFactor);

		return constraint;
	}
}
