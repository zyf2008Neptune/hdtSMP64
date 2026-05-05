#include "hdtSkyrimSystem.h"

#include <bit>
#include <utility>

#include "HavokUtils.h"
#include "XmlReader.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkyrimPhysicsWorld.h"

// F16C isn't supported on super old processors. AVX2+ (AVX processors can have it, but not guaranteed)
#if defined(__AVX2__) || defined(__AVX512F__)
#include <immintrin.h>
#else
// float32
// Martin Kallman
//
// Fast half-precision to single-precision floating point conversion
//  - Supports signed zero and denormals-as-zero (DAZ)
//  - Does not support infinities or NaN
//  - Few, partially pipelinable, non-branching instructions,
//  - Core operations ~6 clock cycles on modern x86-64
static auto __float32(float* __restrict out, const uint16_t in) -> void
{
    uint32_t t1 = in & 0x7fff; // Non-sign bits
    uint32_t t2 = in & 0x8000; // Sign bit
    const uint32_t t3 = in & 0x7c00; // Exponent

    t1 <<= 13; // Align mantissa on MSB
    t2 <<= 16; // Shift sign bit into position

    t1 += 0x38000000; // Adjust bias

    t1 = (t3 == 0 ? 0 : t1); // Denormals-as-zero

    t1 |= t2; // Re-insert sign bit

    *out = std::bit_cast<float>(t1);
};
#endif

namespace hdt
{
    [[maybe_unused]] static constexpr auto PI = std::numbers::pi_v<float>;

    btEmptyShape SkyrimSystemCreator::BoneTemplate::emptyShape[1];

    auto SkyrimSystem::findBone(const RE::BSFixedString& name) const -> SkinnedMeshBone*
    {
        for (const auto& i : m_bones)
        {
            if (i->m_name == name)
            {
                return i.get();
            }
        }

        return nullptr;
    }

    auto SkyrimSystem::findBody(const RE::BSFixedString& name) const -> SkinnedMeshBody*
    {
        for (const auto& i : m_meshes)
        {
            if (i->m_name == name)
            {
                return i.get();
            }
        }

        return nullptr;
    }

    auto SkyrimSystem::findBoneIdx(const RE::BSFixedString& name) const -> int
    {
        for (auto i = 0; i < m_bones.size(); ++i)
        {
            if (m_bones[i]->m_name == name)
            {
                return i;
            }
        }

        return -1;
    }

    SkyrimSystem::SkyrimSystem(RE::NiNode* skeleton) : m_skeleton(skeleton), m_oldRoot(nullptr)
    {
        m_oldRoot = m_skeleton;
    }

    auto SkyrimSystem::prepareForRead(float timeStep) -> float
    {
        auto newRoot = m_skeleton.get();
        while (newRoot->parent)
        {
            newRoot = newRoot->parent;
        }

        if (m_oldRoot != newRoot)
        {
            timeStep = RESET_PHYSICS;
        }

        if (!m_initialized)
        {
            timeStep = RESET_PHYSICS;
            m_initialized = true;
        }

        if (timeStep <= RESET_PHYSICS)
        {
            if (!this->block_resetting)
            {
                updateTransformUpDown(m_skeleton.get(), true);
            }

            m_lastRootRotation = convertNi(m_skeleton->world.rotate);
        }
        else if (m_skeleton->parent == RE::PlayerCharacter::GetSingleton()->Get3D2())
        {
            if (SkyrimPhysicsWorld::get()->m_resetPc > 0)
            {
                timeStep = RESET_PHYSICS;
                updateTransformUpDown(m_skeleton.get(), true);
                m_lastRootRotation = convertNi(m_skeleton->world.rotate);
            }
            else if (!RE::PlayerCamera::GetSingleton()->GetRuntimeData2().isWeapSheathed ||
                     RE::PlayerCamera::GetSingleton()->currentState->id == RE::CameraState::kFirstPerson)
            // isWeaponSheathed or potentially isCameraFree || cameraState is first person
            {
                m_lastRootRotation = convertNi(m_skeleton->world.rotate);
            }
            else
            {
                const btQuaternion newRot = convertNi(m_skeleton->world.rotate);
                btVector3 rotAxis;
                float rotAngle;
                btTransformUtil::calculateDiffAxisAngleQuaternion(m_lastRootRotation, newRot, rotAxis, rotAngle);

                if (SkyrimPhysicsWorld::get()->m_clampRotations)
                {
                    const float limit = SkyrimPhysicsWorld::get()->m_rotationSpeedLimit * timeStep;

                    if (rotAngle < -limit || rotAngle > limit)
                    {
                        rotAngle = btClamped(rotAngle, -limit, limit);
                        const btQuaternion clampedRot(rotAxis, rotAngle);
                        m_lastRootRotation = clampedRot * m_lastRootRotation;
                        m_skeleton->world.rotate = convertBt(m_lastRootRotation);

                        const auto& children = m_skeleton->GetChildren();
                        for (const auto& i : children)
                        {
                            if (const auto node = castNiNode(i.get()))
                            {
                                updateTransformUpDown(node, true);
                            }
                        }
                    }
                }
                else if (SkyrimPhysicsWorld::get()->m_unclampedResets)
                {
                    const float limit = SkyrimPhysicsWorld::get()->m_unclampedResetAngle * timeStep;

                    if (rotAngle < -limit || rotAngle > limit)
                    {
                        timeStep = RESET_PHYSICS;
                        updateTransformUpDown(m_skeleton.get(), true);
                        m_lastRootRotation = convertNi(m_skeleton->world.rotate);
                    }
                }
            }
        }

        m_oldRoot = hdt::make_nismart(newRoot);
        return timeStep;
    }

    auto SkyrimSystemCreator::indexBone(SkyrimBone* bone) -> void { m_boneIndex.emplace(bone->m_name.data(), bone); }

    auto SkyrimSystemCreator::findBoneFromIndex(const RE::BSFixedString& name) const -> SkyrimBone*
    {
        const auto it = m_boneIndex.find(name.data());
        return it != m_boneIndex.end() ? it->second : nullptr;
    }

    auto SkyrimSystemCreator::findObjectByName(const RE::BSFixedString& name) const -> RE::NiNode*
    {
        // TODO check it's not a lurker skeleton
        return findNode(m_skeleton, name);
    }

    auto SkyrimSystemCreator::getOrCreateBone(const RE::BSFixedString& name) -> SkyrimBone*
    {
        if (const auto bone = findBoneFromIndex(getRenamedBone(name)))
        {
            return bone;
        }

        logger::warn("Bone {} used before being created, trying to create it with current default values",
                     name.c_str());
        return createBoneFromNodeName(name);
    }

    auto SkyrimSystemCreator::getRenamedBone(const RE::BSFixedString& name) -> RE::BSFixedString
    {
        const auto iter = m_renameMap.find(name);
        if (iter != m_renameMap.end())
        {
            return iter->second;
        }
        return name;
    }

    auto SkyrimSystemCreator::createOrUpdateSystem(RE::NiNode* skeleton, RE::NiAVObject* model,
                                                   DefaultBBP::PhysicsFile_t* file,
                                                   std::unordered_map<RE::BSFixedString, RE::BSFixedString>&& renameMap,
                                                   SkyrimSystem* old_system) -> RE::BSTSmartPointer<SkyrimSystem>
    {
        auto path = file->first;
        if (path.empty())
        {
            return nullptr;
        }

        auto loaded = readAllFile(path.c_str());
        if (loaded.empty())
        {
            return nullptr;
        }

        m_renameMap = std::move(renameMap);
        m_skeleton = skeleton;
        m_model = model;
        m_filePath = path;

        XMLReader reader(reinterpret_cast<uint8_t*>(loaded.data()), loaded.size());
        m_reader = &reader;

        m_reader->nextStartElement();
        if (m_reader->GetName() != "system")
        {
            if (!old_system)
            {
                updateTransformUpDown(m_skeleton, true);
            }

            return nullptr;
        }

        auto meshNameMap = file->second;

        m_mesh = RE::make_smart<SkyrimSystem>(skeleton);
        m_boneIndex.clear();

        // Store original locale
        const auto saved_locale = std::locale();

        // Set locale to en_US
        std::locale::global(std::locale::classic());

        // This forces the skeleton into a neutral reference pose, which avoids building invalid shape data
        // We pull the references directly from havok for the exact same reference data the engine uses
        std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>> savedPoses;

        if (auto* userData = skeleton->GetUserData())
        {
            if (auto* actor = userData->As<RE::Actor>())
            {
                if (auto havokSkel = havok::getAnimationSkeleton(actor))
                {
                    savedPoses.reserve(havokSkel->bones.size());

                    for (auto i = 0; i < havokSkel->bones.size(); ++i)
                    {
                        if (auto boneNode =
                                skeleton->GetObjectByName(RE::BSFixedString(havokSkel->bones[i].name.data())))
                        {
                            savedPoses.emplace_back(boneNode, boneNode->local);

                            RE::hkQsTransform refPose;
                            if (havok::getReferencePoseByIndex(havokSkel, i, refPose))
                            {
                                boneNode->local = havok::hKQsTransformToNiTransform(refPose);
                            }
                        }
                    }

                    if (!savedPoses.empty())
                    {
                        RE::NiUpdateData updateData;
                        skeleton->Update(updateData);
                    }
                }
            }
        }

        if (!old_system)
        {
            updateTransformUpDown(m_skeleton, true);
        }

        try
        {
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    const auto name = m_reader->GetName();
                    if (name == "bone")
                    {
                        readOrUpdateBone();
                    }
                    else if (name == "bone-default")
                    {
                        auto clsname = m_reader->getAttribute("name", "");
                        auto extends = m_reader->getAttribute("extends", "");
                        auto defaultBoneInfo = getBoneTemplate(extends);
                        readBoneTemplate(defaultBoneInfo);
                        m_boneTemplates[clsname] = defaultBoneInfo;
                    }
                    else if (name == "per-vertex-shape")
                    {
                        auto shape = readPerVertexShape(meshNameMap);
                        if (shape && !shape->m_vertices.empty())
                        {
                            m_mesh->m_meshes.emplace_back(shape);
                            shape->m_mesh = m_mesh.get();
                        }
                    }
                    else if (name == "per-triangle-shape")
                    {
                        auto shape = readPerTriangleShape(&meshNameMap);
                        if (shape && !shape->m_vertices.empty())
                        {
                            m_mesh->m_meshes.emplace_back(shape);
                            shape->m_mesh = m_mesh.get();
                        }
                    }
                    else if (name == "constraint-group")
                    {
                        if (auto constraint = readConstraintGroup())
                        {
                            m_mesh->m_constraintGroups.push_back(constraint);
                        }
                    }
                    else if (name == "generic-constraint")
                    {
                        if (auto constraint = readGenericConstraint())
                        {
                            m_mesh->m_constraints.emplace_back(constraint);
                        }
                    }
                    else if (name == "stiffspring-constraint")
                    {
                        if (auto constraint = readStiffSpringConstraint())
                        {
                            m_mesh->m_constraints.emplace_back(constraint);
                        }
                    }
                    else if (name == "conetwist-constraint")
                    {
                        if (auto constraint = readConeTwistConstraint())
                        {
                            m_mesh->m_constraints.emplace_back(constraint);
                        }
                    }
                    else if (name == "generic-constraint-default")
                    {
                        auto clsname = m_reader->getAttribute("name", "");
                        auto extends = m_reader->getAttribute("extends", "");
                        auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
                        readGenericConstraintTemplate(defaultGenericConstraintTemplate);
                        m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
                    }
                    else if (name == "stiffspring-constraint-default")
                    {
                        auto clsname = m_reader->getAttribute("name", "");
                        auto extends = m_reader->getAttribute("extends", "");
                        auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
                        readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
                        m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
                    }
                    else if (name == "conetwist-constraint-default")
                    {
                        auto clsname = m_reader->getAttribute("name", "");
                        auto extends = m_reader->getAttribute("extends", "");
                        auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
                        readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
                        m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
                    }
                    else if (name == "shape")
                    {
                        auto attrName = m_reader->getAttribute("name");
                        if (auto shape = readShape())
                        {
                            m_shapeRefs.push_back(shape);
                            m_shapes.insert(std::make_pair(attrName, shape));
                        }
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
        }
        catch (const std::string& err)
        {
            logger::error("xml parse error - {}", err.c_str());
            return nullptr;
        }

        if (m_deferredBuilds.size() > 2)
        {
            tbb::parallel_for_each(m_deferredBuilds.begin(), m_deferredBuilds.end(),
                                   [](const DeferredBuild& db)
                                   {
                                       if (db.vertexShape)
                                       {
                                           db.vertexShape->autoGen();
                                       }
                                       db.body->finishBuild();
                                   });
        }
        else if (!m_deferredBuilds.empty() && m_deferredBuilds.size() <= 2)
        {
            for (const auto& db : m_deferredBuilds)
            {
                if (db.vertexShape)
                {
                    db.vertexShape->autoGen();
                }
                db.body->finishBuild();
            }
        }

        m_deferredBuilds.clear();

        // Restore original locale
        std::locale::global(saved_locale);

        if (m_reader->GetErrorCode() != Xml::ErrorCode::None)
        {
            logger::error("xml parse error - {}", m_reader->GetErrorMessage());
            return nullptr;
        }

        m_mesh->m_skeleton = hdt::make_nismart(m_skeleton);
        m_mesh->m_shapeRefs.swap(m_shapeRefs);
        std::ranges::sort(
            m_mesh->m_bones, [](const auto& a, const auto& b)
            { return static_cast<SkyrimBone*>(a.get())->m_depth < static_cast<SkyrimBone*>(b.get())->m_depth; });

        // Restore the original pose to avoid a visual 1 Havok tick T-pose (only for visual reasons, it won't break
        // anything otherwise)
        for (auto& [node, transform] : savedPoses)
        {
            node->local = transform;
        }
        if (!savedPoses.empty())
        {
            RE::NiUpdateData updateData;
            skeleton->Update(updateData);
        }

        return m_mesh->valid() ? m_mesh : nullptr;
    }

    auto SkyrimSystemCreator::readConstraintGroup() -> RE::BSTSmartPointer<ConstraintGroup>
    {
        RE::BSTSmartPointer<ConstraintGroup> ret = RE::make_smart<ConstraintGroup>();

        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();

                if (name == "generic-constraint")
                {
                    if (auto constraint = readGenericConstraint())
                    {
                        ret->m_constraints.emplace_back(constraint);
                    }
                }
                else if (name == "stiffspring-constraint")
                {
                    if (auto constraint = readStiffSpringConstraint())
                    {
                        ret->m_constraints.emplace_back(constraint);
                    }
                }
                else if (name == "conetwist-constraint")
                {
                    if (auto constraint = readConeTwistConstraint())
                    {
                        ret->m_constraints.emplace_back(constraint);
                    }
                }
                else if (name == "generic-constraint-default")
                {
                    auto clsname = m_reader->getAttribute("name", "");
                    auto extends = m_reader->getAttribute("extends", "");
                    auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
                    readGenericConstraintTemplate(defaultGenericConstraintTemplate);
                    m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
                }
                else if (name == "stiffspring-constraint-default")
                {
                    auto clsname = m_reader->getAttribute("name", "");
                    auto extends = m_reader->getAttribute("extends", "");
                    auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
                    readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
                    m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
                }
                else if (name == "conetwist-constraint-default")
                {
                    auto clsname = m_reader->getAttribute("name", "");
                    auto extends = m_reader->getAttribute("extends", "");
                    auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
                    readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
                    m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
        return ret;
    }

    auto SkyrimSystemCreator::readBoneTemplate(BoneTemplate& cinfo) -> void
    {
        auto clearCollide = true;
        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();
                if (name == "mass")
                {
                    cinfo.m_mass = m_reader->readFloat();
                }
                else if (name == "inertia")
                {
                    cinfo.m_localInertia = m_reader->readVector3();
                }
                else if (name == "centerOfMassTransform")
                {
                    cinfo.m_centerOfMassTransform = m_reader->readTransform();
                }
                else if (name == "linearDamping")
                {
                    cinfo.m_linearDamping = m_reader->readFloat();
                }
                else if (name == "angularDamping")
                {
                    cinfo.m_angularDamping = m_reader->readFloat();
                }
                else if (name == "friction")
                {
                    cinfo.m_friction = m_reader->readFloat();
                }
                else if (name == "rollingFriction")
                {
                    cinfo.m_rollingFriction = m_reader->readFloat();
                }
                else if (name == "restitution")
                {
                    cinfo.m_restitution = m_reader->readFloat();
                }
                else if (name == "margin-multiplier")
                {
                    cinfo.m_marginMultipler = m_reader->readFloat();
                }
                else if (name == "shape")
                {
                    auto shape = readShape();
                    if (shape)
                    {
                        m_shapeRefs.push_back(shape);
                        cinfo.m_collisionShape = shape.get();
                    }
                    else
                    {
                        cinfo.m_collisionShape = BoneTemplate::emptyShape;
                    }
                }
                else if (name == "collision-filter")
                {
                    cinfo.m_collisionFilter = m_reader->readInt();
                }
                else if (name == "can-collide-with-bone")
                {
                    if (clearCollide)
                    {
                        cinfo.m_canCollideWithBone.clear();
                        cinfo.m_noCollideWithBone.clear();
                        clearCollide = false;
                    }
                    cinfo.m_canCollideWithBone.emplace_back(m_reader->readText());
                }
                else if (name == "no-collide-with-bone")
                {
                    if (clearCollide)
                    {
                        cinfo.m_canCollideWithBone.clear();
                        cinfo.m_noCollideWithBone.clear();
                        clearCollide = false;
                    }
                    cinfo.m_noCollideWithBone.emplace_back(m_reader->readText());
                }
                else if (name == "gravity-factor")
                {
                    cinfo.m_gravityFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
                }
                else if (name == "wind-factor")
                {
                    cinfo.m_windFactor = std::max(m_reader->readFloat(), 0.0f);
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
    }

    auto SkyrimSystemCreator::readShape() -> std::shared_ptr<btCollisionShape>
    {
        const auto typeStr = m_reader->getAttribute("type");
        if (typeStr == "ref")
        {
            const auto shapeName = m_reader->getAttribute("name");
            m_reader->skipCurrentElement();
            const auto iter = m_shapes.find(shapeName);
            if (iter != m_shapes.end())
            {
                return iter->second;
            }
            logger::warn("unknown shape - {}", shapeName.c_str());
            return nullptr;
        }
        if (typeStr == "box")
        {
            btVector3 halfExtend(0, 0, 0);
            float margin = 0;
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    auto name = m_reader->GetName();
                    if (name == "halfExtend")
                    {
                        halfExtend = m_reader->readVector3();
                    }
                    else if (name == "margin")
                    {
                        margin = m_reader->readFloat();
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
            auto ret = std::make_shared<btBoxShape>(halfExtend);
            ret->setMargin(margin);
            return ret;
        }
        if (typeStr == "sphere")
        {
            float radius = 0;
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    auto name = m_reader->GetName();
                    if (name == "radius")
                    {
                        radius = m_reader->readFloat();
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
            return std::make_shared<btSphereShape>(radius);
        }
        if (typeStr == "capsule")
        {
            float radius = 0;
            float height = 0;
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    auto name = m_reader->GetName();
                    if (name == "radius")
                    {
                        radius = m_reader->readFloat();
                    }
                    else if (name == "height")
                    {
                        height = m_reader->readFloat();
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
            return std::make_shared<btCapsuleShape>(radius, height);
        }
        if (typeStr == "hull")
        {
            float margin = 0;
            auto ret = std::make_shared<btConvexHullShape>();
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    auto name = m_reader->GetName();
                    if (name == "point")
                    {
                        ret->addPoint(m_reader->readVector3(), false);
                    }
                    else if (name == "margin")
                    {
                        margin = m_reader->readFloat();
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
            ret->recalcLocalAabb();
            return ret->getNumPoints() ? ret : nullptr;
        }
        if (typeStr == "cylinder")
        {
            float height = 0;
            float radius = 0;
            float margin = 0;
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    auto name = m_reader->GetName();
                    if (name == "height")
                    {
                        height = m_reader->readFloat();
                    }
                    else if (name == "radius")
                    {
                        radius = m_reader->readFloat();
                    }
                    else if (name == "margin")
                    {
                        margin = m_reader->readFloat();
                    }
                    else
                    {
                        logger::warn("unknown element - {}", name.c_str());
                        m_reader->skipCurrentElement();
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }

            if (radius >= 0 && height >= 0)
            {
                auto ret = std::make_shared<btCylinderShape>(btVector3(radius, height, radius));
                ret->setMargin(margin);
                return ret;
            }
            return nullptr;
        }
        if (typeStr == "compound")
        {
            auto ret = std::make_shared<btCompoundShape>();
            while (m_reader->Inspect())
            {
                if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                {
                    if (m_reader->GetName() == "child")
                    {
                        btTransform tr;
                        std::shared_ptr<btCollisionShape> shape;

                        while (m_reader->Inspect())
                        {
                            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
                            {
                                if (m_reader->GetName() == "transform")
                                {
                                    tr = m_reader->readTransform();
                                }
                                else if (m_reader->GetName() == "shape")
                                {
                                    shape = readShape();
                                }
                                else
                                {
                                    logger::warn("unknown element - {}", m_reader->GetName().c_str());
                                    m_reader->skipCurrentElement();
                                }
                            }
                            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                            {
                                break;
                            }
                        }

                        if (shape)
                        {
                            ret->addChildShape(tr, shape.get());
                            m_shapeRefs.push_back(shape);
                        }
                    }
                }
                else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
                {
                    break;
                }
            }
            return ret->getNumChildShapes() ? ret : nullptr;
        }
        logger::warn("Unknown shape type {}", typeStr.c_str());
        return nullptr;
    }

    auto SkyrimSystemCreator::readOrUpdateBone() -> void
    {
        const RE::BSFixedString name = getRenamedBone(m_reader->getAttribute("name"));
        if (findBoneFromIndex(name))
        {
            logger::warn("Bone {} already exists, skipped", name.c_str());
            return;
        }

        const RE::BSFixedString cls = m_reader->getAttribute("template", "");
        if (!createBoneFromNodeName(name, cls, true))
        {
            m_reader->skipCurrentElement();
        }
    }

    auto SkyrimSystemCreator::createBoneFromNodeName(const RE::BSFixedString& bodyName,
                                                     const RE::BSFixedString& templateName, const bool readTemplate)
        -> SkyrimBone*
    {
        if (const auto node = findObjectByName(bodyName))
        {
            logger::info("Found node named {}, creating bone", bodyName.c_str());
            auto boneTemplate = getBoneTemplate(templateName);
            if (readTemplate)
            {
                readBoneTemplate(boneTemplate);
            }
            const auto bone = new SkyrimBone(node->name.c_str(), node, this->m_skeleton, boneTemplate);
            bone->m_localToRig = boneTemplate.m_centerOfMassTransform;
            bone->m_rigToLocal = boneTemplate.m_centerOfMassTransform.inverse();
            bone->m_marginMultipler = boneTemplate.m_marginMultipler;
            bone->m_gravityFactor = boneTemplate.m_gravityFactor;
            bone->m_windFactor = boneTemplate.m_windFactor;

            bone->readTransform(RESET_PHYSICS);

            m_mesh->m_bones.emplace_back(hdt::make_smart(bone));
            indexBone(bone);
            return bone;
        }
        logger::warn("Node named {} doesn't exist, skipped, no bone created", bodyName.c_str());
        return nullptr;
    }

    auto SkyrimSystemCreator::generateMeshBody(const std::string_view name, DefaultBBP::NameSet_t* names)
        -> std::pair<RE::BSTSmartPointer<SkyrimBody>, VertexOffsetMap>
    {
        RE::BSTSmartPointer<SkyrimBody> body = RE::make_smart<SkyrimBody>();
        body->m_name = name;

        auto vertexStart = 0;
        auto boneStart = 0;

        VertexOffsetMap vertexOffsetMap;

        for (auto& meshName : *names)
        {
            // We wouldn't find the trishape here without the ActorManager::fixArmorNameMaps() fix when the related bug
            // happens (for example when doing the smp reset).
            auto* triShape = castBSTriShape(findObject(m_model, meshName));
            auto* dynamicShape = castBSDynamicTriShape(findObject(m_model, meshName));
            if (!triShape)
            {
                continue;
            }

            if (!triShape->GetGeometryRuntimeData().skinInstance)
            {
                continue;
            }

            RE::NiSkinInstance* skinInstance = triShape->GetGeometryRuntimeData().skinInstance.get();
            RE::NiSkinData* skinData = skinInstance->skinData.get();
            for (uint32_t boneIdx = 0; boneIdx < skinData->bones; ++boneIdx)
            {
                auto node = skinInstance->bones[boneIdx];
                if (!node)
                {
                    continue;
                }
                auto boneData = &skinData->boneData[boneIdx];
                auto boundingSphere = BoundingSphere(convertNi(boneData->bound.center), boneData->bound.radius);
                const RE::BSFixedString& boneName = node->name;
                auto bone = static_cast<SkinnedMeshBone*>(findBoneFromIndex(boneName));
                if (!bone)
                {
                    auto defaultBoneInfo = getBoneTemplate("");
                    auto newBone = new SkyrimBone(boneName, node->AsNode(), this->m_skeleton, defaultBoneInfo);
                    m_mesh->m_bones.emplace_back(hdt::make_smart(newBone));
                    indexBone(newBone);
                    bone = newBone;
                    logger::info("Created bone {} added to body {}, created without default values", boneName.c_str(),
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
            if (dynamicShape)
            {
                dynamicVData = static_cast<uint8_t*>(dynamicShape->GetDynamicTrishapeRuntimeData().dynamicData);
            }

            uint8_t boneOffset = 0;

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_VERTEX)
            {
                boneOffset += 16;
            }

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV)
            {
                boneOffset += 4;
            }

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV_2)
            {
                boneOffset += 4;
            }

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_NORMAL)
            {
                boneOffset += 4;
            }

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_TANGENT)
            {
                boneOffset += 4;
            }

            if (vFlags & RE::BSGraphics::Vertex::Flags::VF_COLORS)
            {
                boneOffset += 4;
            }

            for (uint32_t j = 0; j < skinPartition->vertexCount; ++j)
            {
                RE::NiPoint3* vertexPos;

                if (dynamicShape)
                {
                    vertexPos = reinterpret_cast<RE::NiPoint3*>(&dynamicVData[j * 16]);
                }
                else
                {
                    vertexPos = reinterpret_cast<RE::NiPoint3*>(&vertexBlock[j * vSize]);
                }

                body->m_vertices[j + vertexStart].m_skinPos = convertNi(*vertexPos);

                auto boneData = reinterpret_cast<SkyrimSystem::BoneData*>(&vertexBlock[j * vSize + boneOffset]);

#if defined(__AVX2__) || defined(__AVX512F__)
                // batch convert all 4 bone weights FP16 to FP32 through F16C hardware instruction
                __m128i halfWeights = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(boneData->boneWeights));
                _mm_storeu_ps(body->m_vertices[j + vertexStart].m_weight, _mm_cvtph_ps(halfWeights));
                // cleanse garbage NIF data for unused bones
                for (int k = partition->bonesPerVertex; k < 4; ++k)
                {
                    body->m_vertices[j + vertexStart].m_weight[k] = 0.0f;
                }
#else
                for (auto k = 0; std::cmp_less(k, partition->bonesPerVertex) && k < 4; ++k)
                {
                    __float32(&body->m_vertices[j + vertexStart].m_weight[k], boneData->boneWeights[k]);
                }
#endif

                for (auto k = 0; std::cmp_less(k, partition->bonesPerVertex) && k < 4; ++k)
                {
                    auto localBoneIndex = boneData->boneIndices[k];
                    assert(localBoneIndex < body->m_skinnedBones.size());
                    body->m_vertices[j + vertexStart].m_boneIdx[k] = localBoneIndex + boneStart;
                }
            }

            vertexOffsetMap.insert({meshName, vertexStart});
            boneStart = static_cast<int>(body->m_skinnedBones.size());
            vertexStart = static_cast<int>(body->m_vertices.size());
        }

        if (0 == vertexStart)
        {
            m_reader->skipCurrentElement();
            return {nullptr, {}};
        }

        for (auto& i : body->m_vertices)
        {
            i.sortWeight();
        }

        return {body, vertexOffsetMap};
    }

    auto SkyrimSystemCreator::readPerVertexShape(DefaultBBP::NameMap_t meshNameMap) -> RE::BSTSmartPointer<SkyrimBody>
    {
        auto name = m_reader->getAttribute("name");
        const auto it = meshNameMap.find(name);
        auto names = (it == meshNameMap.end()) ? DefaultBBP::NameSet_t({name}) : it->second;

        auto body = generateMeshBody(name, &names).first;
        if (!body)
        {
            return nullptr;
        }

        const auto shape = RE::make_smart<PerVertexShape>(body.get());

        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto nodeName = m_reader->GetName();
                if (nodeName == "priority")
                {
                    logger::warn("priority is deprecated and no longer used");
                    m_reader->skipCurrentElement();
                }
                else if (nodeName == "margin")
                {
                    shape->m_shapeProp.margin = m_reader->readFloat();
                }
                else if (nodeName == "shared")
                {
                    auto str = m_reader->readText();
                    if (str == "public")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
                    }
                    else if (str == "internal")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
                    }
                    else if (str == "external")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
                    }
                    else if (str == "private")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
                    }
                    else
                    {
                        logger::warn("unknown shared value, use default value \"public\"");
                        body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
                    }
                }
                else if (nodeName == "tag")
                {
                    body->m_tags.emplace_back(m_reader->readText());
                }
                else if (nodeName == "can-collide-with-tag")
                {
                    body->m_canCollideWithTags.insert(m_reader->readText());
                }
                else if (nodeName == "no-collide-with-tag")
                {
                    body->m_noCollideWithTags.insert(m_reader->readText());
                }
                else if (nodeName == "can-collide-with-bone")
                {
                    if (const auto bone = getOrCreateBone(m_reader->readText()))
                    {
                        body->m_canCollideWithBones.insert(bone);
                    }
                }
                else if (nodeName == "no-collide-with-bone")
                {
                    if (const auto bone = getOrCreateBone(m_reader->readText()))
                    {
                        body->m_noCollideWithBones.insert(bone);
                    }
                }
                else if (nodeName == "weight-threshold")
                {
                    auto boneName = m_reader->getAttribute("bone");
                    const float wt = m_reader->readFloat();
                    for (auto& m_skinnedBone : body->m_skinnedBones)
                    {
                        if (m_skinnedBone.ptr->m_name == getRenamedBone(boneName))
                        {
                            m_skinnedBone.weightThreshold = wt;
                            break;
                        }
                    }
                }
                else if (nodeName == "disable-tag")
                {
                    body->m_disableTag = m_reader->readText();
                }
                else if (nodeName == "disable-priority")
                {
                    body->m_disablePriority = m_reader->readInt();
                }

                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }

        m_deferredBuilds.emplace_back(body.get(), shape.get());

        return body;
    }

    auto SkyrimSystemCreator::readPerTriangleShape(DefaultBBP::NameMap_t* meshNameMap)
        -> RE::BSTSmartPointer<SkyrimBody>
    {
        auto name = m_reader->getAttribute("name");
        auto it = meshNameMap->find(name);
        auto names = (it == meshNameMap->end()) ? DefaultBBP::NameSet_t({name}) : it->second;

        auto bodyData = generateMeshBody(name, &names);
        auto body = bodyData.first;
        auto vertexOffsetMap = bodyData.second;
        if (!body)
        {
            return nullptr;
        }

        auto shape = RE::make_smart<PerTriangleShape>(body.get());

        for (const auto& entry : vertexOffsetMap)
        {
            auto* g = castBSTriShape(findObject(m_model, entry.first.c_str()));
            if (!g)
            {
                continue;
            }
            if (g->GetGeometryRuntimeData().skinInstance)
            {
                int offset = entry.second;
                RE::NiSkinPartition* skinPartition = g->GetGeometryRuntimeData().skinInstance->skinPartition.get();
                for (auto& partition : skinPartition->partitions)
                {
                    for (auto j = 0; std::cmp_less(j, partition.triangles); ++j)
                    {
                        shape->addTriangle(partition.triList[j * 3] + offset, partition.triList[j * 3 + 1] + offset,
                                           partition.triList[j * 3 + 2] + offset);
                    }
                }
            }
            else
            {
                logger::warn("Shape {} has no skin data, skipped", entry.first.c_str());
                return nullptr;
            }
        }

        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto nodeName = m_reader->GetName();
                if (nodeName == "priority")
                {
                    logger::warn("priority is deprecated and no longer used");
                    m_reader->skipCurrentElement();
                }
                else if (nodeName == "margin")
                {
                    shape->m_shapeProp.margin = m_reader->readFloat();
                }
                else if (nodeName == "shared")
                {
                    auto str = m_reader->readText();
                    if (str == "public")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
                    }
                    else if (str == "internal")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
                    }
                    else if (str == "external")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
                    }
                    else if (str == "private")
                    {
                        body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
                    }
                    else
                    {
                        logger::warn("unknown shared value, use default value \"public\"");
                        body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
                    }
                }
                else if (nodeName == "prenetration" || nodeName == "penetration")
                {
                    shape->m_shapeProp.penetration = m_reader->readFloat();
                }
                else if (nodeName == "tag")
                {
                    body->m_tags.emplace_back(m_reader->readText());
                }
                else if (nodeName == "no-collide-with-tag")
                {
                    body->m_noCollideWithTags.insert(m_reader->readText());
                }
                else if (nodeName == "can-collide-with-tag")
                {
                    body->m_canCollideWithTags.insert(m_reader->readText());
                }
                else if (nodeName == "can-collide-with-bone")
                {
                    if (auto bone = getOrCreateBone(m_reader->readText()))
                    {
                        body->m_canCollideWithBones.insert(bone);
                    }
                }
                else if (nodeName == "no-collide-with-bone")
                {
                    if (auto bone = getOrCreateBone(m_reader->readText()))
                    {
                        body->m_noCollideWithBones.insert(bone);
                    }
                }
                else if (nodeName == "weight-threshold")
                {
                    auto boneName = m_reader->getAttribute("bone");
                    float wt = m_reader->readFloat();
                    for (auto& m_skinnedBone : body->m_skinnedBones)
                    {
                        if (m_skinnedBone.ptr->m_name == getRenamedBone(boneName))
                        {
                            m_skinnedBone.weightThreshold = wt;
                        }
                    }
                }
                else if (nodeName == "disable-tag")
                {
                    body->m_disableTag = m_reader->readText();
                }
                else if (nodeName == "disable-priority")
                {
                    body->m_disablePriority = m_reader->readInt();
                }

                else
                {
                    logger::warn("unknown element - {}", nodeName.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }

        m_deferredBuilds.emplace_back(body.get(), nullptr);

        return body;
    }

    auto SkyrimSystemCreator::readFrameLerp(btTransform& tr) const -> void
    {
        tr.setIdentity();
        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();
                if (name == "translationLerp")
                {
                    tr.getOrigin().setX(m_reader->readFloat());
                }
                else if (name == "rotationLerp")
                {
                    tr.getOrigin().setY(m_reader->readFloat());
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
    }

    auto SkyrimSystemCreator::parseFrameType(const std::string& name, FrameType& frameType, btTransform& frame) const
        -> bool
    {
        if (name == "frameInA")
        {
            frameType = FrameType::FrameInA;
            frame = m_reader->readTransform();
        }
        else if (name == "frameInB")
        {
            frameType = FrameType::FrameInB;
            frame = m_reader->readTransform();
        }
        else if (name == "frameInLerp")
        {
            frameType = FrameType::FrameInLerp;
            readFrameLerp(frame);
        }
        else
        {
            return false;
        }
        return true;
    }

    auto SkyrimSystemCreator::readGenericConstraintTemplate(GenericConstraintTemplate& dest) const -> void
    {
        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();
                if (parseFrameType(name, dest.frameType, dest.frame))
                {
                }
                else if (name == "enableLinearSprings")
                {
                    dest.enableLinearSprings = m_reader->readBool();
                }
                else if (name == "enableAngularSprings")
                {
                    dest.enableAngularSprings = m_reader->readBool();
                }
                else if (name == "linearStiffnessLimited")
                {
                    dest.linearStiffnessLimited = m_reader->readBool();
                }
                else if (name == "angularStiffnessLimited")
                {
                    dest.angularStiffnessLimited = m_reader->readBool();
                }

                else if (name == "springDampingLimited")
                {
                    dest.springDampingLimited = m_reader->readBool();
                }
                else if (name == "linearNonHookeanDamping")
                {
                    dest.linearNonHookeanDamping = m_reader->readVector3();
                }
                else if (name == "angularNonHookeanDamping")
                {
                    dest.angularNonHookeanDamping = m_reader->readVector3();
                }
                else if (name == "linearNonHookeanStiffness")
                {
                    dest.linearNonHookeanStiffness = m_reader->readVector3();
                }
                else if (name == "angularNonHookeanStiffness")
                {
                    dest.angularNonHookeanStiffness = m_reader->readVector3();
                }

                else if (name == "linearMotors")
                {
                    dest.linearMotors = m_reader->readBool();
                }
                else if (name == "angularMotors")
                {
                    dest.angularMotors = m_reader->readBool();
                }
                else if (name == "linearServoMotors")
                {
                    dest.linearServoMotors = m_reader->readBool();
                }
                else if (name == "angularServoMotors")
                {
                    dest.angularServoMotors = m_reader->readBool();
                }
                else if (name == "linearTargetVelocity")
                {
                    dest.linearTargetVelocity = m_reader->readVector3();
                }
                else if (name == "angularTargetVelocity")
                {
                    dest.angularTargetVelocity = m_reader->readVector3();
                }
                else if (name == "linearMaxMotorForce")
                {
                    dest.linearMaxMotorForce = m_reader->readVector3();
                }
                else if (name == "angularMaxMotorForce")
                {
                    dest.angularMaxMotorForce = m_reader->readVector3();
                }

                else if (name == "stopERP")
                {
                    dest.stopERP = m_reader->readFloat();
                }
                else if (name == "stopCFM")
                {
                    dest.stopCFM = m_reader->readFloat();
                }
                else if (name == "motorERP")
                {
                    dest.motorERP = m_reader->readFloat();
                }
                else if (name == "motorCFM")
                {
                    dest.motorCFM = m_reader->readFloat();
                }

                else if (name == "useLinearReferenceFrameA")
                {
                    dest.useLinearReferenceFrameA = m_reader->readBool();
                }
                else if (name == "linearLowerLimit")
                {
                    dest.linearLowerLimit = m_reader->readVector3();
                }
                else if (name == "linearUpperLimit")
                {
                    dest.linearUpperLimit = m_reader->readVector3();
                }
                else if (name == "angularLowerLimit")
                {
                    dest.angularLowerLimit = m_reader->readVector3();
                }
                else if (name == "angularUpperLimit")
                {
                    dest.angularUpperLimit = m_reader->readVector3();
                }
                else if (name == "linearStiffness")
                {
                    dest.linearStiffness = m_reader->readVector3();
                }
                else if (name == "angularStiffness")
                {
                    dest.angularStiffness = m_reader->readVector3();
                }
                else if (name == "linearDamping")
                {
                    dest.linearDamping = m_reader->readVector3();
                }
                else if (name == "angularDamping")
                {
                    dest.angularDamping = m_reader->readVector3();
                }
                else if (name == "linearEquilibrium")
                {
                    dest.linearEquilibrium = m_reader->readVector3();
                }
                else if (name == "angularEquilibrium")
                {
                    dest.angularEquilibrium = m_reader->readVector3();
                }
                else if (name == "linearBounce")
                {
                    dest.linearBounce = m_reader->readVector3();
                }
                else if (name == "angularBounce")
                {
                    dest.angularBounce = m_reader->readVector3();
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
    }

    auto SkyrimSystemCreator::findBones(const RE::BSFixedString& bodyAName, const RE::BSFixedString& bodyBName,
                                        SkyrimBone*& bodyA, SkyrimBone*& bodyB) -> bool
    {
        bodyA = findBoneFromIndex(bodyAName);
        bodyB = findBoneFromIndex(bodyBName);

        if (!bodyA)
        {
            logger::warn("constraint {} <-> {} : bone for bodyA doesn't exist, will try to create it",
                         bodyAName.c_str(), bodyBName.c_str());
            bodyA = createBoneFromNodeName(bodyAName);
            if (!bodyA)
            {
                m_reader->skipCurrentElement();
                return false;
            }
        }
        if (!bodyB)
        {
            logger::warn("constraint {} <-> {} : bone for bodyB doesn't exist, will try to create it",
                         bodyAName.c_str(), bodyBName.c_str());
            bodyB = createBoneFromNodeName(bodyBName);
            if (!bodyB)
            {
                m_reader->skipCurrentElement();
                return false;
            }
        }
        if (bodyA == bodyB)
        {
            logger::warn("constraint between same object {} <-> {}, skipped", bodyAName.c_str(), bodyBName.c_str());
            m_reader->skipCurrentElement();
            return false;
        }

        if (bodyA->m_rig.isKinematicObject() && bodyB->m_rig.isKinematicObject())
        {
            logger::warn("constraint between two kinematic object {} <-> {}, skipped", bodyAName.c_str(),
                         bodyBName.c_str());
            m_reader->skipCurrentElement();
            return false;
        }

        logger::info("OK: constraint between object {} <-> {}", bodyAName.c_str(), bodyBName.c_str());
        return true;
    }

    namespace
    {
        auto rotFromAtoB(const btVector3& a, const btVector3& b) -> btQuaternion
        {
            const auto axis = a.cross(b);
            if (axis.fuzzyZero())
            {
                return btQuaternion::getIdentity();
            }
            const float sinA = axis.length();
            const float cosA = a.dot(b);
            const float angle = btAtan2(cosA, sinA);
            return {axis, angle};
        }
    } // namespace

    auto SkyrimSystemCreator::calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA,
                                        const btQsTransform& trB, btTransform& frameA, btTransform& frameB) -> void
    {
        btQsTransform frameInWorld;
        switch (type)
        {
        case FrameType::FrameInA:
            frameA = frame;
            frameInWorld = trA * frame;
            frameB = (trB.inverse() * frameInWorld).asTransform();
            break;
        case FrameType::FrameInB:
            frameB = frame;
            frameInWorld = trB * frameB;
            frameA = (trA.inverse() * frameInWorld).asTransform();
            break;
        case FrameType::FrameInLerp:
        {
            const auto trans = trA.getOrigin().lerp(trB.getOrigin(), frame.getOrigin().x());
            const auto rot = trA.getBasis().slerp(trB.getBasis(), frame.getOrigin().y());
            frameInWorld = btQsTransform(rot, trans);
            frameA = (trA.inverse() * frameInWorld).asTransform();
            frameB = (trB.inverse() * frameInWorld).asTransform();
            break;
        }
        case FrameType::AWithXPointToB:
        {
            const btMatrix3x3 matr(trA.getBasis());
            frameInWorld = trA;
            const auto old = matr.getColumn(0).normalized();
            const auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
            const auto q = rotFromAtoB(old, a2b);
            frameInWorld.getBasis() *= q;
            frameA = (trA.inverse() * frameInWorld).asTransform();
            frameB = (trB.inverse() * frameInWorld).asTransform();
            break;
        }
        case FrameType::AWithYPointToB:
        {
            const btMatrix3x3 matr(trA.getBasis());
            frameInWorld = trA;
            const auto old = matr.getColumn(1).normalized();
            const auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
            const auto q = rotFromAtoB(old, a2b);
            frameInWorld.getBasis() *= q;
            frameA = (trA.inverse() * frameInWorld).asTransform();
            frameB = (trB.inverse() * frameInWorld).asTransform();
            break;
        }
        case FrameType::AWithZPointToB:
        {
            const btMatrix3x3 matr(trA.getBasis());
            frameInWorld = trA;
            const auto old = matr.getColumn(2).normalized();
            const auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
            const auto q = rotFromAtoB(old, a2b);
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

        SkyrimBone* bodyA;
        SkyrimBone* bodyB;
        if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
        {
            return nullptr;
        }

        auto trA = bodyA->m_currentTransform;
        auto trB = bodyB->m_currentTransform;

        auto cinfo = getGenericConstraintTemplate(clsname);
        readGenericConstraintTemplate(cinfo);
        btTransform frameA;
        btTransform frameB;
        calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

        RE::BSTSmartPointer<Generic6DofConstraint> constraint;
        if (cinfo.useLinearReferenceFrameA)
        {
            constraint = RE::make_smart<Generic6DofConstraint>(bodyB, bodyA, frameB, frameA);
        }
        else
        {
            constraint = RE::make_smart<Generic6DofConstraint>(bodyA, bodyB, frameA, frameB);
        }

        constraint->setLinearLowerLimit(cinfo.linearLowerLimit);
        constraint->setLinearUpperLimit(cinfo.linearUpperLimit);
        constraint->setAngularLowerLimit(cinfo.angularLowerLimit);
        constraint->setAngularUpperLimit(cinfo.angularUpperLimit);
        for (auto i = 0; i < 3; ++i)
        {
            constraint->setStiffness(i, cinfo.linearStiffness[i], cinfo.linearStiffnessLimited);
            constraint->setStiffness(i + 3, cinfo.angularStiffness[i], cinfo.angularStiffnessLimited);
            constraint->setDamping(i, cinfo.linearDamping[i], cinfo.springDampingLimited);
            constraint->setDamping(i + 3, cinfo.angularDamping[i], cinfo.springDampingLimited);

            constraint->setEquilibriumPoint(i, cinfo.linearEquilibrium[i]);
            constraint->setEquilibriumPoint(i + 3, cinfo.angularEquilibrium[i]);

            constraint->setNonHookeanDamping(i, cinfo.linearNonHookeanDamping[i]);
            constraint->setNonHookeanDamping(i + 3, cinfo.angularNonHookeanDamping[i]);
            constraint->setNonHookeanStiffness(i, cinfo.linearNonHookeanStiffness[i]);
            constraint->setNonHookeanStiffness(i + 3, cinfo.angularNonHookeanStiffness[i]);

            constraint->enableSpring(i, cinfo.enableLinearSprings);
            constraint->enableSpring(i + 3, cinfo.enableAngularSprings);

            constraint->enableMotor(i, cinfo.linearMotors);
            constraint->enableMotor(i + 3, cinfo.angularMotors);
            constraint->setServo(i, cinfo.linearServoMotors);
            constraint->setServo(i + 3, cinfo.angularServoMotors);
            // TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server
            // motor target.  Hard coded to equilibrium right now.
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

            if (auto rotMotor = constraint->getRotationalLimitMotor(i))
            {
                rotMotor->m_motorERP = cinfo.motorERP;
                rotMotor->m_motorCFM = cinfo.motorCFM;
                rotMotor->m_stopERP = cinfo.stopERP;
                rotMotor->m_stopCFM = cinfo.stopCFM;
            }
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

    auto SkyrimSystemCreator::readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest) const -> void
    {
        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();
                if (name == "minDistanceFactor")
                {
                    dest.minDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
                }
                else if (name == "maxDistanceFactor")
                {
                    dest.maxDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
                }
                else if (name == "stiffness")
                {
                    dest.stiffness = std::max(m_reader->readFloat(), 0.0f);
                }
                else if (name == "damping")
                {
                    dest.damping = std::max(m_reader->readFloat(), 0.0f);
                }
                else if (name == "equilibrium")
                {
                    dest.equilibriumFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
    }

    auto SkyrimSystemCreator::readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest) const -> void
    {
        while (m_reader->Inspect())
        {
            if (m_reader->GetInspected() == XMLReader::Inspected::StartTag)
            {
                auto name = m_reader->GetName();
                if (parseFrameType(name, dest.frameType, dest.frame))
                {
                }
                else if (name == "swingSpan1" || name == "coneLimit" || name == "limitZ")
                {
                    dest.swingSpan1 = std::max(m_reader->readFloat(), 0.f);
                }
                else if (name == "swingSpan2" || name == "planeLimit" || name == "limitY")
                {
                    dest.swingSpan2 = std::max(m_reader->readFloat(), 0.f);
                }
                else if (name == "twistSpan" || name == "twistLimit" || name == "limitX")
                {
                    dest.twistSpan = std::max(m_reader->readFloat(), 0.f);
                }
                else if (name == "limitSoftness")
                {
                    dest.limitSoftness = btClamped(m_reader->readFloat(), 0.f, 1.f);
                }
                else if (name == "biasFactor")
                {
                    dest.biasFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
                }
                else if (name == "relaxationFactor")
                {
                    dest.relaxationFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
                }
                else
                {
                    logger::warn("unknown element - {}", name.c_str());
                    m_reader->skipCurrentElement();
                }
            }
            else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
            {
                break;
            }
        }
    }

    auto SkyrimSystemCreator::getBoneTemplate(const RE::BSFixedString& name) -> const BoneTemplate&
    {
        const auto iter = m_boneTemplates.find(name);
        if (iter == m_boneTemplates.end())
        {
            return m_boneTemplates[RE::BSFixedString()];
        }
        return iter->second;
    }

    auto SkyrimSystemCreator::getGenericConstraintTemplate(const RE::BSFixedString& name)
        -> const GenericConstraintTemplate&
    {
        const auto iter = m_genericConstraintTemplates.find(name);
        if (iter == m_genericConstraintTemplates.end())
        {
            return m_genericConstraintTemplates[RE::BSFixedString()];
        }
        return iter->second;
    }

    auto SkyrimSystemCreator::getStiffSpringConstraintTemplate(const RE::BSFixedString& name)
        -> const StiffSpringConstraintTemplate&
    {
        const auto iter = m_stiffSpringConstraintTemplates.find(name);
        if (iter == m_stiffSpringConstraintTemplates.end())
        {
            return m_stiffSpringConstraintTemplates[RE::BSFixedString()];
        }
        return iter->second;
    }

    auto SkyrimSystemCreator::getConeTwistConstraintTemplate(const RE::BSFixedString& name)
        -> const ConeTwistConstraintTemplate&
    {
        const auto iter = m_coneTwistConstraintTemplates.find(name);
        if (iter == m_coneTwistConstraintTemplates.end())
        {
            return m_coneTwistConstraintTemplates[RE::BSFixedString()];
        }
        return iter->second;
    }

    auto SkyrimSystemCreator::readStiffSpringConstraint() -> RE::BSTSmartPointer<StiffSpringConstraint>
    {
        const auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
        const auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
        const auto clsname = m_reader->getAttribute("template", "");

        SkyrimBone* bodyA;
        SkyrimBone* bodyB;
        if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
        {
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
        const auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
        const auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
        const auto clsname = m_reader->getAttribute("template", "");

        SkyrimBone* bodyA = nullptr;
        SkyrimBone* bodyB = nullptr;
        if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
        {
            return nullptr;
        }

        const auto trA = bodyA->m_currentTransform;
        const auto trB = bodyB->m_currentTransform;

        auto cinfo = getConeTwistConstraintTemplate(clsname);
        readConeTwistConstraintTemplate(cinfo);
        btTransform frameA;
        btTransform frameB;
        calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

        RE::BSTSmartPointer<ConeTwistConstraint> constraint =
            RE::make_smart<ConeTwistConstraint>(bodyA, bodyB, frameA, frameB);
        constraint->setLimit(cinfo.swingSpan1, cinfo.swingSpan2, cinfo.twistSpan, cinfo.limitSoftness, cinfo.biasFactor,
                             cinfo.relaxationFactor);

        return constraint;
    }
} // namespace hdt
