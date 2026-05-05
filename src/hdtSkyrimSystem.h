#pragma once

#include "hdtConvertNi.h"
#include "hdtDefaultBBP.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"
#include "hdtSkyrimBody.h"
#include "hdtSkyrimBone.h"

namespace hdt
{
    class PerVertexShape;

    class SkyrimSystem : public SkinnedMeshSystem
    {
        friend class SkyrimSystemCreator;

    public:
        struct BoneData
        {
            uint16_t boneWeights[4];
            uint8_t boneIndices[4];
        };

        SkyrimSystem(RE::NiNode* skeleton);
        ~SkyrimSystem() override = default;

        auto findBone(const RE::BSFixedString& name) const -> SkinnedMeshBone*;
        auto findBody(const RE::BSFixedString& name) const -> SkinnedMeshBody*;
        auto findBoneIdx(const RE::BSFixedString& name) const -> int;

        auto prepareForRead(float timeStep) -> float override;

        auto meshes() const -> const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& { return m_meshes; }

        RE::NiPointer<RE::NiNode> m_skeleton;
        RE::NiPointer<RE::NiNode> m_oldRoot;
        bool m_initialized = false;
        float m_windFactor = 1.f;
        // wind factor for the system (i.e., full actor/skeleton) (calculated based off obstructions)

        // angular velocity damper
        btQuaternion m_lastRootRotation;
    };

    class XMLReader;

    class SkyrimSystemCreator
    {
    public:
        SkyrimSystemCreator() = default;

        auto createOrUpdateSystem(RE::NiNode* skeleton, RE::NiAVObject* model, DefaultBBP::PhysicsFile_t* file,
                                  std::unordered_map<RE::BSFixedString, RE::BSFixedString>&& renameMap,
                                  SkyrimSystem* old_system) -> RE::BSTSmartPointer<SkyrimSystem>;

    protected:
        // O(1) bone lookup index. These are just to speed up the hashmap more since BSStrings are pooled
        struct PooledStringHash
        {
            auto operator()(const char* p) const noexcept -> size_t { return reinterpret_cast<size_t>(p); }
        };

        struct PooledStringEqual
        {
            auto operator()(const char* a, const char* b) const noexcept -> bool { return a == b; }
        };

        std::unordered_map<const char*, SkyrimBone*, PooledStringHash, PooledStringEqual> m_boneIndex;

        auto indexBone(SkyrimBone* bone) -> void;
        auto findBoneFromIndex(const RE::BSFixedString& name) const -> SkyrimBone*;

        struct DeferredBuild
        {
            SkinnedMeshBody* body;
            PerVertexShape* vertexShape;
        };

        std::vector<DeferredBuild> m_deferredBuilds;

        struct BoneTemplate : btRigidBody::btRigidBodyConstructionInfo
        {
            static btEmptyShape emptyShape[1];

            BoneTemplate() : btRigidBodyConstructionInfo(0, nullptr, emptyShape)
            {
                m_centerOfMassTransform = btTransform::getIdentity();
                m_marginMultipler = 1.f;
            }

            std::shared_ptr<btCollisionShape> m_shape;
            std::vector<RE::BSFixedString> m_canCollideWithBone;
            std::vector<RE::BSFixedString> m_noCollideWithBone;
            btTransform m_centerOfMassTransform;
            float m_marginMultipler;
            float m_gravityFactor = 1.0f;
            float m_windFactor = 1.0f;
            U32 m_collisionFilter = 0;
        };

        enum struct FrameType : uint8_t
        {
            FrameInA,
            FrameInB,
            FrameInLerp,
            AWithXPointToB,
            AWithYPointToB,
            AWithZPointToB
        };

        struct GenericConstraintTemplate
        {
            FrameType frameType = FrameType::FrameInB;
            bool useLinearReferenceFrameA = false;
            btTransform frame = btTransform::getIdentity();
            btVector3 linearLowerLimit = btVector3(1, 1, 1);
            btVector3 linearUpperLimit = btVector3(-1, -1, -1);
            btVector3 angularLowerLimit = btVector3(1, 1, 1);
            btVector3 angularUpperLimit = btVector3(-1, -1, -1);
            btVector3 linearStiffness = btVector3(0, 0, 0);
            btVector3 angularStiffness = btVector3(0, 0, 0);
            btVector3 linearDamping = btVector3(0, 0, 0);
            btVector3 angularDamping = btVector3(0, 0, 0);
            btVector3 linearEquilibrium = btVector3(0, 0, 0);
            btVector3 angularEquilibrium = btVector3(0, 0, 0);
            btVector3 linearBounce = btVector3(0, 0, 0);
            btVector3 angularBounce = btVector3(0, 0, 0);
            bool enableLinearSprings = true;
            bool enableAngularSprings = true;
            bool linearStiffnessLimited = true;
            bool angularStiffnessLimited = true;
            bool springDampingLimited = true;
            bool linearMotors = false;
            bool angularMotors = false;
            // TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server
            // motor target.  Hard coded to equilibrium right now.
            bool linearServoMotors = false;
            bool angularServoMotors = false;
            btVector3 linearNonHookeanDamping = btVector3(0, 0, 0);
            btVector3 angularNonHookeanDamping = btVector3(0, 0, 0);
            btVector3 linearNonHookeanStiffness = btVector3(0, 0, 0);
            btVector3 angularNonHookeanStiffness = btVector3(0, 0, 0);
            btVector3 linearTargetVelocity = btVector3(0, 0, 0);
            btVector3 angularTargetVelocity = btVector3(0, 0, 0);
            btVector3 linearMaxMotorForce = btVector3(0, 0, 0);
            btVector3 angularMaxMotorForce = btVector3(0, 0, 0);
            btScalar motorERP = 0.9f;
            btScalar motorCFM = 0;
            btScalar stopERP = 0.2f;
            btScalar stopCFM = 0;
        };

        struct StiffSpringConstraintTemplate
        {
            float minDistanceFactor = 1;
            float maxDistanceFactor = 1;
            float stiffness = 0;
            float damping = 0;
            float equilibriumFactor = 0.5;
        };

        struct ConeTwistConstraintTemplate
        {
            btTransform frame = btTransform::getIdentity();
            FrameType frameType = FrameType::FrameInB;
            float swingSpan1 = 0;
            float swingSpan2 = 0;
            float twistSpan = 0;
            float limitSoftness = 1.0f;
            float biasFactor = 0.3f;
            float relaxationFactor = 1.0f;
        };

        using VertexOffsetMap = std::unordered_map<std::string, int>;

        auto getRenamedBone(const RE::BSFixedString& name) -> RE::BSFixedString;

        RE::BSTSmartPointer<SkyrimSystem> m_mesh;
        RE::NiNode* m_skeleton;
        RE::NiAVObject* m_model;
        XMLReader* m_reader;
        std::unordered_map<RE::BSFixedString, RE::BSFixedString> m_renameMap;

        auto findObjectByName(const RE::BSFixedString& name) const -> RE::NiNode*;
        auto getOrCreateBone(const RE::BSFixedString& name) -> SkyrimBone*;

        std::string m_filePath;

        std::unordered_map<RE::BSFixedString, BoneTemplate> m_boneTemplates;
        std::unordered_map<RE::BSFixedString, GenericConstraintTemplate> m_genericConstraintTemplates;
        std::unordered_map<RE::BSFixedString, StiffSpringConstraintTemplate> m_stiffSpringConstraintTemplates;
        std::unordered_map<RE::BSFixedString, ConeTwistConstraintTemplate> m_coneTwistConstraintTemplates;
        std::unordered_map<RE::BSFixedString, std::shared_ptr<btCollisionShape>> m_shapes;
        std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;

        auto generateMeshBody(std::string_view name, DefaultBBP::NameSet_t* names)
            -> std::pair<RE::BSTSmartPointer<SkyrimBody>, VertexOffsetMap>;

        auto findBones(const RE::BSFixedString& bodyAName, const RE::BSFixedString& bodyBName, SkyrimBone*& bodyA,
                       SkyrimBone*& bodyB) -> bool;
        auto parseFrameType(const std::string& name, FrameType& type, btTransform& frame) const -> bool;
        static auto calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA,
                              const btQsTransform& trB, btTransform& frameA, btTransform& frameB) -> void;
        auto readFrameLerp(btTransform& tr) const -> void;
        auto readBoneTemplate(BoneTemplate& dest) -> void;
        auto readGenericConstraintTemplate(GenericConstraintTemplate& dest) const -> void;
        auto readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest) const -> void;
        auto readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest) const -> void;

        auto getBoneTemplate(const RE::BSFixedString& name) -> const BoneTemplate&;
        auto getGenericConstraintTemplate(const RE::BSFixedString& name) -> const GenericConstraintTemplate&;
        auto getStiffSpringConstraintTemplate(const RE::BSFixedString& name) -> const StiffSpringConstraintTemplate&;
        auto getConeTwistConstraintTemplate(const RE::BSFixedString& name) -> const ConeTwistConstraintTemplate&;

        auto createBoneFromNodeName(const RE::BSFixedString& bodyName, const RE::BSFixedString& templateName = "",
                                    bool readTemplate = false) -> SkyrimBone*;
        auto readOrUpdateBone() -> void;
        auto readPerVertexShape(DefaultBBP::NameMap_t meshNameMap) -> RE::BSTSmartPointer<SkyrimBody>;
        auto readPerTriangleShape(DefaultBBP::NameMap_t* meshNameMap) -> RE::BSTSmartPointer<SkyrimBody>;
        auto readGenericConstraint() -> RE::BSTSmartPointer<Generic6DofConstraint>;
        auto readStiffSpringConstraint() -> RE::BSTSmartPointer<StiffSpringConstraint>;
        auto readConeTwistConstraint() -> RE::BSTSmartPointer<ConeTwistConstraint>;
        auto readConstraintGroup() -> RE::BSTSmartPointer<ConstraintGroup>;
        auto readShape() -> std::shared_ptr<btCollisionShape>;
    };
} // namespace hdt
